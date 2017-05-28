/* -*- Mode: ObjC; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * ***** BEGIN LICENSE BLOCK *****
 * Version: BSD
 *
 * Copyright (C) 2006-2009 Mozilla Corporation.  All rights reserved.
 *
 * Contributor(s):
 *   Vladimir Vukicevic <vladimir@pobox.com>
 *   Masayuki Nakano <masayuki@d-toybox.com>
 *   John Daggett <jdaggett@mozilla.com>
 *   Jonathan Kew <jfkthame@gmail.com>
 *
 * Copyright (C) 2006 Apple Computer, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1.  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 2.  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * 3.  Neither the name of Apple Computer, Inc. ("Apple") nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE AND ITS CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL APPLE OR ITS CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ***** END LICENSE BLOCK ***** */

#ifdef MOZ_LOGGING
#define FORCE_PR_LOG /* Allow logging in the release build */
#endif
#include "prlog.h"

#include <algorithm>

#import <AppKit/AppKit.h>

#include "gfxPlatformMac.h"
#include "gfxMacPlatformFontList.h"
#include "gfxMacFont.h"
#include "gfxUserFontSet.h"
#include "harfbuzz/hb.h"

#include "nsServiceManagerUtils.h"
#include "nsTArray.h"

#include "nsDirectoryServiceUtils.h"
#include "nsDirectoryServiceDefs.h"
#include "nsISimpleEnumerator.h"
#include "nsCharTraits.h"
#include "nsCocoaFeatures.h"
#include "gfxFontConstants.h"

#include "mozilla/MemoryReporting.h"
#include "mozilla/Preferences.h"
#include "mozilla/Telemetry.h"
#include "mozilla/gfx/2D.h"

#include <unistd.h>
#include <time.h>

using namespace mozilla;

class nsAutoreleasePool {
public:
    nsAutoreleasePool()
    {
        mLocalPool = [[NSAutoreleasePool alloc] init];
    }
    ~nsAutoreleasePool()
    {
        [mLocalPool release];
    }
private:
    NSAutoreleasePool *mLocalPool;
};

// indexes into the NSArray objects that the Cocoa font manager returns
// as the available members of a family
#define INDEX_FONT_POSTSCRIPT_NAME 0
#define INDEX_FONT_FACE_NAME 1
#define INDEX_FONT_WEIGHT 2
#define INDEX_FONT_TRAITS 3

static const int kAppleMaxWeight = 14;
static const int kAppleExtraLightWeight = 3;
static const int kAppleUltraLightWeight = 2;

static const int gAppleWeightToCSSWeight[] = {
    0,
    1, // 1.
    1, // 2.  W1, ultralight
    2, // 3.  W2, extralight
    3, // 4.  W3, light
    4, // 5.  W4, semilight
    5, // 6.  W5, medium
    6, // 7.
    6, // 8.  W6, semibold
    7, // 9.  W7, bold
    8, // 10. W8, extrabold
    8, // 11.
    9, // 12. W9, ultrabold
    9, // 13
    9  // 14
};

// cache Cocoa's "shared font manager" for performance
static NSFontManager *sFontManager;

static void GetStringForNSString(const NSString *aSrc, nsAString& aDist)
{
    aDist.SetLength([aSrc length]);
    [aSrc getCharacters:reinterpret_cast<unichar*>(aDist.BeginWriting())];
}

static NSString* GetNSStringForString(const nsAString& aSrc)
{
    return [NSString stringWithCharacters:reinterpret_cast<const unichar*>(aSrc.BeginReading())
                                   length:aSrc.Length()];
}

#ifdef PR_LOGGING

#define LOG_FONTLIST(args) PR_LOG(gfxPlatform::GetLog(eGfxLog_fontlist), \
                               PR_LOG_DEBUG, args)
#define LOG_FONTLIST_ENABLED() PR_LOG_TEST( \
                                   gfxPlatform::GetLog(eGfxLog_fontlist), \
                                   PR_LOG_DEBUG)
#define LOG_CMAPDATA_ENABLED() PR_LOG_TEST( \
                                   gfxPlatform::GetLog(eGfxLog_cmapdata), \
                                   PR_LOG_DEBUG)

#endif // PR_LOGGING

#pragma mark-

// Complex scripts will not render correctly unless appropriate AAT or OT
// layout tables are present.
// For OpenType, we also check that the GSUB table supports the relevant
// script tag, to avoid using things like Arial Unicode MS for Lao (it has
// the characters, but lacks OpenType support).

// TODO: consider whether we should move this to gfxFontEntry and do similar
// cmap-masking on other platforms to avoid using fonts that won't shape
// properly.

nsresult
MacOSFontEntry::ReadCMAP(FontInfoData *aFontInfoData)
{
    // attempt this once, if errors occur leave a blank cmap
    if (mCharacterMap) {
        return NS_OK;
    }

    // If there is no glyf or CFF table, Harfbuzz will choke on this font.
    // See 10.4Fx issue 171.
    if(!HasFontTable(TRUETYPE_TAG('g','l','y','f')) &&
       !HasFontTable(TRUETYPE_TAG('C','F','F',' '))) {
       fprintf(stderr, "Warning: TenFourFox rejecting bitmap-only font %s.\n",
                 NS_ConvertUTF16toUTF8(mName).get());
       mCharacterMap = new gfxCharacterMap(); // no characters
       return NS_OK;
    }

    nsRefPtr<gfxCharacterMap> charmap;
    nsresult rv;
    bool symbolFont;

    if (aFontInfoData && (charmap = GetCMAPFromFontInfo(aFontInfoData,
                                                        mUVSOffset,
                                                        symbolFont))) {
        rv = NS_OK;
    } else {
        uint32_t kCMAP = TRUETYPE_TAG('c','m','a','p');
        charmap = new gfxCharacterMap();
        AutoTable cmapTable(this, kCMAP);

        if (cmapTable) {
            bool unicodeFont = false, symbolFont = false; // currently ignored
            uint32_t cmapLen;
            const uint8_t* cmapData =
                reinterpret_cast<const uint8_t*>(hb_blob_get_data(cmapTable,
                                                                  &cmapLen));
            rv = gfxFontUtils::ReadCMAP(cmapData, cmapLen,
                                        *charmap, mUVSOffset,
                                        unicodeFont, symbolFont);
        } else {
            rv = NS_ERROR_NOT_AVAILABLE;
        }
    }

    if (NS_SUCCEEDED(rv) && !HasGraphiteTables()) {
        // We assume a Graphite font knows what it's doing,
        // and provides whatever shaping is needed for the
        // characters it supports, so only check/clear the
        // complex-script ranges for non-Graphite fonts

        // for layout support, check for the presence of mort/morx and/or
        // opentype layout tables
        bool hasAATLayout = HasFontTable(TRUETYPE_TAG('m','o','r','x')) ||
                            HasFontTable(TRUETYPE_TAG('m','o','r','t'));
        bool hasGSUB = HasFontTable(TRUETYPE_TAG('G','S','U','B'));
        bool hasGPOS = HasFontTable(TRUETYPE_TAG('G','P','O','S'));
        if (hasAATLayout && !(hasGSUB || hasGPOS)) {
            mRequiresAAT = true; // prefer CoreText if font has no OTL tables
        }

        for (const ScriptRange* sr = gfxPlatformFontList::sComplexScriptRanges;
             sr->rangeStart; sr++) {
            // check to see if the cmap includes complex script codepoints
            if (charmap->TestRange(sr->rangeStart, sr->rangeEnd)) {
                if (hasAATLayout) {
                    // prefer CoreText for Apple's complex-script fonts,
                    // even if they also have some OpenType tables
                    // (e.g. Geeza Pro Bold on 10.6; see bug 614903)
                    mRequiresAAT = true;
                    // and don't mask off complex-script ranges, we assume
                    // the AAT tables will provide the necessary shaping
                    continue;
                }

                // We check for GSUB here, as GPOS alone would not be ok.
                if (hasGSUB && SupportsScriptInGSUB(sr->tags)) {
                    continue;
                }

                charmap->ClearRange(sr->rangeStart, sr->rangeEnd);
            }
        }
    }

    mHasCmapTable = NS_SUCCEEDED(rv);
    if (mHasCmapTable) {
        gfxPlatformFontList *pfl = gfxPlatformFontList::PlatformFontList();
        mCharacterMap = pfl->FindCharMap(charmap);
    } else {
        // if error occurred, initialize to null cmap
        mCharacterMap = new gfxCharacterMap();
    }

#ifdef PR_LOGGING
    LOG_FONTLIST(("(fontlist-cmap) name: %s, size: %d hash: %8.8x%s\n",
                  NS_ConvertUTF16toUTF8(mName).get(),
                  charmap->SizeOfIncludingThis(moz_malloc_size_of),
                  charmap->mHash, mCharacterMap == charmap ? " new" : ""));
    if (LOG_CMAPDATA_ENABLED()) {
        char prefix[256];
        sprintf(prefix, "(cmapdata) name: %.220s",
                NS_ConvertUTF16toUTF8(mName).get());
        charmap->Dump(prefix, eGfxLog_cmapdata);
    }
#endif

    return rv;
}

gfxFont*
MacOSFontEntry::CreateFontInstance(const gfxFontStyle *aFontStyle, bool aNeedsBold)
{
    return new gfxMacFont(this, aFontStyle, aNeedsBold);
}

bool
MacOSFontEntry::IsCFF()
{
    if (!mIsCFFInitialized) {
        mIsCFFInitialized = true;
        mIsCFF = HasFontTable(TRUETYPE_TAG('C','F','F',' '));
    }

    return mIsCFF;
}

MacOSFontEntry::MacOSFontEntry(const nsAString& aPostscriptName,
                               int32_t aWeight,
                               bool aIsStandardFace)
    : gfxFontEntry(aPostscriptName, aIsStandardFace),
      mFontRef(NULL),
      mFontRefInitialized(false),
      mRequiresAAT(false),
      mIsCFF(false),
      mIsCFFInitialized(false)
      , mATSFontRef(kInvalidFont),
      mFontTableDirSize(0),
      mContainerRef(NULL),
      mATSFontRefInitialized(false) /* 10.4Fx */
{
    mWeight = aWeight;
}

MacOSFontEntry::MacOSFontEntry(const nsAString& aPostscriptName,
#if(0)
                               CGFontRef aFontRef,
#else
                               ATSFontRef aFontRef, // 10.4Fx
#endif
                               uint16_t aWeight, uint16_t aStretch,
                               uint32_t aItalicStyle,
			       ATSFontContainerRef aContainerRef, // 10.4Fx
                               bool aIsUserFont, bool aIsLocal)
    : gfxFontEntry(aPostscriptName, false),
      mFontRef(NULL),
      mFontRefInitialized(false),
      mRequiresAAT(false),
      mIsCFF(false),
      mIsCFFInitialized(false)
{
#if(0)
    mFontRef = aFontRef;
    mFontRefInitialized = true;
    ::CFRetain(mFontRef);
#else
    // ATSFontRef version
    // We don't retain mFontRef here because we synthesize it.
    mATSFontRef = aFontRef;
    mATSFontRefInitialized = true; // keep mFontRef as if not initialized
    mContainerRef = aContainerRef;
    mFontTableDirSize = 0;
#endif

    mWeight = aWeight;
    mStretch = aStretch;
    mFixedPitch = false; // xxx - do we need this for downloaded fonts?
    mItalic = (aItalicStyle & (NS_FONT_STYLE_ITALIC | NS_FONT_STYLE_OBLIQUE)) != 0;
    mIsUserFont = aIsUserFont;
    mIsLocalUserFont = aIsLocal;
}

#if(0)
CGFontRef
MacOSFontEntry::GetFontRef()
{
    if (!mFontRefInitialized) {
        mFontRefInitialized = true;
        NSString *psname = GetNSStringForString(mName);
        mFontRef = ::CGFontCreateWithFontName(CFStringRef(psname));
    }
    return mFontRef;
}
#else
/* Define ATSFontRef and CGFontRef getters for 10.4/5. */
ATSFontRef
MacOSFontEntry::GetATSFontRef()
{
    if (!mATSFontRefInitialized) {
        mATSFontRefInitialized = true;
        NSString *psname = GetNSStringForString(mName);
        mATSFontRef = ::ATSFontFindFromPostScriptName(CFStringRef(psname),
                                                      kATSOptionFlagsDefault);
    }
    return mATSFontRef;
}
CGFontRef
MacOSFontEntry::GetFontRef()
{
    if (mFontRefInitialized) {
        return mFontRef;
    }

    // GetATSFontRef will initialize mATSFontRef
    if (GetATSFontRef() == kInvalidFont) {
        return nullptr;
    }
    
    mFontRef = ::CGFontCreateWithPlatformFont(&mATSFontRef);
    // Per Apple, we need to release this later with CGFontRelease. See
// https://developer.apple.com/library/mac/documentation/graphicsimaging/reference/CGFont/DeprecationAppendix/AppendixADeprecatedAPI.html
    mFontRefInitialized = true;

    return mFontRef;
}
#endif

/*static*/ void
MacOSFontEntry::DestroyBlobFunc(void* aUserData)
{
    ::CFRelease((CFDataRef)aUserData);
}

hb_blob_t *
MacOSFontEntry::GetFontTable(uint32_t aTag)
{
#if(0)
    CGFontRef fontRef = GetFontRef();
    if (!fontRef) {
        return nullptr;
    }

    CFDataRef dataRef = ::CGFontCopyTableForTag(fontRef, aTag);
    if (dataRef) {
        return hb_blob_create((const char*)::CFDataGetBytePtr(dataRef),
                              ::CFDataGetLength(dataRef),
                              HB_MEMORY_MODE_READONLY,
                              (void*)dataRef, DestroyBlobFunc);
    }

    return nullptr;
#else
    // ATSFontRef version
    // This is based on the high probability we called HasFontTable() before
    // we called this to actually get it; HasFontTable() will cache the
    // directory for us.
    nsAutoreleasePool localPool;

    ATSFontRef fontRef = GetATSFontRef();
    if (fontRef == kInvalidFont) return nullptr;

    ByteCount dataLength = 0;
    
    // See if we already know how long the table is. This saves a potentially
    // expensive call to ATSGetFontTable() to simply get the length.
    // Essentially a hardcoded form of FindTagInTableDir; see below.
    if (MOZ_LIKELY(mFontTableDirSize > 0)) {
        // XXX: This assumes big endian (warning Intel)
#ifndef __ppc__
#error needs GetFontTable fast path needs little endian version
#endif

#ifdef DEBUG
        uint32_t j = 12;
        uint8_t *table = (reinterpret_cast<uint8_t *>(
                mFontTableDir.Elements()));
        fprintf(stderr, "fast fetch ");
#endif
        uint32_t i;
        uint32_t *wtable = (reinterpret_cast<uint32_t *>(
                mFontTableDir.Elements()));
        for (i=3; i<(mFontTableDirSize/4); i+=4) { // Skip header
#ifdef DEBUG
                char tag[5] = { table[j], table[j+1], table[j+2], table[j+3],
                        '\0' };
                fprintf(stderr, "%s ", tag); // remember: big endian
                j += 16;
#endif
                // ASSUME THAT aTag is already big endian
                if(wtable[i] == aTag) {
#ifdef DEBUG
                        fprintf(stderr, "MATCH: length %i\n", wtable[i+3]);
#endif
                        dataLength = (ByteCount)wtable[i+3];
                        break;
                }
        }
    }
    
    if (MOZ_UNLIKELY(dataLength == 0)) {
        // Either we don't know, or something was wrong with the table.
#ifdef DEBUG
        if (mFontTableDirSize > 0) fprintf(stderr, "NO MATCH\n");
#endif
        OSStatus status = ::ATSFontGetTable(fontRef, aTag, 0, 0, 0,
                &dataLength);
        if (status != noErr) return nullptr;
    }

    // Taking advantage of bridging CFMutableDataRef to CFDataRef.
    CFMutableDataRef dataRef = ::CFDataCreateMutable(kCFAllocatorDefault,
        dataLength);
    if (!dataRef) return nullptr;

    ::CFDataIncreaseLength(dataRef, dataLength); // paranoia
    if(::ATSFontGetTable(fontRef, aTag, 0, dataLength,
                ::CFDataGetMutableBytePtr(dataRef),
                &dataLength) != noErr) {
        ::CFRelease(dataRef);
        return nullptr;
    }

    return hb_blob_create((const char*)::CFDataGetBytePtr(dataRef),
                          ::CFDataGetLength(dataRef),
                          HB_MEMORY_MODE_READONLY,
                          (void*)dataRef, DestroyBlobFunc);
#endif
}

static bool FindTagInTableDir(FallibleTArray<uint8_t>& table, 
                uint32_t aTableTag, ByteCount sizer) {
  // Parse it. In big endian format, each entry is 4 32-bit words
  // corresponding to the tag, checksum, offset and length, with a
  // 96 bit header (three 32-bit words). One day we could even write
  // an AltiVec version ...
#ifdef DEBUG
  uint32_t j = 12;
#endif
  uint32_t i;
  uint32_t *wtable = (reinterpret_cast<uint32_t *>(table.Elements()));
  for (i=3; i<(sizer/4); i+=4) { // Skip header
#ifdef DEBUG
    char tag[5] = { table[j], table[j+1], table[j+2], table[j+3], '\0' };
    fprintf(stderr, "%s ", tag); // remember: big endian
    j+=16;
#endif
    // ASSUME THAT aTableTag is already big endian
    if(wtable[i] == aTableTag) {
#ifdef DEBUG
      fprintf(stderr, "MATCH\n");
#endif
      return true;
    }
  }
  // Hmmm. Either something is wrong, or there is no table. So no table.
#ifdef DEBUG
  fprintf(stderr, "NO MATCH\n");
#endif
  return false;
}


bool
MacOSFontEntry::HasFontTable(uint32_t aTableTag)
{
    nsAutoreleasePool localPool;

#if(0)
    CGFontRef fontRef = GetFontRef();
    if (!fontRef) {
        return false;
    }

    CFDataRef tableData = ::CGFontCopyTableForTag(fontRef, aTableTag);
    if (!tableData) {
        return false;
    }

    ::CFRelease(tableData);
    return true;
#else
    // ATSFontRef version
    // This is higher performance than the previous version.

    ATSFontRef fontRef = GetATSFontRef();
    if (fontRef == kInvalidFont) return false;

    // Use cached directory to avoid repeatedly fetching the same data.
    if (MOZ_LIKELY(mFontTableDirSize > 0))
        return FindTagInTableDir(mFontTableDir, aTableTag, mFontTableDirSize);

    ByteCount sizer;

    if(::ATSFontGetTableDirectory(fontRef, 0, NULL, &sizer) == noErr) {
      // If the header is abnormal, try the old, slower way in case this
      // is a gap in our algorithm.
      if (sizer <= 12 || ((sizer-12) % 16) || sizer >= 1024) {
        fprintf(stderr, "Warning: TenFourFox found "
                "abnormal font table dir in %s (%i).\n",
                 NS_ConvertUTF16toUTF8(mName).get(), sizer);
        return
        (::ATSFontGetTable(fontRef, aTableTag, 0, 0, 0, &sizer) == noErr);
      }

      // Get and cache the font table directory.
      mFontTableDirSize = sizer;
      mFontTableDir.SetLength(mFontTableDirSize);
#ifdef DEBUG
      fprintf(stderr, "Size of %s font table directory: %i\nTables: ",
                NS_ConvertUTF16toUTF8(mName).get(), mFontTableDir.Length());
#endif
      if (::ATSFontGetTableDirectory(fontRef, mFontTableDirSize,
        reinterpret_cast<void *>(mFontTableDir.Elements()), &sizer) == noErr)
        return FindTagInTableDir(mFontTableDir, aTableTag, mFontTableDirSize);
   }
   return nullptr;
#endif
}

void
MacOSFontEntry::AddSizeOfIncludingThis(MallocSizeOf aMallocSizeOf,
                                       FontListSizes* aSizes) const
{
    aSizes->mFontListSize += aMallocSizeOf(this);
    AddSizeOfExcludingThis(aMallocSizeOf, aSizes);
}

/* gfxMacFontFamily */
#pragma mark-

class gfxMacFontFamily : public gfxFontFamily
{
public:
    gfxMacFontFamily(nsAString& aName) :
        gfxFontFamily(aName)
    {}

    virtual ~gfxMacFontFamily() {}

    virtual void LocalizedName(nsAString& aLocalizedName);

    virtual void FindStyleVariations(FontInfoData *aFontInfoData = nullptr);

    void EliminateDuplicateFaces(); // needed for 10.4
};

void
gfxMacFontFamily::LocalizedName(nsAString& aLocalizedName)
{
    nsAutoreleasePool localPool;

    if (!HasOtherFamilyNames()) {
        aLocalizedName = mName;
        return;
    }

    NSString *family = GetNSStringForString(mName);
    NSString *localized = [sFontManager
                           localizedNameForFamily:family
                                             face:nil];

    if (localized) {
        GetStringForNSString(localized, aLocalizedName);
        return;
    }

    // failed to get localized name, just use the canonical one
    aLocalizedName = mName;
}

// Return the CSS weight value to use for the given face, overriding what
// AppKit gives us (used to adjust families with bad weight values, see
// bug 931426).
// A return value of 0 indicates no override - use the existing weight.
static inline int
GetWeightOverride(const nsAString& aPSName)
{
    nsAutoCString prefName("font.weight-override.");
    // The PostScript name is required to be ASCII; if it's not, the font is
    // broken anyway, so we really don't care that this is lossy.
    LossyAppendUTF16toASCII(aPSName, prefName);
    return Preferences::GetInt(prefName.get(), 0);
}

void
gfxMacFontFamily::FindStyleVariations(FontInfoData *aFontInfoData)
{
    if (mHasStyles)
        return;

    nsAutoreleasePool localPool;

    NSString *family = GetNSStringForString(mName);

    // create a font entry for each face
    NSArray *fontfaces = [sFontManager
                          availableMembersOfFontFamily:family];  // returns an array of [psname, style name, weight, traits] elements, goofy api
    int faceCount = [fontfaces count];
    int faceIndex;

    for (faceIndex = 0; faceIndex < faceCount; faceIndex++) {
        NSArray *face = [fontfaces objectAtIndex:faceIndex];
        NSString *psname = [face objectAtIndex:INDEX_FONT_POSTSCRIPT_NAME];
        int32_t appKitWeight = [[face objectAtIndex:INDEX_FONT_WEIGHT] unsignedIntValue];
        uint32_t macTraits = [[face objectAtIndex:INDEX_FONT_TRAITS] unsignedIntValue];
        NSString *facename = [face objectAtIndex:INDEX_FONT_FACE_NAME];
        bool isStandardFace = false;

        if (appKitWeight == kAppleExtraLightWeight) {
            // if the facename contains UltraLight, set the weight to the ultralight weight value
            NSRange range = [facename rangeOfString:@"ultralight" options:NSCaseInsensitiveSearch];
            if (range.location != NSNotFound) {
                appKitWeight = kAppleUltraLightWeight;
            }
        }

        // make a nsString
        nsAutoString postscriptFontName;
        GetStringForNSString(psname, postscriptFontName);

        int32_t cssWeight = GetWeightOverride(postscriptFontName);
        if (cssWeight) {
            // scale down and clamp, to get a value from 1..9
            cssWeight = ((cssWeight + 50) / 100);
            cssWeight = std::max(1, std::min(cssWeight, 9));
        } else {
            cssWeight =
                gfxMacPlatformFontList::AppleWeightToCSSWeight(appKitWeight);
        }
        cssWeight *= 100; // scale up to CSS values

        if ([facename isEqualToString:@"Regular"] ||
            [facename isEqualToString:@"Bold"] ||
            [facename isEqualToString:@"Italic"] ||
            [facename isEqualToString:@"Oblique"] ||
            [facename isEqualToString:@"Bold Italic"] ||
            [facename isEqualToString:@"Bold Oblique"])
        {
            isStandardFace = true;
        }

        // create a font entry
        MacOSFontEntry *fontEntry =
            new MacOSFontEntry(postscriptFontName, cssWeight, isStandardFace);
        if (!fontEntry) {
            break;
        }

        // set additional properties based on the traits reported by Cocoa
        if (macTraits & (NSCondensedFontMask | NSNarrowFontMask | NSCompressedFontMask)) {
            fontEntry->mStretch = NS_FONT_STRETCH_CONDENSED;
        } else if (macTraits & NSExpandedFontMask) {
            fontEntry->mStretch = NS_FONT_STRETCH_EXPANDED;
        }
        // Cocoa fails to set the Italic traits bit for HelveticaLightItalic,
        // at least (see bug 611855), so check for style name endings as well
        if ((macTraits & NSItalicFontMask) ||
            [facename hasSuffix:@"Italic"] ||
            [facename hasSuffix:@"Oblique"])
        {
            fontEntry->mItalic = true;
        }
        if (macTraits & NSFixedPitchFontMask) {
            fontEntry->mFixedPitch = true;
        }

#ifdef PR_LOGGING
        if (LOG_FONTLIST_ENABLED()) {
            LOG_FONTLIST(("(fontlist) added (%s) to family (%s)"
                 " with style: %s weight: %d stretch: %d"
                 " (apple-weight: %d macTraits: %8.8x)",
                 NS_ConvertUTF16toUTF8(fontEntry->Name()).get(), 
                 NS_ConvertUTF16toUTF8(Name()).get(), 
                 fontEntry->IsItalic() ? "italic" : "normal",
                 cssWeight, fontEntry->Stretch(),
                 appKitWeight, macTraits));
        }
#endif

        // insert into font entry array of family
        AddFontEntry(fontEntry);
    }

    SortAvailableFonts();
    SetHasStyles(true);

    if (mIsBadUnderlineFamily) {
        SetBadUnderlineFonts();
    }
}

// restored from bug 663688 for 10.4
void
gfxMacFontFamily::EliminateDuplicateFaces()
{
    uint32_t i, bold, numFonts, italicIndex;
    MacOSFontEntry *italic, *nonitalic;

    FindStyleVariations();

    // if normal and italic have the same ATS font ref, delete italic
    // if bold and bold-italic have the same ATS font ref, delete bold-italic

    // two iterations, one for normal, one for bold
    for (bold = 0; bold < 2; bold++) {
        numFonts = mAvailableFonts.Length();

        // find the non-italic face
        nonitalic = nullptr;
        for (i = 0; i < numFonts; i++) {
            if ((mAvailableFonts[i]->IsBold() == (bold == 1)) &&
                !mAvailableFonts[i]->IsItalic()) {
                nonitalic = static_cast<MacOSFontEntry*>(mAvailableFonts[i].get());
                break;
            }
        }

        // find the italic face
        if (nonitalic) {
            italic = nullptr;
            for (i = 0; i < numFonts; i++) {
                if ((mAvailableFonts[i]->IsBold() == (bold == 1)) &&
                     mAvailableFonts[i]->IsItalic()) {
                    italic = static_cast<MacOSFontEntry*>(mAvailableFonts[i].get());
                    italicIndex = i;
                    break;
                }
            }

            // if italic face and non-italic face have matching ATS refs,
            // or if the italic returns 0 rather than an actual ATSFontRef,
            // then the italic face is bogus so remove it
            if (italic && (italic->GetATSFontRef() == 0 ||
			   italic->GetATSFontRef() == kInvalidFont ||
                           italic->GetATSFontRef() == nonitalic->GetATSFontRef())) {
                mAvailableFonts.RemoveElementAt(italicIndex);
            }
        }
    }
}



/* gfxSingleFaceMacFontFamily */
#pragma mark-

class gfxSingleFaceMacFontFamily : public gfxFontFamily
{
public:
    gfxSingleFaceMacFontFamily(nsAString& aName) :
        gfxFontFamily(aName)
    {
        mFaceNamesInitialized = true; // omit from face name lists
    }

    virtual ~gfxSingleFaceMacFontFamily() {}

    virtual void LocalizedName(nsAString& aLocalizedName);

    virtual void ReadOtherFamilyNames(gfxPlatformFontList *aPlatformFontList);
};

void
gfxSingleFaceMacFontFamily::LocalizedName(nsAString& aLocalizedName)
{
    nsAutoreleasePool localPool;

    if (!HasOtherFamilyNames()) {
        aLocalizedName = mName;
        return;
    }

    gfxFontEntry *fe = mAvailableFonts[0];
    NSFont *font = [NSFont fontWithName:GetNSStringForString(fe->Name())
                                   size:0.0];
    if (font) {
        NSString *localized = [font displayName];
        if (localized) {
            GetStringForNSString(localized, aLocalizedName);
            return;
        }
    }

    // failed to get localized name, just use the canonical one
    aLocalizedName = mName;
}

void
gfxSingleFaceMacFontFamily::ReadOtherFamilyNames(gfxPlatformFontList *aPlatformFontList)
{
    if (mOtherFamilyNamesInitialized) {
        return;
    }

    gfxFontEntry *fe = mAvailableFonts[0];
    if (!fe) {
        return;
    }

    const uint32_t kNAME = TRUETYPE_TAG('n','a','m','e');

    gfxFontEntry::AutoTable nameTable(fe, kNAME);
    if (!nameTable) {
        return;
    }

    mHasOtherFamilyNames = ReadOtherFamilyNamesForFace(aPlatformFontList,
                                                       nameTable,
                                                       true);

    mOtherFamilyNamesInitialized = true;
}


/* gfxMacPlatformFontList */
#pragma mark-

gfxMacPlatformFontList::gfxMacPlatformFontList() :
    gfxPlatformFontList(false),
    mDefaultFont(NULL) // we can't use nullptr for an ATSFontRef
    , mATSGeneration(uint32_t(kATSGenerationInitial)) // backout 869762
{
#if(0)
    ::CFNotificationCenterAddObserver(::CFNotificationCenterGetLocalCenter(),
                                      this,
                                      RegisteredFontsChangedNotificationCallback,
                                      kCTFontManagerRegisteredFontsChangedNotification,
                                      0,
                                      CFNotificationSuspensionBehaviorDeliverImmediately);
#else
	// backout bug 869762
    ::ATSFontNotificationSubscribe(ATSNotification,
                                   kATSFontNotifyOptionDefault,
                                   (void*)this, nullptr);
#endif

    // cache this in a static variable so that MacOSFontFamily objects
    // don't have to repeatedly look it up
    sFontManager = [NSFontManager sharedFontManager];
}

gfxMacPlatformFontList::~gfxMacPlatformFontList()
{
#if(0)
    if (mDefaultFont) {
        ::CFRelease(mDefaultFont);
    }
#endif
}

nsresult
gfxMacPlatformFontList::InitFontList()
{
    nsAutoreleasePool localPool;

    Telemetry::AutoTimer<Telemetry::MAC_INITFONTLIST_TOTAL> timer;

// backout bug 869762
    ATSGeneration currentGeneration = ::ATSGetGeneration();

    // need to ignore notifications after adding each font
    if (mATSGeneration == currentGeneration)
        return NS_OK;
    mATSGeneration = currentGeneration;
#ifdef PR_LOGGING
    LOG_FONTLIST(("(fontlist) updating to generation: %d", mATSGeneration));
#endif

    // reset font lists
    gfxPlatformFontList::InitFontList();
    
#if(0)
    // iterate over available families

    CFArrayRef familyNames = CTFontManagerCopyAvailableFontFamilyNames();

    // iterate over families
    uint32_t i, numFamilies;

    numFamilies = CFArrayGetCount(familyNames);
    for (i = 0; i < numFamilies; i++) {
        CFStringRef family = (CFStringRef)CFArrayGetValueAtIndex(familyNames, i);

        // CTFontManager includes weird internal family names and
        // LastResort, skip over those
        if (!family ||
            ::CFStringHasPrefix(family, CFSTR(".")) ||
            CFStringCompare(family, CFSTR("LastResort"),
                            kCFCompareCaseInsensitive) == kCFCompareEqualTo) {
            continue;
        }

        nsAutoTArray<UniChar, 1024> buffer;
        CFIndex len = ::CFStringGetLength(family);
        buffer.SetLength(len+1);
        ::CFStringGetCharacters(family, ::CFRangeMake(0, len),
                                buffer.Elements());
        buffer[len] = 0;
        nsAutoString familyName(reinterpret_cast<char16_t*>(buffer.Elements()), len);

        // create a family entry
        gfxFontFamily *familyEntry = new gfxMacFontFamily(familyName);
        if (!familyEntry) break;

        // add the family entry to the hash table
        ToLowerCase(familyName);
        mFontFamilies.Put(familyName, familyEntry);

        // check the bad underline blacklist
        if (mBadUnderlineFamilyNames.Contains(familyName))
            familyEntry->SetBadUnderlineFamily();
    }

    CFRelease(familyNames);
#else
    // Pre-Core Text version.
    NSEnumerator *families = [[sFontManager availableFontFamilies]
                              objectEnumerator];
			// returns "canonical", non-localized family name
 
    nsAutoString availableFamilyName;

    NSString *availableFamily = nil;
    while ((availableFamily = [families nextObject])) {
        // make a nsString
        GetStringForNSString(availableFamily, availableFamilyName);
        // create a family entry
        gfxFontFamily *familyEntry = new gfxMacFontFamily(availableFamilyName);
        if (!familyEntry) break;
 
        // add the family entry to the hash table
        ToLowerCase(availableFamilyName);
        mFontFamilies.Put(availableFamilyName, familyEntry);

        // check the bad underline blacklist
        if (mBadUnderlineFamilyNames.Contains(availableFamilyName))
            familyEntry->SetBadUnderlineFamily();
   }
#endif

    InitSingleFaceList();

    // to avoid full search of font name tables, seed the other names table with localized names from
    // some of the prefs fonts which are accessed via their localized names.  changes in the pref fonts will only cause
    // a font lookup miss earlier. this is a simple optimization, it's not required for correctness
    PreloadNamesList();

    // clean up various minor 10.4 font problems for specific fonts
    if (!nsCocoaFeatures::OnLeopardOrLater()) {
        // Cocoa calls report that italic faces exist for Courier and Helvetica
        // even though only bold faces exist so test for this using ATS font
	// refs (10.5 has proper faces)
        EliminateDuplicateFaces(NS_LITERAL_STRING("Courier"));
        EliminateDuplicateFaces(NS_LITERAL_STRING("Helvetica"));

        // Cocoa reports that Courier and Monaco are not fixed-pitch fonts
        // so explicitly tweak these settings
        SetFixedPitch(NS_LITERAL_STRING("Courier"));
        SetFixedPitch(NS_LITERAL_STRING("Monaco"));
    }

    // start the delayed cmap loader
    GetPrefsAndStartLoader();

    return NS_OK;
}

void
gfxMacPlatformFontList::InitSingleFaceList()
{
    nsAutoTArray<nsString, 10> singleFaceFonts;
    gfxFontUtils::GetPrefsFontList("font.single-face-list", singleFaceFonts);

    uint32_t numFonts = singleFaceFonts.Length();
    for (uint32_t i = 0; i < numFonts; i++) {
#ifdef PR_LOGGING
        LOG_FONTLIST(("(fontlist-singleface) face name: %s\n",
                      NS_ConvertUTF16toUTF8(singleFaceFonts[i]).get()));
#endif
        gfxFontEntry *fontEntry = LookupLocalFont(nullptr, singleFaceFonts[i]);
        if (fontEntry) {
            nsAutoString familyName, key;
            familyName = singleFaceFonts[i];
            GenerateFontListKey(familyName, key);
#ifdef PR_LOGGING
            LOG_FONTLIST(("(fontlist-singleface) family name: %s, key: %s\n",
                          NS_ConvertUTF16toUTF8(familyName).get(),
                          NS_ConvertUTF16toUTF8(key).get()));
#endif

            // add only if doesn't exist already
            if (!mFontFamilies.GetWeak(key)) {
                gfxFontFamily *familyEntry =
                    new gfxSingleFaceMacFontFamily(familyName);
                familyEntry->AddFontEntry(fontEntry);
                familyEntry->SetHasStyles(true);
                mFontFamilies.Put(key, familyEntry);
#ifdef PR_LOGGING
                LOG_FONTLIST(("(fontlist-singleface) added new family\n",
                              NS_ConvertUTF16toUTF8(familyName).get(),
                              NS_ConvertUTF16toUTF8(key).get()));
#endif
            }
        }
    }
}

void
gfxMacPlatformFontList::EliminateDuplicateFaces(const nsAString& aFamilyName)
{
    gfxMacFontFamily *family =
        static_cast<gfxMacFontFamily*>(FindFamily(aFamilyName));

    if (family)
        family->EliminateDuplicateFaces();
}

bool
gfxMacPlatformFontList::GetStandardFamilyName(const nsAString& aFontName, nsAString& aFamilyName)
{
    gfxFontFamily *family = FindFamily(aFontName);
    if (family) {
        family->LocalizedName(aFamilyName);
        return true;
    }

    // backout bug 925241
    // Gecko 1.8 used Quickdraw font api's which produce a slightly different set of "family"
    // names.  Try to resolve based on these names, in case this is stored in an old profile
    // 1.8: "Futura", "Futura Condensed" ==> 1.9: "Futura"

    // convert the name to a Pascal-style Str255 to try as Quickdraw name
    Str255 qdname;
    NS_ConvertUTF16toUTF8 utf8name(aFontName);
    qdname[0] = std::max<size_t>(255, strlen(utf8name.get()));
    memcpy(&qdname[1], utf8name.get(), qdname[0]);

    // look up the Quickdraw name
    ATSFontFamilyRef atsFamily = ::ATSFontFamilyFindFromQuickDrawName(qdname);
    if (atsFamily == (ATSFontFamilyRef)kInvalidFontFamily) {
        return false;
    }

    // if we found a family, get its ATS name
    CFStringRef cfName;
    OSStatus status = ::ATSFontFamilyGetName(atsFamily, kATSOptionFlagsDefault, &cfName);
    if (status != noErr) {
        return false;
    }

    // then use this to locate the family entry and retrieve its localized name
    nsAutoString familyName;
    GetStringForNSString((const NSString*)cfName, familyName);
    ::CFRelease(cfName);

    family = FindFamily(familyName);
    if (family) {
        family->LocalizedName(aFamilyName);
        return true;
    }

    return false;
}

#if(0)
void
gfxMacPlatformFontList::RegisteredFontsChangedNotificationCallback(CFNotificationCenterRef center,
                                                                   void *observer,
                                                                   CFStringRef name,
                                                                   const void *object,
                                                                   CFDictionaryRef userInfo)
{
    if (!::CFEqual(name, kCTFontManagerRegisteredFontsChangedNotification)) {
        return;
    }

    gfxMacPlatformFontList* fl = static_cast<gfxMacPlatformFontList*>(observer);

    // xxx - should be carefully pruning the list of fonts, not rebuilding it from scratch
    fl->UpdateFontList();

    // modify a preference that will trigger reflow everywhere
    fl->ForceGlobalReflow();
}
#else
// backout bug 869762
void
gfxMacPlatformFontList::ATSNotification(ATSFontNotificationInfoRef aInfo,
                                        void* aUserArg)
{
    gfxMacPlatformFontList* fl =  static_cast<gfxMacPlatformFontList*>(aUserArg);

    // xxx - should be carefully pruning the list of fonts, not rebuilding it from scratch
    fl->UpdateFontList();

    // modify a preference that will trigger reflow everywhere
    fl->ForceGlobalReflow();
}
#endif

gfxFontEntry*
gfxMacPlatformFontList::GlobalFontFallback(const uint32_t aCh,
                                           int32_t aRunScript,
                                           const gfxFontStyle* aMatchStyle,
                                           uint32_t& aCmapCount,
                                           gfxFontFamily** aMatchedFamily)
{
#if(0)
    bool useCmaps = gfxPlatform::GetPlatform()->UseCmapsDuringSystemFallback();

    if (useCmaps) {
#endif
        return gfxPlatformFontList::GlobalFontFallback(aCh,
                                                       aRunScript,
                                                       aMatchStyle,
                                                       aCmapCount,
                                                       aMatchedFamily);
// 10.4 CoreText does not support the features needed, but more to the point,
// this gives us a CTFontRef and we can't work with that at all.
#if(0)
    }

    CFStringRef str;
    UniChar ch[2];
    CFIndex len = 1;

    if (IS_IN_BMP(aCh)) {
        ch[0] = aCh;
        str = ::CFStringCreateWithCharactersNoCopy(kCFAllocatorDefault, ch, 1,
                                                   kCFAllocatorNull);
    } else {
        ch[0] = H_SURROGATE(aCh);
        ch[1] = L_SURROGATE(aCh);
        str = ::CFStringCreateWithCharactersNoCopy(kCFAllocatorDefault, ch, 2,
                                                   kCFAllocatorNull);
        if (!str) {
            return nullptr;
        }
        len = 2;
    }

    // use CoreText to find the fallback family

    gfxFontEntry *fontEntry = nullptr;
    CTFontRef fallback;
    bool cantUseFallbackFont = false;

    if (!mDefaultFont) {
        mDefaultFont = ::CTFontCreateWithName(CFSTR("LucidaGrande"), 12.f,
                                              NULL);
    }

    fallback = ::CTFontCreateForString(mDefaultFont, str,
                                       ::CFRangeMake(0, len));

    if (fallback) {
        CFStringRef familyName = ::CTFontCopyFamilyName(fallback);
        ::CFRelease(fallback);

        if (familyName &&
            ::CFStringCompare(familyName, CFSTR("LastResort"),
                              kCFCompareCaseInsensitive) != kCFCompareEqualTo)
        {
            nsAutoTArray<UniChar, 1024> buffer;
            CFIndex len = ::CFStringGetLength(familyName);
            buffer.SetLength(len+1);
            ::CFStringGetCharacters(familyName, ::CFRangeMake(0, len),
                                    buffer.Elements());
            buffer[len] = 0;
            nsDependentString familyName(reinterpret_cast<char16_t*>(buffer.Elements()), len);

            bool needsBold;  // ignored in the system fallback case

            gfxFontFamily *family = FindFamily(familyName);
            if (family) {
                fontEntry = family->FindFontForStyle(*aMatchStyle, needsBold);
                if (fontEntry) {
                    if (fontEntry->TestCharacterMap(aCh)) {
                        *aMatchedFamily = family;
                    } else {
                        fontEntry = nullptr;
                        cantUseFallbackFont = true;
                    }
                }
            }
        }

        if (familyName) {
            ::CFRelease(familyName);
        }
    }

    if (cantUseFallbackFont) {
        Telemetry::Accumulate(Telemetry::BAD_FALLBACK_FONT, cantUseFallbackFont);
    }

    ::CFRelease(str);

    return fontEntry;
#else
    NS_NOTREACHED("GlobalFontFallback fell back");
    return nullptr;
#endif
}

gfxFontFamily*
gfxMacPlatformFontList::GetDefaultFont(const gfxFontStyle* aStyle)
{
    nsAutoreleasePool localPool;

    NSString *defaultFamily = [[NSFont userFontOfSize:aStyle->size] familyName];
    nsAutoString familyName;

    GetStringForNSString(defaultFamily, familyName);
    return FindFamily(familyName);
}

int32_t
gfxMacPlatformFontList::AppleWeightToCSSWeight(int32_t aAppleWeight)
{
    if (aAppleWeight < 1)
        aAppleWeight = 1;
    else if (aAppleWeight > kAppleMaxWeight)
        aAppleWeight = kAppleMaxWeight;
    return gAppleWeightToCSSWeight[aAppleWeight];
}

gfxFontEntry*
gfxMacPlatformFontList::LookupLocalFont(const gfxProxyFontEntry *aProxyEntry,
                                        const nsAString& aFontName)
{
    nsAutoreleasePool localPool;

    NSString *faceName = GetNSStringForString(aFontName);
    MacOSFontEntry *newFontEntry;

#if(1)
    // ATSFontRef version

    // first lookup a single face based on postscript name
    ATSFontRef fontRef = ::ATSFontFindFromPostScriptName(CFStringRef(faceName),
        kATSOptionFlagsDefault);
    // if not found, lookup using full font name
    if (fontRef == kInvalidFont) {
        fontRef = ::ATSFontFindFromName(CFStringRef(faceName),
                                        kATSOptionFlagsDefault);
        if (fontRef == kInvalidFont) {
            return nullptr;
        }
    }

    if (aProxyEntry) {
        uint16_t w = aProxyEntry->mWeight;
        NS_ASSERTION(w >= 100 && w <= 900, "bogus font weight value!");

        newFontEntry =
            new MacOSFontEntry(aFontName, fontRef,
                               w, aProxyEntry->mStretch,
                               aProxyEntry->mItalic ?
                                   NS_FONT_STYLE_ITALIC : NS_FONT_STYLE_NORMAL,
// we use NULL on purpose! or, I guess we could cast to ATSFontContainerRef
                               NULL, true, true);
    } else {
        newFontEntry =
            new MacOSFontEntry(aFontName, fontRef,
                               400, 0, NS_FONT_STYLE_NORMAL,
                               NULL, false, false);
    }
#else
    // lookup face based on postscript or full name
    CGFontRef fontRef = ::CGFontCreateWithFontName(CFStringRef(faceName));
    if (!fontRef) {
        return nullptr;
    }

    if (aProxyEntry) {
        uint16_t w = aProxyEntry->mWeight;
        NS_ASSERTION(w >= 100 && w <= 900, "bogus font weight value!");

        newFontEntry =
            new MacOSFontEntry(aFontName, fontRef,
                               w, aProxyEntry->mStretch,
                               aProxyEntry->mItalic ?
                                   NS_FONT_STYLE_ITALIC : NS_FONT_STYLE_NORMAL,
                               true, true);
    } else {
        newFontEntry =
            new MacOSFontEntry(aFontName, fontRef,
                               400, 0, NS_FONT_STYLE_NORMAL,
                               false, false);
    }
    ::CFRelease(fontRef);
#endif

    return newFontEntry;
}

static void ReleaseData(void *info, const void *data, size_t size)
{
    NS_Free((void*)data);
}

// Backout from bug 811312
// grumble, another non-publised Apple API dependency (found in Webkit code)
// activated with this value, font will not be found via system lookup routines
// it can only be used via the created ATSFontRef
// needed to prevent one doc from finding a font used in a separate doc

enum {
    kPrivateATSFontContextPrivate = 3
};

gfxFontEntry*
gfxMacPlatformFontList::MakePlatformFont(const gfxProxyFontEntry *aProxyEntry,
                                         const uint8_t *aFontData,
                                         uint32_t aLength)
{
#if(1)
    // ATSFontRef version
    OSStatus err;

    NS_ASSERTION(aFontData, "MakePlatformFont called with null data");

    // MakePlatformFont is responsible for deleting the font data with NS_Free
    // so we set up a stack object to ensure it is freed even if we take an
    // early exit
    // XXX Is this still needed? If we exit early, we die anyway.
    struct FontDataDeleter {
        FontDataDeleter(const uint8_t *aFontData)
            : mFontData(aFontData) { }
        ~FontDataDeleter() { NS_Free((void*)mFontData); }
        const uint8_t *mFontData;
    };
    FontDataDeleter autoDelete(aFontData);

    ATSFontRef fontRef;
    ATSFontContainerRef containerRef;

    // we get occasional failures when multiple fonts are activated in quick succession
    // if the ATS font cache is damaged; to work around this, we can retry the activation
    const uint32_t kMaxRetries = 3;
    uint32_t retryCount = 0;
    while (retryCount++ < kMaxRetries) {
        err = ::ATSFontActivateFromMemory(const_cast<uint8_t*>(aFontData), aLength,
                                          kPrivateATSFontContextPrivate,
                                          kATSFontFormatUnspecified,
                                          NULL,
                                          kATSOptionFlagsDoNotNotify,
                                          &containerRef);
        mATSGeneration = ::ATSGetGeneration();

        if (err != noErr) {
#if DEBUG
            char warnBuf[1024];
            sprintf(warnBuf, "downloaded font error, ATSFontActivateFromMemory err: %d",
                    int32_t(err));
            NS_WARNING(warnBuf);
#endif
            return nullptr;
        }

        // ignoring containers with multiple fonts, use the first face only for now
        err = ::ATSFontFindFromContainer(containerRef, kATSOptionFlagsDefault, 1,
                                         &fontRef, NULL);
        if (err != noErr) {
#if DEBUG
            char warnBuf[1024];
            sprintf(warnBuf, "downloaded font error, ATSFontFindFromContainer err: %d",
                    int32_t(err));
            NS_WARNING(warnBuf);
#endif
            ::ATSFontDeactivate(containerRef, NULL, kATSOptionFlagsDefault);
            return nullptr;
        }

        // now lookup the Postscript name; this may fail if the font cache is bad
        OSStatus err;
        NSString *psname = NULL;
        err = ::ATSFontGetPostScriptName(fontRef, kATSOptionFlagsDefault, (CFStringRef*) (&psname));
        if (err == noErr) {
		// Check the font blacklist (TenFourFox issue 261). 
		if ([psname isEqualToString:@"prisjakticons"]) { 
			fprintf(stderr,
"Warning: TenFourFox rejected ATSUI-incompatible web font %s.\n",
				[psname UTF8String]);
			[psname release];
			::ATSFontDeactivate(containerRef, NULL,
				kATSOptionFlagsDefault);
			return nullptr;
		}
            [psname release];
        } else {
#ifdef DEBUG
            char warnBuf[1024];
            sprintf(warnBuf, "ATSFontGetPostScriptName err = %d, retries = %d", (int32_t)err,
                    retryCount);
            NS_WARNING(warnBuf);
#endif
            ::ATSFontDeactivate(containerRef, NULL, kATSOptionFlagsDefault);
            // retry the activation a couple of times if this fails
            // (may be a transient failure due to ATS font cache issues)
            continue;
        }

        uint16_t w = aProxyEntry->mWeight;
        NS_ASSERTION(w >= 100 && w <= 900, "bogus font weight value!");

        nsAutoString uniqueName;
        nsresult rv = gfxFontUtils::MakeUniqueUserFontName(uniqueName);
        if (NS_FAILED(rv)) {
            return nullptr;
        }

        // font entry will own the container ref now
	// THIS MUST BE A C++ OBJECT, not an nsAutoPtr, or it will not
	// live long enough to be instantiated!
	MacOSFontEntry *newFontEntry =
            new MacOSFontEntry(uniqueName,
                             fontRef,
                             w, aProxyEntry->mStretch,
                             aProxyEntry->mItalic ?
                                 NS_FONT_STYLE_ITALIC : NS_FONT_STYLE_NORMAL,
                             containerRef, true, false);

        // if succeeded and font cmap is good, return the new font
        if (newFontEntry->mIsValid && NS_SUCCEEDED(newFontEntry->ReadCMAP())) {
            return newFontEntry;
        }

        // if something is funky about this font, delete immediately
#if DEBUG
        char warnBuf[1024];
        sprintf(warnBuf, "downloaded font not loaded properly");
        NS_WARNING(warnBuf);
#endif
        delete newFontEntry;

        // We don't retry from here; the ATS font cache issue would have caused failure earlier
        // so if we get here, there's something else bad going on within our font data structures.
        // Currently, there should be no way to reach here, as fontentry creation cannot fail
        // except by memory allocation failure.
        NS_WARNING("invalid font entry for a newly activated font");
        break;
    }

    // if we get here, the activation failed (even with possible retries); can't use this font
    return nullptr;
#else
    NS_ASSERTION(aFontData, "MakePlatformFont called with null data");

    uint16_t w = aProxyEntry->mWeight;
    NS_ASSERTION(w >= 100 && w <= 900, "bogus font weight value!");

    // create the font entry
    nsAutoString uniqueName;

    nsresult rv = gfxFontUtils::MakeUniqueUserFontName(uniqueName);
    if (NS_FAILED(rv)) {
        return nullptr;
    }

    CGDataProviderRef provider =
        ::CGDataProviderCreateWithData(nullptr, aFontData, aLength,
                                       &ReleaseData);
    CGFontRef fontRef = ::CGFontCreateWithDataProvider(provider);
    ::CGDataProviderRelease(provider);

    if (!fontRef) {
        return nullptr;
    }

    nsAutoPtr<MacOSFontEntry>
        newFontEntry(new MacOSFontEntry(uniqueName, fontRef, w,
                                        aProxyEntry->mStretch,
                                        aProxyEntry->mItalic ?
                                            NS_FONT_STYLE_ITALIC :
                                            NS_FONT_STYLE_NORMAL,
                                        true, false));
    ::CFRelease(fontRef);

    // if succeeded and font cmap is good, return the new font
    if (newFontEntry->mIsValid && NS_SUCCEEDED(newFontEntry->ReadCMAP())) {
        return newFontEntry.forget();
    }

    // if something is funky about this font, delete immediately

#if DEBUG
    NS_WARNING("downloaded font not loaded properly");
#endif

    return nullptr;
#endif
}

// used to load system-wide font info on off-main thread
class MacFontInfo : public FontInfoData {
public:
    MacFontInfo(bool aLoadOtherNames,
                bool aLoadFaceNames,
                bool aLoadCmaps) :
        FontInfoData(aLoadOtherNames, aLoadFaceNames, aLoadCmaps)
    {}

    virtual ~MacFontInfo() {}

    virtual void Load() {
        nsAutoreleasePool localPool;
        // bug 975460 - async font loader crashes sometimes under 10.6, disable
        if (nsCocoaFeatures::OnLionOrLater()) {
            FontInfoData::Load();
        }
    }

    // loads font data for all members of a given family
    virtual void LoadFontFamilyData(const nsAString& aFamilyName);
};

#if(0)
void
MacFontInfo::LoadFontFamilyData(const nsAString& aFamilyName)
{
    // family name ==> CTFontDescriptor
    NSString *famName = GetNSStringForString(aFamilyName);
    CFStringRef family = CFStringRef(famName);

    CFMutableDictionaryRef attr =
        CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks,
                                  &kCFTypeDictionaryValueCallBacks);
    CFDictionaryAddValue(attr, kCTFontFamilyNameAttribute, family);
    CTFontDescriptorRef fd = CTFontDescriptorCreateWithAttributes(attr);
    CFRelease(attr);
    CFArrayRef matchingFonts =
        CTFontDescriptorCreateMatchingFontDescriptors(fd, NULL);
    CFRelease(fd);
    if (!matchingFonts) {
        return;
    }

    nsTArray<nsString> otherFamilyNames;
    bool hasOtherFamilyNames = true;

    // iterate over faces in the family
    int f, numFaces = (int) CFArrayGetCount(matchingFonts);
    for (f = 0; f < numFaces; f++) {
        mLoadStats.fonts++;

        CTFontDescriptorRef faceDesc =
            (CTFontDescriptorRef)CFArrayGetValueAtIndex(matchingFonts, f);
        if (!faceDesc) {
            continue;
        }
        CTFontRef fontRef = CTFontCreateWithFontDescriptor(faceDesc,
                                                           0.0, nullptr);
        if (!fontRef) {
            NS_WARNING("failed to create a CTFontRef");
            continue;
        }

        if (mLoadCmaps) {
            // face name
            CFStringRef faceName = (CFStringRef)
                CTFontDescriptorCopyAttribute(faceDesc, kCTFontNameAttribute);

            nsAutoTArray<UniChar, 1024> buffer;
            CFIndex len = CFStringGetLength(faceName);
            buffer.SetLength(len+1);
            CFStringGetCharacters(faceName, ::CFRangeMake(0, len),
                                    buffer.Elements());
            buffer[len] = 0;
            nsAutoString fontName(reinterpret_cast<char16_t*>(buffer.Elements()),
                                  len);

            // load the cmap data
            FontFaceData fontData;
            CFDataRef cmapTable = CTFontCopyTable(fontRef, kCTFontTableCmap,
                                                  kCTFontTableOptionNoOptions);

            if (cmapTable) {
                bool unicodeFont = false, symbolFont = false; // ignored
                const uint8_t *cmapData =
                    (const uint8_t*)CFDataGetBytePtr(cmapTable);
                uint32_t cmapLen = CFDataGetLength(cmapTable);
                nsRefPtr<gfxCharacterMap> charmap = new gfxCharacterMap();
                uint32_t offset;
                nsresult rv;

                rv = gfxFontUtils::ReadCMAP(cmapData, cmapLen, *charmap, offset,
                                            unicodeFont, symbolFont);
                if (NS_SUCCEEDED(rv)) {
                    fontData.mCharacterMap = charmap;
                    fontData.mUVSOffset = offset;
                    fontData.mSymbolFont = symbolFont;
                    mLoadStats.cmaps++;
                }
                CFRelease(cmapTable);
            }

            mFontFaceData.Put(fontName, fontData);
            CFRelease(faceName);
        }

        if (mLoadOtherNames && hasOtherFamilyNames) {
            CFDataRef nameTable = CTFontCopyTable(fontRef, kCTFontTableName,
                                                  kCTFontTableOptionNoOptions);

            if (nameTable) {
                const char *nameData = (const char*)CFDataGetBytePtr(nameTable);
                uint32_t nameLen = CFDataGetLength(nameTable);
                gfxFontFamily::ReadOtherFamilyNamesForFace(aFamilyName,
                                                           nameData, nameLen,
                                                           otherFamilyNames,
                                                           false);
                hasOtherFamilyNames = otherFamilyNames.Length() != 0;
                CFRelease(nameTable);
            }
        }

        CFRelease(fontRef);
    }
    CFRelease(matchingFonts);

    // if found other names, insert them in the hash table
    if (otherFamilyNames.Length() != 0) {
        mOtherFamilyNames.Put(aFamilyName, otherFamilyNames);
        mLoadStats.othernames += otherFamilyNames.Length();
    }
}
#else
// ATS-based version.
void
MacFontInfo::LoadFontFamilyData(const nsAString& aFamilyName)
{
    // ATS does not have the concept of fonts belonging to a family like
    // CoreText does, but the list of names we got from [NSFontManager
    // availableFontFamilies] may not exactly correspond to an ATS name.
    // However, [NSFontManager availableMembersOfFontFamily] will tell us.

NS_WARNING("LoadFontFamilyData not yet supported, do not call (bug 962440)");
MOZ_ASSERT(0);

#if(0)
    nsAutoreleasePool bubble_bubble_toil_and_trouble;
    NSString *famName = GetNSStringForString(aFamilyName);
//    CFStringRef family = CFStringRef(famName);
    NSArray *matchingFonts =
	[sFontManager availableMembersOfFontFamily:famName];
    int f, numFaces = (int) CFArrayGetCount(matchingFonts);
    if(!numFaces) {
#ifdef DEBUG
	fprintf(stderr, "no available fonts for family %s\n",
		aFamilyName.get());
#endif
	return;
    }

    nsTArray<nsString> otherFamilyNames;
    bool hasOtherFamilyNames = true;

    for (f = 0; f < numFaces; f++) {
        mLoadStats.fonts++;

	// Each element is an array of arrays:
	// (("Times-Roman", "Roman", 5, 4), ...
	// corresponding to the full PSName, the weight name, the weight
	// and its traits. We only care about the PSName, because now we
	// can get an ATSFontRef from that.
	NSArray *k = (NSArray *)CFArrayGetValueAtIndex(matchingFonts, f);
	if((int)CFArrayGetCount(k) < 2) { // wtf
		continue;
	}
	NSString *psname = (NSString *)CFArrayGetValueAtIndex(k, 0);
#ifdef DEBUG
	fprintf(stderr, "Deferred loading: %s", [psname UTF8String]);
#endif
	ATSFontRef fontRef = ::ATSFontFindFromPostScriptName(
		(CFStringRef)psname,
		kATSOptionFlagsDefault);
	if (!fontRef) { // wtff
#ifdef DEBUG
		fprintf(stderr, "Failed loading: %s", [psname UTF8String]);
#endif
		continue;
	}

        if (mLoadCmaps) {
            // face name (fudge it into char16_t)
            CFStringRef faceName = (CFStringRef)psname;
            nsAutoTArray<UniChar, 1024> buffer;
            CFIndex len = CFStringGetLength(faceName);
            buffer.SetLength(len+1);
            CFStringGetCharacters(faceName, ::CFRangeMake(0, len),
                                    buffer.Elements());
            buffer[len] = 0;
            nsAutoString
		fontName(reinterpret_cast<char16_t*>(buffer.Elements()),
                         len);

            // load the cmap data
            FontFaceData fontData;

// Replace with one of the ATSFontTable loaders, but this sucks.
            //CFDataRef cmapTable = CTFontCopyTable(fontRef, kCTFontTableCmap,
            //                                     kCTFontTableOptionNoOptions);
            if (cmapTable) {
                bool unicodeFont = false, symbolFont = false; // ignored
                const uint8_t *cmapData =
                    (const uint8_t*)CFDataGetBytePtr(cmapTable);
                uint32_t cmapLen = CFDataGetLength(cmapTable);
                nsRefPtr<gfxCharacterMap> charmap = new gfxCharacterMap();
                uint32_t offset;
                nsresult rv;

                rv = gfxFontUtils::ReadCMAP(cmapData, cmapLen, *charmap, offset,
                                            unicodeFont, symbolFont);
                if (NS_SUCCEEDED(rv)) {
                    fontData.mCharacterMap = charmap;
                    fontData.mUVSOffset = offset;
                    fontData.mSymbolFont = symbolFont;
                    mLoadStats.cmaps++;
                }
                CFRelease(cmapTable);
            }

            mFontFaceData.Put(fontName, fontData);
            CFRelease(faceName);
        }

#if (0)
// I don't think this is true for ATS fonts.
        if (mLoadOtherNames && hasOtherFamilyNames) {
            CFDataRef nameTable = CTFontCopyTable(fontRef, kCTFontTableName,
                                                  kCTFontTableOptionNoOptions);
            if (nameTable) {
                const char *nameData = (const char*)CFDataGetBytePtr(nameTable);
                uint32_t nameLen = CFDataGetLength(nameTable);
                gfxFontFamily::ReadOtherFamilyNamesForFace(aFamilyName,
                                                           nameData, nameLen,
                                                           otherFamilyNames,
                                                           false);
                hasOtherFamilyNames = otherFamilyNames.Length() != 0;
                CFRelease(nameTable);
            }
        }

        CFRelease(fontRef);
    }
#endif
    CFRelease(matchingFonts);

    // if found other names, insert them in the hash table
    if (otherFamilyNames.Length() != 0) {
        mOtherFamilyNames.Put(aFamilyName, otherFamilyNames);
        mLoadStats.othernames += otherFamilyNames.Length();
    }
#endif
}
#endif


already_AddRefed<FontInfoData>
gfxMacPlatformFontList::CreateFontInfoData()
{
    bool loadCmaps = !UsesSystemFallback() ||
        gfxPlatform::GetPlatform()->UseCmapsDuringSystemFallback();

    nsRefPtr<MacFontInfo> fi =
        new MacFontInfo(true, NeedFullnamePostscriptNames(), loadCmaps);
    return fi.forget();
}

