// Copyright (c) 2006-2008 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/file_version_info.h"

#import <Cocoa/Cocoa.h>

#include "base/logging.h"
#include "base/string_util.h"

FileVersionInfo::FileVersionInfo(NSBundle *bundle) : bundle_(bundle) {
  [bundle_ retain];
}

FileVersionInfo::~FileVersionInfo() {
  [bundle_ release];
}

// static
FileVersionInfo* FileVersionInfo::CreateFileVersionInfoForCurrentModule() {
  // TODO(erikkay): this should really use bundleForClass, but we don't have
  // a class to hang onto yet.
  NSBundle* bundle = [NSBundle mainBundle];
  return new FileVersionInfo(bundle);
}

// static
FileVersionInfo* FileVersionInfo::CreateFileVersionInfo(
    const std::wstring& file_path) {
#if(0)
  NSString* path = [NSString stringWithCString:
      reinterpret_cast<const char*>(file_path.c_str())
        encoding:NSUTF32StringEncoding];
#else
  // There is no NSUTF32StringEncoding for 10.4, so we use CFStrings
  // and toll-free bridge the CFStringRef to an NSString.
  NSString *path = (NSString *)CFStringCreateWithBytes(kCFAllocatorDefault,
	reinterpret_cast<const UInt8 *>(file_path.c_str()),
	(file_path.length() * sizeof(wchar_t)),
	kCFStringEncodingUTF32,
	false);
#endif
  return new FileVersionInfo([NSBundle bundleWithPath:path]);
}

// static
FileVersionInfo* FileVersionInfo::CreateFileVersionInfo(
    const FilePath& file_path) {
  NSString* path = [NSString stringWithUTF8String:file_path.value().c_str()];
  return new FileVersionInfo([NSBundle bundleWithPath:path]);
}

std::wstring FileVersionInfo::company_name() {
  return L"";
}

std::wstring FileVersionInfo::company_short_name() {
  return L"";
}

std::wstring FileVersionInfo::internal_name() {
  return L"";
}

std::wstring FileVersionInfo::product_name() {
  return GetStringValue(L"CFBundleName");
}

std::wstring FileVersionInfo::product_short_name() {
  return GetStringValue(L"CFBundleName");
}

std::wstring FileVersionInfo::comments() {
  return L"";
}

std::wstring FileVersionInfo::legal_copyright() {
  return GetStringValue(L"CFBundleGetInfoString");
}

std::wstring FileVersionInfo::product_version() {
  return GetStringValue(L"CFBundleShortVersionString");
}

std::wstring FileVersionInfo::file_description() {
  return L"";
}

std::wstring FileVersionInfo::legal_trademarks() {
  return L"";
}

std::wstring FileVersionInfo::private_build() {
  return L"";
}

std::wstring FileVersionInfo::file_version() {
  // CFBundleVersion has limitations that may not be honored by a
  // proper Chromium version number, so try KSVersion first.
  std::wstring version = GetStringValue(L"KSVersion");
  if (version == L"")
    version = GetStringValue(L"CFBundleVersion");
  return version;
}

std::wstring FileVersionInfo::original_filename() {
  return GetStringValue(L"CFBundleName");
}

std::wstring FileVersionInfo::special_build() {
  return L"";
}

std::wstring FileVersionInfo::last_change() {
  return L"";
}

bool FileVersionInfo::is_official_build() {
  return false;
}

bool FileVersionInfo::GetValue(const wchar_t* name, std::wstring* value_str) {
  if (bundle_) {
    NSString* value = [bundle_ objectForInfoDictionaryKey:
        [NSString stringWithUTF8String:WideToUTF8(name).c_str()]];
    if (value) {
#if(0)
      *value_str = reinterpret_cast<const wchar_t*>(
          [value cStringUsingEncoding:NSUTF32StringEncoding]);
#else
      // There is no NSUTF32StringEncoding in 10.4, so we use CFStrings to
      // get at the NSString via toll-free bridging, convert it, and return
      // that. Broadly adapted from sys_string_conversions_mac.mm.
	CFStringRef cfstring = (CFStringRef)value;
	CFIndex length = CFStringGetLength(cfstring);
	CFIndex out_size;

	if (!length) return false;
	CFRange whole_string = CFRangeMake(0, length);
	// Figure out how much space we will need for the converted string.
	CFIndex converted = CFStringGetBytes(cfstring,
		whole_string,
		kCFStringEncodingUTF32,
		0, false, NULL, 0, &out_size);
	if (!converted || !out_size) return false;
	// Create a vector large enough to hold it, with space for NULL.
	int elements = out_size * sizeof(UInt8) * sizeof(wchar_t) + 1;
	std::vector<std::wstring::value_type> out_buffer(elements);
	converted = CFStringGetBytes(cfstring,
		whole_string,
		kCFStringEncodingUTF32,
		0, false, reinterpret_cast<UInt8 *>(&out_buffer[0]),
		out_size, NULL);
	if (!converted) return false;
	out_buffer[elements - 1] = '\0';
// This needs work.
//	value_str = reinterpret_cast<std::wstring *>(&out_buffer[0]);
	perror("GetValue needs work");
	return false;
#endif
      return true;
    }
  }
  return false;
}

std::wstring FileVersionInfo::GetStringValue(const wchar_t* name) {
  std::wstring str;
  if (GetValue(name, &str))
    return str;
  return L"";
}
