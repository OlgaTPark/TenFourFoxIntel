/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#import <Cocoa/Cocoa.h>
#include <stdio.h>
#include <unistd.h>
#include "progressui.h"
#include "readstrings.h"
#include "errors.h"

#define TIMER_INTERVAL 0.2

static float sProgressVal;  // between 0 and 100
static BOOL sQuit = FALSE;
static StringTable sLabels;
static const char *sUpdatePath;

@interface UpdaterUI : NSObject
{
  IBOutlet NSProgressIndicator *progressBar;
  IBOutlet NSTextField *progressTextField;
}
@end

@implementation UpdaterUI

-(void)awakeFromNib
{
  NSWindow *w = [progressBar window];
#ifndef NS_LEOPARD_AND_LATER
  [w center];
#endif

  [w setTitle:[NSString stringWithUTF8String:sLabels.title]];
  [progressTextField setStringValue:[NSString stringWithUTF8String:sLabels.info]];

#ifdef NS_LEOPARD_AND_LATER
  NSRect origTextFrame = [progressTextField frame];
  [progressTextField sizeToFit];

  int widthAdjust = progressTextField.frame.size.width - origTextFrame.size.width;

  if (widthAdjust > 0) {
    NSRect f;
    f.size.width  = w.frame.size.width + widthAdjust;
    f.size.height = w.frame.size.height;
    [w setFrame:f display:YES];
  }

  [w center];
#endif

  [progressBar setIndeterminate:NO];
  [progressBar setDoubleValue:0.0];

  [[NSTimer scheduledTimerWithTimeInterval:TIMER_INTERVAL target:self
                                  selector:@selector(updateProgressUI:)
                                  userInfo:nil repeats:YES] retain];

  // Make sure we are on top initially
  [NSApp activateIgnoringOtherApps:YES];
}

// called when the timer goes off
-(void)updateProgressUI:(NSTimer *)aTimer
{
  if (sQuit) {
    [aTimer invalidate];
    [aTimer release];

    // It seems to be necessary to activate and hide ourselves before we stop,
    // otherwise the "run" method will not return until the user focuses some
    // other app.  The activate step is necessary if we are not the active app.
    // This is a big hack, but it seems to do the trick.
    [NSApp activateIgnoringOtherApps:YES];
    [NSApp hide:self];
    [NSApp stop:self];
  }
  
  float progress = sProgressVal;
  
  [progressBar setDoubleValue:(double)progress];
}

// leave this as returning a BOOL instead of NSApplicationTerminateReply
// for backward compatibility
- (BOOL)applicationShouldTerminate:(NSApplication *)sender
{
  return sQuit;
}

@end

int
InitProgressUI(int *pargc, char ***pargv)
{
  sUpdatePath = (*pargv)[1];
  
  return 0;
}

int
ShowProgressUI()
{
  // Only show the Progress UI if the process is taking a significant amount of
  // time where a significant amount of time is defined as .5 seconds after
  // ShowProgressUI is called sProgress is less than 70.
  usleep(500000);
  
  if (sQuit || sProgressVal > 70.0f)
    return 0;

  char path[PATH_MAX];
  snprintf(path, sizeof(path), "%s/updater.ini", sUpdatePath);
  if (ReadStrings(path, &sLabels) != OK)
    return -1;

  // Continue the update without showing the Progress UI if any of the supplied
  // strings are larger than MAX_TEXT_LEN (Bug 628829).
  if (!(strlen(sLabels.title) < MAX_TEXT_LEN - 1 &&
        strlen(sLabels.info) < MAX_TEXT_LEN - 1))
    return -1;
  
  [NSApplication sharedApplication];
  [NSBundle loadNibNamed:@"MainMenu" owner:NSApp];
  [NSApp run];

  return 0;
}

// Called on a background thread
void
QuitProgressUI()
{
  sQuit = TRUE;
}

// Called on a background thread
void
UpdateProgressUI(float progress)
{
  sProgressVal = progress;  // 32-bit writes are atomic
}
