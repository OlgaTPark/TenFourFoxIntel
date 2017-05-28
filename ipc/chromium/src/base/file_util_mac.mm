// Copyright (c) 2006-2008 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/file_util.h"

#import <Cocoa/Cocoa.h>
//#include <copyfile.h>

// Copyfile does exist on 10.4, but not copyfile.h, so we stub it in.
// This is more or less cribbed from Apple's public libc headers.
#if(0)
#include <stdint.h>
struct _copyfile_state;
typedef struct _copyfile_state * copyfile_state_t;
typedef uint32_t copyfile_flags_t;
int copyfile(const char *from, const char *to, copyfile_state_t state, copyfile_flags_t flags);
#define COPYFILE_ALL ((1<<0)|(1<<1)|(1<<2)|(1<<3))
#endif

#include "base/file_path.h"
#include "base/logging.h"
#include "base/string_util.h"

namespace file_util {

bool GetTempDir(FilePath* path) {
  NSString* tmp = NSTemporaryDirectory();
  if (tmp == nil)
    return false;
  *path = FilePath([tmp fileSystemRepresentation]);
  return true;
}

bool GetShmemTempDir(FilePath* path) {
  return GetTempDir(path);
}

bool CopyFile(const FilePath& from_path, const FilePath& to_path) {
#if(0)
  return (copyfile(from_path.value().c_str(),
                   to_path.value().c_str(), NULL, COPYFILE_ALL) == 0);
#else
  perror("CopyFile not supported on 10.4");
  return false;
#endif
}

}  // namespace
