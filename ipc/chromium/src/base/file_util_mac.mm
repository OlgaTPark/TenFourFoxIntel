// Copyright (c) 2006-2008 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/file_util.h"

#import <Cocoa/Cocoa.h>
#if(0)
// 10.4 no haz.
#include <copyfile.h>
#endif

#include "base/file_path.h"
#include "base/logging.h"
#include "base/string_util.h"
#include "base/scoped_nsautorelease_pool.h"

namespace file_util {

bool GetTempDir(FilePath* path) {
  base::ScopedNSAutoreleasePool autorelease_pool;
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
