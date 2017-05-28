/*
 *  Copyright (c) 2010 The WebM project authors. All Rights Reserved.
 *  Copyright (c) 2011, 2012 Contributors to TenFourFox. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


#include "vpx_config.h"
#include "vpx_ports/x86.h"
#include "vp8/common/subpixel.h"
#include "vp8/common/loopfilter.h"
#include "vp8/common/recon.h"
#include "vp8/common/idct.h"
#include "vp8/common/variance.h"
#include "vp8/common/pragmas.h"
#include "vp8/common/onyxc_int.h"

void vp8_arch_ppc_common_init(VP8_COMMON *ctx)
{
 /* Eventually this will do something useful. Right now it just takes
    up space, but fortunately it's not very good at that either. */
}
