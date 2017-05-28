/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

namespace js {
namespace jit {

bool has_altivec() {
#ifdef TENFOURFOX_VMX
    return true;
#else
    return false;
#endif
}

bool has_sqrt() {
#ifdef TENFOURFOX_G5
    return true;
#else
    return false;
#endif
}

} // namespace jit
} // namespace js

