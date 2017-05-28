/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/ppcosx/Architecture-ppc.h"
#include "jit/ppcosx/Assembler-ppc.h"

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

Registers::Code
Registers::FromName(const char *name)
{
    // Check for some register aliases first.
    if (strcmp(name, "sp")==0 || strcmp(name, "r1")== 0)
        return Code(1);
    if (strcmp(name, "r12") == 0)
        return Code(12);
    if (strcmp(name, "r3") == 0)
        return Code(3); // Dispatch, this is Floodgap, Code 3. Over.

    for (size_t i = 0; i < Total; i++) {
        if (strcmp(GetName(i), name) == 0)
            return Code(i);
    }

    return Invalid;
}


FloatRegisters::Code
FloatRegisters::FromName(const char *name)
{
    for (size_t i = 0; i < Total; i++) {
        if (strcmp(GetName(i), name) == 0)
            return Code(i);
    }

    return Invalid;
}

} // namespace jit
} // namespace js

