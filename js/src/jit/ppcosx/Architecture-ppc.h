/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/* TenFourFox's PowerPC IonMonkey port */

/* This is for a 32-bit PowerPC PowerOpen ABI system (i.e., OS X, probably
   AIX also). We run the G5 in 32-bit mode. */

#ifndef jsion_architecture_ppc_h__
#define jsion_architecture_ppc_h__

#include <limits.h>
#include "assembler/assembler/MacroAssembler.h"

namespace js {
namespace jit {

// This is how many bytes we push to the stack each time. Since we always push
// a word-aligned value, this is always 4.
static const ptrdiff_t STACK_SLOT_SIZE       = 4;
// We use two slots for a double precision float.
static const uint32_t DOUBLE_STACK_ALIGNMENT   = 2;

// In bytes: slots needed for potential memory->memory move spills.
// This can be up to 16 bytes, depending on what we pushed.
static const uint32_t ION_FRAME_SLACK_SIZE    = 16;

// An offset that is illegal for a local variable's stack allocation.
static const int32_t INVALID_STACK_SLOT       = -1;

// These offsets are specific to nunboxing, and capture offsets into the
// components of a js::Value.
// We are big-endian, unlike all those other puny little-endian architectures,
// so we use different constants. (type == tag)
// See MacroAssembler-ppc.h.
static const int32_t NUNBOX32_TYPE_OFFSET         = 0;
static const int32_t NUNBOX32_PAYLOAD_OFFSET      = 4;

////
// These offsets are related to bailouts.
////

// Currently this is the same as ARM, a single call, which clobbers LR.
// However, the ARM backend says this is actually wrong; the preferred
// version should probably be to move an immediate into a handy
// register followed by a branch.
// XXX
// For PPC I'm thinking this should be load CTR, move immediate to whatever,
// G5 nops and finally bctr, so variable size on G3/G4 versus G5, if we
// have to implement that.
static const uint32_t BAILOUT_TABLE_ENTRY_SIZE    = 4;

// GPRs
class Registers
{
  public:
    // Use the machinery we already set up for YARR/JM (R.I.P.).
    typedef JSC::PPCRegisters::RegisterID Code;
    typedef JSC::PPCRegisters::RegisterID RegisterID;

    static const char *GetName(Code code) {
        static const char *Names[] = {
             "r0",  "sp",  "toc", "r3",  "r4",  "r5",  "r6",  "r7",
             "r8",  "r9",  "r10", "r11", "r12", "r13", "r14", "r15",
             "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
             "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31"};
        return Names[code];
    }

    static const Code StackPointer = JSC::PPCRegisters::sp;
    static const Code Invalid = JSC::PPCRegisters::invalid_reg;

    // Ion can only handle up to 29 registers, and the generated trampoline
    // really ruins our day because we have to keep them all in flight. So we
    // restrict ourselves to r13 to r25, for now.
    static const uint32_t Total = 29;
    static const uint32_t Allocatable = 19;

    static const uint32_t AllMask = (1 << Total) - 1;
    static const uint32_t ArgRegMask =
        (1 << JSC::PPCRegisters::r3) |
        (1 << JSC::PPCRegisters::r4) |
        (1 << JSC::PPCRegisters::r5) |
        (1 << JSC::PPCRegisters::r6) |
        (1 << JSC::PPCRegisters::r7) |
        (1 << JSC::PPCRegisters::r8) |
        (1 << JSC::PPCRegisters::r9) |
        (1 << JSC::PPCRegisters::r10);

    static const uint32_t VolatileMask = ArgRegMask |
        (1 << JSC::PPCRegisters::r0)  |
        (1 << JSC::PPCRegisters::r1)  |
        (1 << JSC::PPCRegisters::r2)  |
        (1 << JSC::PPCRegisters::r11) |
        (1 << JSC::PPCRegisters::r12);

    static const uint32_t NonVolatileMask =
        (1 << JSC::PPCRegisters::r13) |
        (1 << JSC::PPCRegisters::r14) |
        (1 << JSC::PPCRegisters::r15) |
        (1 << JSC::PPCRegisters::r16) |
        (1 << JSC::PPCRegisters::r17) |
        (1 << JSC::PPCRegisters::r18) |
        (1 << JSC::PPCRegisters::r19) |
        (1 << JSC::PPCRegisters::r20) |
        (1 << JSC::PPCRegisters::r21) |
        (1 << JSC::PPCRegisters::r22) |
        (1 << JSC::PPCRegisters::r23) |
        (1 << JSC::PPCRegisters::r24) |
        (1 << JSC::PPCRegisters::r25) |
        (1 << JSC::PPCRegisters::r26) |
        (1 << JSC::PPCRegisters::r27) |
        (1 << JSC::PPCRegisters::r28);
/*
    // For future expansion.
        (1 << JSC::PPCRegisters::r29) |
        (1 << JSC::PPCRegisters::r30) |
        (1 << JSC::PPCRegisters::r31);
*/

    // Mirror ARM (r14/r15 are parallel with their Baseline R* selections)
    static const uint32_t WrapperMask = VolatileMask |
        (1 << JSC::PPCRegisters::r14) | // = outReg
        (1 << JSC::PPCRegisters::r15);  // = argBase

    static const uint32_t SingleByteRegs =
        VolatileMask | NonVolatileMask;

    static const uint32_t NonAllocatableMask =
        (1 << JSC::PPCRegisters::r0) |
        (1 << JSC::PPCRegisters::sp) |
        (1 << JSC::PPCRegisters::r2) | // maybe later
        (1 << JSC::PPCRegisters::r11) |
        (1 << JSC::PPCRegisters::r12) |
        // (r13-r15 are Baseline, see below)
        // Temporary stack work register that must be non-volatile.
        (1 << JSC::PPCRegisters::r16) |
        // Trampoline stack register.
        (1 << JSC::PPCRegisters::r17) |
        // ABI call work register that must be non-volatile.
        (1 << JSC::PPCRegisters::r18) |
        // r19 is used as the arguments rectifier, but must be allocatable.

        // Not yet allowed for the allocator.
        (1 << JSC::PPCRegisters::r26) |
        (1 << JSC::PPCRegisters::r27) |
        (1 << JSC::PPCRegisters::r28);

    // Registers that can be allocated without being saved, generally.
    static const uint32_t TempMask = VolatileMask & ~NonAllocatableMask;

    // Registers returned from a JS -> JS call.
    // ARM uses r2-r3 of their r0-r3 volatile block.
    static const uint32_t JSCallMask =
        (1 << JSC::PPCRegisters::r5) |
        (1 << JSC::PPCRegisters::r6);

    // Registers returned from a JS -> C call.
    // Seems we could do better here with all our argregs ...
    static const uint32_t CallMask =
        (1 << JSC::PPCRegisters::r3) |
        (1 << JSC::PPCRegisters::r4);  // used for double-size returns

    static const uint32_t AllocatableMask = 
        // Be explicit
        (1 << JSC::PPCRegisters::r3) |
        (1 << JSC::PPCRegisters::r4) |
        (1 << JSC::PPCRegisters::r5) |
        (1 << JSC::PPCRegisters::r6) |
        (1 << JSC::PPCRegisters::r7) |
        (1 << JSC::PPCRegisters::r8) |
        (1 << JSC::PPCRegisters::r9) |
        (1 << JSC::PPCRegisters::r10) |
        // Baseline registers (these must be "allocatable" even though
        // they're not)
        (1 << JSC::PPCRegisters::r13) |
        (1 << JSC::PPCRegisters::r14) |
        (1 << JSC::PPCRegisters::r15) |
        // Free non-volatiles
        (1 << JSC::PPCRegisters::r19) |
        (1 << JSC::PPCRegisters::r20) |
        (1 << JSC::PPCRegisters::r21) |
        (1 << JSC::PPCRegisters::r22) |
        (1 << JSC::PPCRegisters::r23) |
        (1 << JSC::PPCRegisters::r24) |
        (1 << JSC::PPCRegisters::r25) ;
};

// Smallest integer type that can hold a register bitmask.
typedef uint32_t PackedRegisterMask;

// FPRs (ARM calls these "dxx")
class FloatRegisters
{
  public:
    typedef JSC::PPCRegisters::FPRegisterID Code;

    static const char *GetName(Code code) {
        static const char *Names[] = {
            "f0",  "f1",  "f2",  "f3",  "f4",  "f5",  "f6",  "f7",
            "f8",  "f9",  "f10", "f11", "f12", "f13", "f14", "f15",
            "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
            "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
        };
        return Names[code];
    }

    static const Code Invalid = JSC::PPCRegisters::invalid_freg;

    // By declaring the allocator can only use the volatile FPRs, this
    // saves us a great deal of stack space because we don't have to save
    // anything. (Don't allocate f0/1, though.) Ion can only use 29 FPRs.
    static const uint32_t Total = 29;
    static const uint32_t Allocatable = 12;

    static const uint32_t AllMask = (1 << Total) - 1;

    static const uint32_t VolatileMask = 
        (1 << JSC::PPCRegisters::f0) |
        (1 << JSC::PPCRegisters::f1) |
        (1 << JSC::PPCRegisters::f2) |
        (1 << JSC::PPCRegisters::f3) |
        (1 << JSC::PPCRegisters::f4) |
        (1 << JSC::PPCRegisters::f5) |
        (1 << JSC::PPCRegisters::f6) |
        (1 << JSC::PPCRegisters::f7) |
        (1 << JSC::PPCRegisters::f8) |
        (1 << JSC::PPCRegisters::f9) |
        (1 << JSC::PPCRegisters::f10) |
        (1 << JSC::PPCRegisters::f11) |
        (1 << JSC::PPCRegisters::f12) |
        (1 << JSC::PPCRegisters::f13);

    static const uint32_t NonVolatileMask = 
        (1 << JSC::PPCRegisters::f14) |
        (1 << JSC::PPCRegisters::f15) |
        (1 << JSC::PPCRegisters::f16) |
        (1 << JSC::PPCRegisters::f17) |
        (1 << JSC::PPCRegisters::f18) |
        (1 << JSC::PPCRegisters::f19) |
        (1 << JSC::PPCRegisters::f20) |
        (1 << JSC::PPCRegisters::f21) |
        (1 << JSC::PPCRegisters::f22) |
        (1 << JSC::PPCRegisters::f23) |
        (1 << JSC::PPCRegisters::f24) |
        (1 << JSC::PPCRegisters::f25) |
        (1 << JSC::PPCRegisters::f26) |
        (1 << JSC::PPCRegisters::f27) |
        (1 << JSC::PPCRegisters::f28);
/*
 * Future expansion.
        (1 << JSC::PPCRegisters::f29) |
        (1 << JSC::PPCRegisters::f30) |
        (1 << JSC::PPCRegisters::f31);
 */

    static const uint32_t WrapperMask = VolatileMask;

    // The allocator is not allowed to use f0 (the scratch FPR), nor any of
    // the non-volatile registers, nor f1 for some routines. We also hide
    // f2 from the allocator in case we need it for convertInt32ToDouble.
    static const uint32_t NonAllocatableMask =
        (1 << JSC::PPCRegisters::f0) |
        (1 << JSC::PPCRegisters::f1) | // for things like software sqrt
        (1 << JSC::PPCRegisters::f2) | // for conversion in Baseline
        NonVolatileMask;

    // Registers that can be allocated without being saved, generally.
    static const uint32_t TempMask = VolatileMask & ~NonAllocatableMask;

    static const uint32_t AllocatableMask = 
        // Be explicit
        (1 << JSC::PPCRegisters::f3) |
        (1 << JSC::PPCRegisters::f4) |
        (1 << JSC::PPCRegisters::f5) |
        (1 << JSC::PPCRegisters::f6) |
        (1 << JSC::PPCRegisters::f7) |
        (1 << JSC::PPCRegisters::f8) |
        (1 << JSC::PPCRegisters::f9) |
        (1 << JSC::PPCRegisters::f10) |
        (1 << JSC::PPCRegisters::f11) |
        (1 << JSC::PPCRegisters::f12) |
        (1 << JSC::PPCRegisters::f13);

};

// SPRs
// These have no peer in lesser chips. That is because PPC has no peer in
// lesser chips. These don't count against the register cap because the
// allocator is unaware of them. In fact, we don't treat these as regular
// registers at all (hey, they're Special Purpose anyway); they are
// intentionally *not* part of js::jit::Registers.

    typedef JSC::PPCRegisters::SPRegisterID SPRegisterID;

// Suppress warning if not used
#if(0)
    static const char *getSPRName(SPRegisterID code) {
#define XXX "INVALID"
        static const char *N_vrsave = "vrsave";
        static const char *N_bogus = XXX;
        static const char *Names[] = {
            XXX, "xer", XXX, XXX, XXX, XXX, XXX, XXX,
            "lr","ctr"
        };
#undef XXX
        return 
               (code == JSC::PPCRegisters::vrsave) ? N_vrsave :
               (code >  JSC::PPCRegisters::ctr)    ? N_bogus :
               Names[code];
    }
#endif

// CRs
// We have eight condition registers, each for how unconditionally wonderful
// PowerPC is, and sometimes for storing condition results.
// These aren't GPRs either, so the allocator remains blissfully unaware of
// them too, and we don't treat them as part of js::jit::Registers for the
// same reason.

    typedef JSC::PPCRegisters::CRegisterID CRegisterID;

// Suppress warning if not used
#if(0)
    static const char *getCRName(CRegisterID code) {
        static const char *Names[] = {
            "cr0",  "cr1",  "cr2",  "cr3",  "cr4",  "cr5",  "cr6",  "cr7"
        };
        return Names[code];
    }
#endif

bool has_altivec();
bool has_sqrt();

} // namespace jit
} // namespace js

#endif // jsion_architecture_ppc_h__
