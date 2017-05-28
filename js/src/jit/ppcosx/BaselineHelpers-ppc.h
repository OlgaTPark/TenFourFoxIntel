/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#if !defined(jsion_baseline_helpers_ppc_h__) && defined(JS_ION)
#define jsion_baseline_helpers_ppc_h__

#include "jit/IonMacroAssembler.h"
#include "jit/BaselineFrame.h"
#include "jit/BaselineRegisters.h"
#include "jit/BaselineIC.h"

/* Unlike most of the Ion code, the Baseline IC helpers use LR as a
   return register. We assume there are not calls within calls, though
   we do push LR if there is that risk. */

namespace js {
namespace jit {

// Distance from sp to the top Value inside an IC stub. Since we don't
// store that on the stack, this is zero on PPC, like ARM.
static const size_t ICStackValueOffset = 0;

inline void
EmitRestoreTailCallReg(MacroAssembler &masm)
{
    // Nothing to do on PPC.
}

inline void
EmitCallIC(CodeOffsetLabel *patchOffset, MacroAssembler &masm)
{
    ispew("[[ EmitCallIC");

    // Move ICEntry offset into BaselineStubReg
    CodeOffsetLabel offset = masm.movWithPatch(ImmWord(-1), BaselineStubReg);
    *patchOffset = offset;

    // Load stub pointer into BaselineStubReg
    masm.loadPtr(Address(BaselineStubReg, ICEntry::offsetOfFirstStub()),
        BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry.
    // Baseline R2 won't be active when we call ICs, so we can use r3.
    JS_ASSERT(R2 == ValueOperand(r4, r3));
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r3);

    // Call the stubcode. Use a full call stanza just in case it calls a
    // VMWrapper and the LR gets "adjusted." ARM does not save LR, so neither
    // do we.
    masm.call(r3);

    ispew("   EmitCallIC ]]");
}

inline void
EmitEnterTypeMonitorIC(MacroAssembler &masm,
                       size_t monitorStubOffset = ICMonitoredStub::offsetOfFirstMonitorStub())
{
    ispew("[[ EmitEnterTypeMonitorIC");

    // This is expected to be called from within an IC, when BaselineStubReg
    // is properly initialized to point to the stub.
    masm.loadPtr(Address(BaselineStubReg, (uint32_t) monitorStubOffset),
        BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry.
    // Baseline R2 won't be active when we call ICs, so we can use r3.
    JS_ASSERT(R2 == ValueOperand(r4, r3));
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r3);

    // Jump to the stubcode (not as a subroutine).
// XXX Consider rescheduling this.
    masm.x_baa(r3);

    ispew("   EmitEnterTypeMonitorIC ]]");
}

inline void
EmitReturnFromIC(MacroAssembler &masm)
{
    masm.blr(); // This actually *is* a blr (ARM does mov lr,pc).
}

inline void
EmitChangeICReturnAddress(MacroAssembler &masm, Register reg)
{
    masm.x_mtlr(reg);
}

inline void
EmitTailCallVM(IonCode *target, MacroAssembler &masm, uint32_t argSize)
{
    ispew("[[ EmitTailCallVM");
    // We assume during this that Baseline R0 and R1 have been pushed, and
    // that Baseline R2 is unused. The argregs will not be set until we
    // actually call the VMWrapper, so we can trample R2's set.
    JS_ASSERT(R2 == ValueOperand(r4, r3));

    // Compute frame size.
    masm.x_mr(r3, BaselineFrameReg);
    masm.add32(Imm32(BaselineFrame::FramePointerOffset), r3, r3);
    masm.subf(r3, BaselineStackReg, r3); // r3 = r3 - sp (fp > sp)

    // Store frame size without VMFunction arguments for GC marking.
    masm.sub32(Imm32(argSize), r3, r4);
    masm.store32(r4,
        Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfFrameSize()));

    // Push frame descriptor and perform the tail call. The call is to
    // Ion code, so it does not need to be ABI compliant. LR contains the
    // return address, which the VMWrapper will push. We branch rather
    // than call, because this is the end, my friend.
    masm.x_mflr(BaselineTailCallReg);
    masm.makeFrameDescriptor(r3, IonFrame_BaselineJS);
    masm.push2(r3, BaselineTailCallReg); // match ARM
    masm.b(target);

    ispew("   EmitTailCallVM ]]");
}

inline void
EmitCreateStubFrameDescriptor(MacroAssembler &masm, Register reg)
{
    // Compute stub frame size. We have to add two pointers: the stub reg
    // and previous fake-o frame pointer pushed by EmitEnterStubFrame.
    ispew("EmitCreateStubFrameDescriptor");
    JS_ASSERT(reg != r0); // this will turn the addi into an li!
    masm.x_mr(reg, BaselineFrameReg);
    masm.addi(reg, reg, sizeof(void *) * 2); // should be 8, should fit
    masm.subf(reg, BaselineStackReg, reg); // fp > sp
    masm.makeFrameDescriptor(reg, IonFrame_BaselineStub);
}

inline void
EmitCallVM(IonCode *target, MacroAssembler &masm)
{
    ispew("[[ EmitCallVM");

    // Use r12, since we can't use r0 with the addi.
    EmitCreateStubFrameDescriptor(masm, r12);
    masm.push(r12);
    masm.call(target);

    ispew("   EmitCallVM ]]");
}

// Size of values pushed by EmitEnterStubFrame.
static const uint32_t STUB_FRAME_SIZE = 4 * sizeof(void *);
static const uint32_t STUB_FRAME_SAVED_STUB_OFFSET = sizeof(void *);

// EmitEnterStubFrame depends on this equivalence.
JS_STATIC_ASSERT(PPC_CALL_STANZA_LENGTH == PPC_BRANCH_STANZA_LENGTH);

inline void
EmitEnterStubFrame(MacroAssembler &masm, Register scratch)
{
    ispew("[[ EmitEnterStubFrame");
    JS_ASSERT(scratch != BaselineTailCallReg);

    // Compute frame size.
    masm.x_mr(scratch, BaselineFrameReg);
    masm.add32(Imm32(BaselineFrame::FramePointerOffset), scratch);
    masm.subf(scratch, BaselineStackReg, scratch);

    masm.store32(scratch,
        Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfFrameSize()));

    // Push frame descriptor and return address. Remember, LR is not a
    // GPR, so we need to mflr it first to push the return address. The
    // return address may be inside of a call stanza, which BC/Ion need
    // to account for, since we can't assume the LR originated from one here.
    masm.makeFrameDescriptor(scratch, IonFrame_BaselineJS);
    masm.x_mflr(BaselineTailCallReg);
    masm.addi(stackPointerRegister, stackPointerRegister, -16);
    masm.stw(scratch, stackPointerRegister, 12);
    masm.stw(BaselineTailCallReg, stackPointerRegister, 8);

    // Save old frame pointer, stack pointer and stub reg.
    // We don't care about stack alignment here.
    masm.stw(BaselineStubReg, stackPointerRegister, 4);
    masm.stw(BaselineFrameReg, stackPointerRegister, 0);
    JS_ASSERT(BaselineStackReg == stackPointerRegister);
    masm.x_mr(BaselineFrameReg, BaselineStackReg);

    // We pushed four words.
    // Note: when making changes here, don't forget to update STUB_FRAME_SIZE
    // if needed.
    ispew("   EmitEnterStubFrame ]]");
}

inline void
EmitLeaveStubFrame(MacroAssembler &masm, bool calledIntoIon = false)
{
    ispew("[[ EmitLeaveStubFrame ");
    // Ion frames do not save and restore the frame pointer. If we called
    // into Ion, we have to restore the stack pointer from the frame descriptor.
    // If we performed a VM call, the descriptor has been popped already so
    // in that case we use the frame pointer.
    // This can happen even if we don't actually create Ion code, btw.
    if (calledIntoIon) {
        // Unwind the descriptor.
        masm.pop(ScratchRegister);
        masm.x_srwi(ScratchRegister, ScratchRegister, FRAMESIZE_SHIFT);
        masm.add(BaselineStackReg, ScratchRegister, BaselineStackReg);
    } else {
        masm.x_mr(BaselineStackReg, BaselineFrameReg);
    }

    masm.pop4(BaselineFrameReg,
        BaselineStubReg,
        BaselineTailCallReg,
        ScratchRegister);

    // Load the return address.
    masm.x_mtlr(BaselineTailCallReg);

    ispew("   EmitLeaveStubFrame ]]");
}

inline void
EmitStowICValues(MacroAssembler &masm, int values)
{
    JS_ASSERT(values >= 0 && values <= 2);
    switch(values) {
      case 1:
        // Stow R0
        masm.pushValue(R0);
        break;
      case 2:
        // Stow R0 and R1
        masm.push4(R0.payloadReg(), R0.typeReg(),
            R1.payloadReg(), R1.typeReg());
        break;
    }
}

inline void
EmitUnstowICValues(MacroAssembler &masm, int values)
{
    JS_ASSERT(values >= 0 && values <= 2);
    switch(values) {
      case 1:
        // Unstow R0
        masm.popValue(R0);
        break;
      case 2:
        // Unstow R0 and R1
        masm.pop4(R1.typeReg(), R1.payloadReg(),
            R0.typeReg(), R0.payloadReg());
        break;
    }
}

inline void
EmitCallTypeUpdateIC(MacroAssembler &masm, IonCode *code, uint32_t objectOffset)
{
    ispew("[[ EmitCallTypeUpdateIC");
    // Baseline R0 contains the value that needs to be typechecked.
    // The object we're updating is a boxed Value on the stack, at offset
    // objectOffset from esp, excluding the return address. R2 should be
    // free, so we can use it as temporary registers.
    JS_ASSERT(R2 == ValueOperand(r4, r3));

    // Save LR so we can get it back. We can't just use r18; the stubcode
    // might call a VMWrapper.
    masm.x_mflr(BaselineTailCallReg); // new dispatch group on G5
    masm.addi(r1, r1, -8);
    masm.stw(BaselineStubReg, r1, 4); // "push"
    masm.stw(BaselineTailCallReg, r1, 0); // "push"

    // This is expected to be called from within an IC, when BaselineStubReg
    // is properly initialized to point to the stub. Since we are inside an
    // IC we ourselves generated, ABI-compliance is unimportant.
    masm.loadPtr(Address(BaselineStubReg,
                ICUpdatedStub::offsetOfFirstUpdateStub()),
                BaselineStubReg);

    // Load stubcode pointer from BaselineStubReg into BaselineTailCallReg.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r3);

    // Call the stubcode. Use a full call stanza. Don't push PC; we have LR.
    masm.call(r3);

    // Restore the old stub reg and tailcall reg (and, indirectly, LR).
    masm.lwz(BaselineTailCallReg, r1, 0);
    masm.x_mtlr(BaselineTailCallReg); // new dispatch group
    masm.lwz(BaselineStubReg, r1, 4);
    masm.addi(r1, r1, 8); // "pop" "pop"

    // The update IC will store 0 or 1 in R1.scratchReg() reflecting if the
    // value in R0 type-checked properly or not.
    Label success;
    masm.cmpwi(R1.scratchReg(), 1);
    masm.bc(Assembler::Equal, &success);

    // If the IC failed, then call the update fallback function.
    EmitEnterStubFrame(masm, R1.scratchReg());
    masm.loadValue(Address(BaselineStackReg,
        STUB_FRAME_SIZE + objectOffset), R1);
    masm.push5(R0.payloadReg(), R0.typeReg(),
        R1.payloadReg(), R1.typeReg(),
        BaselineStubReg);

    // Load previous frame pointer, push BaselineFrame *.
    masm.loadPtr(Address(BaselineFrameReg, 0), R0.scratchReg());
    masm.pushBaselineFramePtr(R0.scratchReg(), R0.scratchReg());

    EmitCallVM(code, masm);
    EmitLeaveStubFrame(masm);

    // Success at end.
    masm.bind(&success);
    ispew("   EmitCallTypeUpdateIC ]]");
}

template <typename AddrType>
inline void
EmitPreBarrier(MacroAssembler &masm, const AddrType &addr, MIRType type)
{
    ispew("[[ EmitPreBarrier");
    // On PPC, LR is clobbered by patchableCallPreBarrier because it can
    // call other functions.
    masm.x_mflr(r0);
    masm.push(r0);
    masm.patchableCallPreBarrier(addr, type);
    masm.pop(r0);
    masm.x_mtlr(r0);

    ispew("   EmitPreBarrier ]]");
}

inline void
EmitStubGuardFailure(MacroAssembler &masm)
{
    ispew("[[ EmitStubGuardFailure");
    // We assume Baseline R2 and component registers are free.
    JS_ASSERT(R2 == ValueOperand(r4, r3));

    // NOTE: This routine assumes that the stub guard code left the stack in
    // the same state it was in when it was entered.
    // BaselineStubEntry points to the current stub.

    // Load next stub into BaselineStubReg
    masm.loadPtr(Address(BaselineStubReg,
        ICStub::offsetOfNext()), BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry into scratch register.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r3);
    masm.x_mtctr(r3);
    masm.x_mflr(BaselineTailCallReg); // make it act like ARM

    // mflr makes a new dispatch group, so we don't need nops for G5.
    // Return address is already loaded in LR, so just jump.
    masm.bctr();

    ispew("   EmitStubGuardFailure ]]");
}


} // namespace jit
} // namespace js

#endif
