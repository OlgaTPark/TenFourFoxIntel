#include "jit/ppcosx/MacroAssembler-ppc.h"
#include "jit/BaselineFrame.h"
#include "jit/MoveEmitter.h"
#include "jit/ppcosx/BaselineRegisters-ppc.h"

namespace js {
namespace jit {

bool
MacroAssemblerPPC::buildFakeExitFrame(const Register &scratch,
	uint32_t *offset) {
	ispew("buildFakeExitFrame(reg, uint32_t)");
	JS_ASSERT(scratch != addressTempRegister);

	// Builds an exit frame on the stack, with a return address to an
	// internal non-function. Returns offset to be passed to
	// markSafepointAt(). ARM: push descriptor, push pc

	mozilla::DebugOnly<uint32_t> initialDepth = framePushed();

	// Because this is LR-relative, do not let a constant pool get
	// inserted in the middle.
	ensureSpace(32); // paranoia
	x_b(4, Assembler::LinkB); // force dispatch group on G5
	x_mflr(addressTempRegister); // new dispatch group // (0)
	uint32_t descriptor = MakeFrameDescriptor(framePushed(),
		IonFrame_OptimizedJS);
	JS_ASSERT(descriptor < 65536);
	x_li(tempRegister, descriptor); // (4)
	Push(tempRegister); // stwu // cracked // (8)
	addi(addressTempRegister, addressTempRegister, 20); // (12)
	Push(addressTempRegister); // stwu // cracked // (16)

	// Emit offset here.
	*offset = currentOffset();
	// (20)

	JS_ASSERT(framePushed() == initialDepth + IonExitFrameLayout::Size());
	return true;
}

bool
MacroAssemblerPPC::buildOOLFakeExitFrame(void *fakeReturnAddr) {
	ispew("buildOOLFakeExitFrame(void *)");

	uint32_t descriptor = MakeFrameDescriptor(framePushed(),
		IonFrame_OptimizedJS);
	// TODO: We could optimize this a bit better with multi-push.
	Push(Imm32(descriptor));
	Push(ImmWord(fakeReturnAddr));
	return true;
}

void
MacroAssemblerPPC::callWithExitFrame(IonCode *target) {
	ispew("callWithExitFrame(ion *)");

	uint32_t descriptor = MakeFrameDescriptor(framePushed(),
		IonFrame_OptimizedJS);
	Push(Imm32(descriptor));
	call(target); // Use the IonCode version, not the PPCAssembler.
}

// WARNING! TRAMPOLINE DEPENDS ON THIS FUNCTION BEING 24 BYTES!
// Alter the portion where the PC is computed in generateEnterJIT if this
// is changed.
void
MacroAssemblerPPC::callIon(const Register &callee) {
	ispew("callIon(reg)");

	// Force a new dispatch group and start the CTR branch move.
	x_mtctr(callee);

	// Push PC. This must occur here, since we don't save it later. Since
	// we never restore LR, assume we can clobber it. We can do this in
	// two dispatch groups because we're awesome. However, because this is
	// LR-relative, make sure a constant pool doesn't get inserted.
	ensureSpace(24);
	x_b(4, Assembler::LinkB); // force dispatch group on G5
	x_mflr(addressTempRegister); // new dispatch group // (0)
	// Make the PC point to the instruction after the bctr. Don't use r0!
	addi(addressTempRegister, addressTempRegister, 16); // (4)
	// framePushed_ doesn't count the PC, so stwu directly to stack.
	stwu(addressTempRegister, stackPointerRegister, -4); // (8)

	// Since IonCode is not ABI compliant, we can just call it.
	bctr(); // (12) (branch slot)

	// Return here (16)
}

// Setup a call to C/C++ code, given the number of general arguments it
// takes.
//
// In order for alignment to work correctly, the MacroAssembler must have a
// consistent view of the stack displacement. It is okay to call "push"
// manually; however, if the stack alignment were to change, the macro
// assembler should be notified before starting a call.
//
// However, for us it doesn't matter; we dynamically compute an ABI
// compliant frame upon calls, so there's really no difference.
void
MacroAssemblerPPC::setupABICall(uint32_t args) {
	JS_ASSERT(!inCall_);
	inCall_ = true;
	// args_ = args;
	passedGPRs_ = 0;
	passedFPRs_ = 0;
}

void
MacroAssemblerPPC::setupAlignedABICall(uint32_t args) {
	ispew("setupAlignedABICall");
	setupABICall(args);
}

// Sets up an ABI call for when the alignment is not known. We actually do
// the stack alignment later, but this is here for consistency.
void
MacroAssemblerPPC::setupUnalignedABICall(uint32_t args,
	const Register &scratch) {

	ispew("setupUnalignedABICall");
	setupABICall(args);
}

// Arguments must be assigned to a C/C++ call in order. They are moved
// in parallel immediately before performing the call. No operations
// should be emitted while setting arguments.
// XXX: This routine does not yet handle the case where there are more
// arguments than argregs. Given how many argregs PPC has, this should
// be confidently rare.
void
MacroAssemblerPPC::passABIArg(const MoveOperand &from) {
#if DEBUG
	IonSpew(IonSpew_Codegen, "state, passABIArg: gprs %i fprs %i\n",
		passedGPRs_, passedFPRs_);
#endif

	if (!enoughMemory_) return;
	if (from.isDouble()) {
		FloatRegister fpr;

		// PPC OSX ABI: f1-f13 are parameter registers
		if (++passedFPRs_ > 13) {
			// If we have more than 13 FPR argregs, OMFG.
			JS_NOT_REACHED("FPR ABI argument overflow");
		} else {
			fpr = FloatRegister::FromCode(
				(FloatRegister::Code)passedFPRs_);
			// All values are treated as doubles, so skip
			// two GPRs as per the OS X ABI Function Call guide.
			passedGPRs_ += 2;
		}
		if (!from.isFloatReg() // i.e., we need to convert
				|| from.floatReg() != fpr)
			enoughMemory_ = moveResolver_.addMove(from,
				MoveOperand(fpr),
				Move::DOUBLE);
	} else {
		Register gpr;

		// PPC OSX ABI: r3-r10 are parameter registers
		if (++passedGPRs_ > 8) {
			// This might be a little more frequent ...
			JS_NOT_REACHED("GPR ABI argument overflow");
		} else {
			gpr = Register::FromCode(
				(Register::Code)2+passedGPRs_);
		}
		if (!from.isGeneralReg() // i.e., we need to convert
				|| from.reg() != gpr)
			enoughMemory_ = moveResolver_.addMove(from,
				MoveOperand(gpr),
				Move::GENERAL);
	}
}
void
MacroAssemblerPPC::passABIArg(const Register &gpr) {
	passABIArg(MoveOperand(gpr));
}
void
MacroAssemblerPPC::passABIArg(const FloatRegister &fpr) {
	passABIArg(MoveOperand(fpr));
}

static const Register stackSave = r16;

// XXX: We could probably get rid of the stackAdjust, we don't use it.
void
MacroAssemblerPPC::callWithABIPre(uint32_t *stackAdjust) {
	JS_ASSERT(inCall_);
	JS_ASSERT(!*stackAdjust); // XXX

	// Position all arguments. It's safe to use r12 here since this is
	// prior to loading CTR.
	{
	    enoughMemory_ = enoughMemory_ && moveResolver_.resolve();
	    if (!enoughMemory_)
	        return;

	    MoveEmitter emitter(*this);
	    emitter.emit(moveResolver_);
	    emitter.finish();
	}

	// At this point we are ready to call. We need to make our stack
	// ABI compliant by adding any additional padding to make it
	// quadword aligned, and then pulling down a dummy frame. This
	// dummy frame should be large, because we could call *anything*.
	// At the end state, it should just look to any routine we call
	// merely as if the trampoline simply enlarged the stack frame
	// it made, complete with new phony linkage area.

	// Align to quadword.
	andi_rc(tempRegister, stackPointerRegister, 15);
	// Save the old stack pointer to r16 so we can get it back after
	// the call without screwing up the ABI-compliant routine's ability
	// to walk stack frames.
	x_mr(stackSave, stackPointerRegister);
	subf(stackPointerRegister, tempRegister, stackPointerRegister);
	// Pull down (empiric?) 256 byte stack frame.
	x_subi(stackPointerRegister, stackPointerRegister, 256);
	// Store r18 as the phony backreference. This lets the called
	// routine skip anything Ion or Baseline may have pushed on stack.
	// (The trampoline saved this way back when.)
	stw(r18, stackPointerRegister, 0);
	// Now that r18 is saved, put the link register there. Since this is
	// an ABI-compliant routine we are calling, it's safe for this call.
	x_mflr(r18);

#if DEBUG
	// Make sure we're aligned before branching!
	checkStackAlignmentPriorToABICall();
#endif
}
// Clean up.
void
MacroAssemblerPPC::callWithABIPost(uint32_t stackAdjust, Result result) {
	// Get back LR from non-volatile r18.
	x_mtlr(r18);
	// Get back the original value of r18 for future calls.
	lwz(r18, stackPointerRegister, 0);
	// Restore the stack pointer, destroying the dummy frame and
	// the alignment stuffage.
	x_mr(stackPointerRegister, stackSave);
	JS_ASSERT(inCall_);
	inCall_ = false;
}

// Emits a call to a C/C++ function, resolving all argument moves.
// This takes an absolute address, so we need to hardcode it.
void
MacroAssemblerPPC::callWithABI(void *fun, Result result = GENERAL) {
	uint32_t x = 0;
	ispew("[[ callWithABI ");
	callWithABIPre(&x);

	// Warning: all moves must have been resolved by this point.
	// Create a call and set it to the destination, since it's fixed.
	addPendingCall(masm.m_call(), fun, Relocation::HARDCODED);
	
	callWithABIPost(x, result);
	ispew("   callWithABI ]]");
}
void
MacroAssemblerPPC::callWithABI(const Address &fun, Result result = GENERAL) {
	uint32_t x = 0;
	ispew("[[ callWithABI ");
	callWithABIPre(&x);

	load32(fun, addressTempRegister);
	x_mtctr(addressTempRegister);
#if defined(_PPC970_)
	x_nop();
	x_nop();
	x_nop();
	x_nop();
#endif
	bctrl();
	callWithABIPost(x, result);
	ispew("   callWithABI ]]");
}

JS_STATIC_ASSERT(FRAMESIZE_SHIFT < 16);
void
MacroAssemblerPPC::makeFrameDescriptor(Register frameSizeReg, FrameType type) {
	ispew("makeFrameDescriptor(r, FrameType)");
	JS_ASSERT(PPC_IMM_OK_U((uint16_t)type));

	x_slwi(frameSizeReg, frameSizeReg, FRAMESIZE_SHIFT);
	ori(frameSizeReg, frameSizeReg, (uint16_t)type);
}

// Save an exit frame (which must be aligned to the stack pointer) to
// ThreadData::ionTop.
void
MacroAssemblerPPC::linkExitFrame() {
	ispew("linkExitFrame");

	JSCompartment *compartment = GetIonContext()->compartment;
	uint32_t dest = (uint32_t)&(compartment->rt->mainThread.ionTop);
	// Use r12 as the effective address.
	x_li32(addressTempRegister, dest);
	stw(stackPointerRegister, addressTempRegister, 0);
}

void
MacroAssemblerPPC::callWithExitFrame(IonCode *target, Register dynStack) {
	ispew("callWithExitFrame");

	addPtr(Imm32(framePushed()), dynStack);
	makeFrameDescriptor(dynStack, IonFrame_OptimizedJS);
	Push(dynStack);
	call(target);
}

void
MacroAssemblerPPC::enterOsr(Register calleeToken, Register code) {
	ispew("enterOsr");

	// We aren't tracking amount pushed here, so ignore framePushed_.
	x_li(tempRegister, 0); // num actual args (always zero)
	addi(stackPointerRegister, stackPointerRegister, -12);
	stw(tempRegister, stackPointerRegister, 8);
	x_li32(tempRegister, MakeFrameDescriptor(0, IonFrame_Osr));
	stw(calleeToken, stackPointerRegister, 4);
	stw(tempRegister, stackPointerRegister, 0); // descriptor

	callIon(code);

	// Because the frame descriptor is popped, we only need to remove
	// the two other items from the stack.
	addi(stackPointerRegister, stackPointerRegister, 8);
}

void
MacroAssemblerPPC::linkParallelExitFrame(const Register &reg) {
	ispew("linkParallelExitFrame");
	// Given the provided register, store the stack pointer there using
	// it as an offset to PerThreadData.
	stw(stackPointerRegister, reg, offsetof(PerThreadData, ionTop));
}

void
MacroAssemblerPPC::handleFailureWithHandler(void *handler) {
	ispew("handleFailureWithHandler");
	// Call an ABI-compliant handler function, reserving adequate space
	// on the stack for a non-ABI compliant exception frame which we will
	// then analyse.

	// ResumeFromException is defined in IonFrames.h
	int size = (sizeof(ResumeFromException) + 16) & ~16;
	x_subi(stackPointerRegister, stackPointerRegister, size);
	x_mr(r3, stackPointerRegister);

	setupUnalignedABICall(1, r4);
	passABIArg(r3);
	callWithABI(handler);

	Label entryFrame;
	Label catch_;
	Label finally;
	Label return_;

	// Now, analyse the returned frame.
	// Get the type.
	lwz(r3, stackPointerRegister, offsetof(ResumeFromException, kind));
	branch32(Assembler::Equal, r3,
		Imm32(ResumeFromException::RESUME_ENTRY_FRAME), &entryFrame);
    	branch32(Assembler::Equal, r3,
		Imm32(ResumeFromException::RESUME_CATCH), &catch_);
    	branch32(Assembler::Equal, r3,
		Imm32(ResumeFromException::RESUME_FINALLY), &finally);
    	branch32(Assembler::Equal, r3,
		Imm32(ResumeFromException::RESUME_FORCED_RETURN), &return_);
	x_trap(); // Oops.

	bind(&entryFrame);
	// Entry frame case: There is no exception handler. Load the error
	// value, load the new stack pointer, and return.
	lwz(stackPointerRegister, stackPointerRegister,
		offsetof(ResumeFromException, stackPointer));
	// Pop the new PC (ARM: ldr pc, [sp]!) and load it for CTR branching.
	lwz(r3, stackPointerRegister, 0);
	x_mtctr(r3); // new dispatch group
	addi(stackPointerRegister, stackPointerRegister, 4);
	moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
#if defined(_PPC970_)
	// moveValue should be at least two instructions, so use one nop
	// to pad the branch slot.
	x_nop();
#endif
	bctr();

	bind(&catch_);
	// Catch handler case: we found a baseline frame which we can use.
	// Restore state and jump to it; first, get the location of the
	// baseline frame code.
	lwz(r3, stackPointerRegister, offsetof(ResumeFromException, target));
	x_mtctr(r3); // force a new dispatch group here for G5
	// Then get the old BaselineFrameReg,
	lwz(BaselineFrameReg, stackPointerRegister,
		offsetof(ResumeFromException, framePointer));
	// get the old stack pointer,
	lwz(stackPointerRegister, stackPointerRegister,
		offsetof(ResumeFromException, stackPointer));
#if defined(_PPC970_)
	x_nop();
	x_nop(); // keep the bctr out of the branch slot
#endif
	// and branch.
	bctr();

	bind(&finally);
	// Finally block case: this is also a baseline frame. Push the two
	// values required by JSOP_RETSUB (Boolean true and the exception)
	// before we branch.
	ValueOperand exception = ValueOperand(r4, r5);
	loadValue(Operand(stackPointerRegister,
		offsetof(ResumeFromException, exception)),
			exception);

	// Then prepare to branch,
	lwz(r3, stackPointerRegister, offsetof(ResumeFromException, target));
	x_mtctr(r3); // force a new dispatch group here for G5
	// get the old BaselineFrameReg,
	lwz(BaselineFrameReg, stackPointerRegister,
		offsetof(ResumeFromException, framePointer));
	// and get the old stack pointer.
	lwz(stackPointerRegister, stackPointerRegister,
		offsetof(ResumeFromException, stackPointer));

	// Onto the stack push Boolean true and the exception. These will
	// confidently be in a second dispatch group, so no nops are needed
	// for G5 to allow the mtctr to complete instruction fetch.
	// TODO: pipeline this into multi-push.
	pushValue(BooleanValue(true));
	pushValue(exception);
	// Bye.
	bctr();

	bind(&return_);
	// Only used in debug mode: return BaselineFrame->returnValue() to
	// the caller. Get the old BaselineFrameReg,
	lwz(BaselineFrameReg, stackPointerRegister,
		offsetof(ResumeFromException, framePointer));
	// and get the old stack pointer.
	lwz(stackPointerRegister, stackPointerRegister,
		offsetof(ResumeFromException, stackPointer));
	// Load our return value.
	loadValue(Address(BaselineFrameReg,
		BaselineFrame::reverseOffsetOfReturnValue()), JSReturnOperand);
	// Remove the stack frame.
	x_mr(stackPointerRegister, BaselineFrameReg);
	pop(BaselineFrameReg);
	// Bail out.
	ret();
}

// Fixed endian versions (see IonMacroAssembler.h).
// Since we're big endian, and therefore we rock, no bitshift is required.

void
MacroAssemblerPPC::branchIfFunctionHasNoScript(Register fun, Label *label) {
	JS_STATIC_ASSERT(offsetof(JSFunction, nargs) % sizeof(uint32_t) == 0);
	JS_STATIC_ASSERT(offsetof(JSFunction, flags) ==
		offsetof(JSFunction, nargs) + 2);

	Address address(fun, offsetof(JSFunction, nargs));
	// In memory, the nargs-flags word looks like NNNNFFFF, with FFFF
	// being the LSB. We want to test the LSB for 0x0001.
	branchTest32(Assembler::Zero, address, Imm32(JSFunction::INTERPRETED),
		label);
}

void
MacroAssemblerPPC::branchIfInterpreted(Register fun, Label *label) {
	JS_STATIC_ASSERT(offsetof(JSFunction, nargs) % sizeof(uint32_t) == 0);
	JS_STATIC_ASSERT(offsetof(JSFunction, flags) ==
		offsetof(JSFunction, nargs) + 2);

	Address address(fun, offsetof(JSFunction, nargs));
	// Ditto.
	branchTest32(Assembler::NonZero, address,
		Imm32(JSFunction::INTERPRETED), label);
}

} // jit
} // js
