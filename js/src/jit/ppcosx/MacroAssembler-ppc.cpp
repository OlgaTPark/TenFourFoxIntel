#include "jit/BaselineFrame.h"
#include "jit/Bailouts.h"
#include "jit/ppcosx/MacroAssembler-ppc.h"
#include "jit/ppcosx/MoveEmitter-ppc.h"
#include "jit/ppcosx/BaselineRegisters-ppc.h"

#if _PPC970_
#define MFCR0(x) mfocrf(x,cr0)
#else
#define MFCR0(x) mfcr(x)
#endif

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
		JitFrame_IonJS);
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
		JitFrame_IonJS);
	// TODO: We could optimize this a bit better with multi-push.
	Push(Imm32(descriptor));
	Push(ImmWord(fakeReturnAddr));
	return true;
}

void
MacroAssemblerPPC::callWithExitFrame(JitCode *target) {
	ispew("callWithExitFrame(ion *)");

	uint32_t descriptor = MakeFrameDescriptor(framePushed(),
		JitFrame_IonJS);
	Push(Imm32(descriptor));
	call(target); // Use the JitCode version, not the PPCAssembler.
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

	// Since JitCode is not ABI compliant, we can just call it.
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
MacroAssemblerPPC::passABIArg(const MoveOperand &from, MoveOp::Type type) {
#if DEBUG
	IonSpew(IonSpew_Codegen, "state, passABIArg: gprs %i fprs %i\n",
		passedGPRs_, passedFPRs_);
#endif

	if (!enoughMemory_) return;
	if (type == MoveOp::FLOAT32 || type == MoveOp::DOUBLE) {
		FloatRegister fpr;

		// PPC OSX ABI: f1-f13 are parameter registers
		if (++passedFPRs_ > 13) {
			// If we have more than 13 FPR argregs, OMFG.
			MOZ_ASSUME_UNREACHABLE("FPR ABI argument overflow");
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
				MoveOperand(fpr), type);
	} else {
		JS_ASSERT(type == MoveOp::GENERAL);
		Register gpr;

		// PPC OSX ABI: r3-r10 are parameter registers
		if (++passedGPRs_ > 8) {
			// This might be a little more frequent ...
			MOZ_ASSUME_UNREACHABLE("GPR ABI argument overflow");
		} else {
			gpr = Register::FromCode(
				(Register::Code)2+passedGPRs_);
		}
		if (!from.isGeneralReg() // i.e., we need to convert
				|| from.reg() != gpr)
			enoughMemory_ = moveResolver_.addMove(from,
				MoveOperand(gpr), type);
	}
}
void
MacroAssemblerPPC::passABIArg(const Register &gpr, MoveOp::Type type =
		MoveOp::GENERAL) {
	passABIArg(MoveOperand(gpr), type);
}
void
MacroAssemblerPPC::passABIArg(const FloatRegister &fpr, MoveOp::Type type =
		MoveOp::DOUBLE) {
	passABIArg(MoveOperand(fpr), type);
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
MacroAssemblerPPC::callWithABIPost(uint32_t stackAdjust, MoveOp::Type result) {
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
MacroAssemblerPPC::callWithABI(void *fun,
		MoveOp::Type result = MoveOp::GENERAL) {
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
MacroAssemblerPPC::callWithABI(const Address &fun,
		MoveOp::Type result = MoveOp::GENERAL) {
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

	uint8_t *dest = ((uint8_t*)GetIonContext()->runtime) +
		offsetof(JSRuntime, mainThread.ionTop);
	// Use r12 as the effective address.
	x_li32(addressTempRegister, (uint32_t)dest);
	stw(stackPointerRegister, addressTempRegister, 0);
}

void
MacroAssemblerPPC::callWithExitFrame(JitCode *target, Register dynStack) {
	ispew("callWithExitFrame");

	addPtr(Imm32(framePushed()), dynStack);
	makeFrameDescriptor(dynStack, JitFrame_IonJS);
	Push(dynStack);
	call(target);
}

void
MacroAssemblerPPC::linkParallelExitFrame(const Register &reg) {
	ispew("linkParallelExitFrame");
	// Given the provided register, store the stack pointer there using
	// it as an offset to PerThreadData.
	stw(stackPointerRegister, reg, offsetof(PerThreadData, ionTop));
}

void
MacroAssemblerPPC::handleFailureWithHandlerTail(void *handler) {
	ispew("handleFailureWithHandler");
	// Call an ABI-compliant handler function, reserving adequate space
	// on the stack for a non-ABI compliant exception frame which we will
	// then analyse.

	// ResumeFromException is defined in IonFrames.h
	int size = (sizeof(ResumeFromException) + 16) & ~15;
	x_subi(stackPointerRegister, stackPointerRegister, size);
	x_mr(r3, stackPointerRegister);

	setupUnalignedABICall(1, r4);
	passABIArg(r3);
	callWithABI(handler);
	
	Label entryFrame;
	Label catch_;
	Label finally;
	Label return_;
	Label bailout;

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
	branch32(Assembler::Equal, r3,
		Imm32(ResumeFromException::RESUME_BAILOUT), &bailout);
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
	ret();
	
	bind(&bailout);
	// If we are bailing out to Baseline to handle an exception, call
	// the tail stub.
	lwz(r4, stackPointerRegister, offsetof(ResumeFromException, target));
	x_mtctr(r4); // new dispatch group
	x_li32(r3, BAILOUT_RETURN_OK);
	lwz(r5, stackPointerRegister, offsetof(ResumeFromException, bailoutInfo));
#if defined(_PPC970_)
	x_nop(); // x_li32 is probably only one instruction
#endif
	// Bye.
	bctr();
}

// Fixed endian versions (see IonMacroAssembler.h).
// Since we're big endian, and therefore we rock, no bitshift is required.

void
MacroAssemblerPPC::branchIfFunctionHasNoScript(Register fun, Label *label) {
	JS_ASSERT(JSFunction::offsetOfNargs() % sizeof(uint32_t) == 0);
	JS_ASSERT(JSFunction::offsetOfFlags() ==
		JSFunction::offsetOfNargs() + 2);

	Address address(fun, JSFunction::offsetOfNargs());
	// In memory, the nargs-flags word looks like NNNNFFFF, with FFFF
	// being the LSB. We want to test the LSB for 0x0001.
	branchTest32(Assembler::Zero, address, Imm32(JSFunction::INTERPRETED),
		label);
}

void
MacroAssemblerPPC::branchIfInterpreted(Register fun, Label *label) {
	JS_ASSERT(JSFunction::offsetOfNargs() % sizeof(uint32_t) == 0);
	JS_ASSERT(JSFunction::offsetOfFlags() ==
		JSFunction::offsetOfNargs() + 2);

	Address address(fun, JSFunction::offsetOfNargs());
	// Ditto.
	branchTest32(Assembler::NonZero, address,
		Imm32(JSFunction::INTERPRETED), label);
}

// Things that go double in the night, and sometimes float.

void
MacroAssemblerPPC::loadConstantDouble(double d, const FloatRegister &dest) {
    ispew("loadConstantDouble(double, fpr)");

#if(1)
    // XXX: Preferably we'd lfd right out of memory, but this is safer just
    // in case the DoubleMap can move in memory after the code is generated.

    uint32_t *broken_double = (uint32_t *)&d;
    x_li32(tempRegister, broken_double[0]);
    x_li32(addressTempRegister, broken_double[1]);
    // Prefer stwu here, even on G5, so that it cracks and soaks up two slots.
    stwu(addressTempRegister, stackPointerRegister, -4);
    stwu(tempRegister, stackPointerRegister, -4);
#if defined(_PPC970_)
    // A little more paranoia.
    x_nop();
    x_nop();
#endif
    lfd(dest, stackPointerRegister, 0);
    addi(stackPointerRegister, stackPointerRegister, 8);

#else
    if (!doubleMap_.initialized()) {
        enoughMemory_ &= doubleMap_.init();
        if (!enoughMemory_)
            return; 
    }
    size_t doubleIndex;
    DoubleMap::AddPtr p = doubleMap_.lookupForAdd(d);
    if (p) { 
        doubleIndex = p->value;
    } else {
        doubleIndex = doubles_.length();
        enoughMemory_ &= doubles_.append(Double(d));
        enoughMemory_ &= doubleMap_.add(p, d, doubleIndex);
        if (!enoughMemory_)
            return;
    }
    Double &dbl = doubles_[doubleIndex];
    JS_ASSERT(!dbl.uses.bound());

    // This seems wrong, even though it appears to work. x86 uses
    // dbl.uses.prev() but that puts in a -1. Are we missing a patch
    // somewhere?
    x_li32(addressTempRegister, (uint32_t)&(dbl.value));
    lfd(dest, addressTempRegister, 0);
    x_trap();
    dbl.uses.setPrev(masm.size());
#endif
}

void
MacroAssemblerPPC::loadConstantFloat32(float f, const FloatRegister &dest)
{
MOZ_ASSUME_UNREACHABLE("fix per loadConstantDouble");
    if (!floatMap_.initialized()) {
        enoughMemory_ &= floatMap_.init();
        if (!enoughMemory_)
            return;
    }
    size_t floatIndex;
    FloatMap::AddPtr p = floatMap_.lookupForAdd(f);
    if (p) {
        floatIndex = p->value();
    } else {
        floatIndex = floats_.length();
        enoughMemory_ &= floats_.append(Float(f));
        enoughMemory_ &= floatMap_.add(p, f, floatIndex);
        if (!enoughMemory_)
            return;
    }
    Float &flt = floats_[floatIndex];
    JS_ASSERT(!flt.uses.bound());

    x_li32(addressTempRegister, (uint32_t)flt.uses.prev());
    lfd(dest, addressTempRegister, 0);
    flt.uses.setPrev(masm.size());
}

void
MacroAssembler::PushRegsInMask(RegisterSet set)
{
	// TODO: We don't need two reserveStacks.
	int32_t diffF = set.fpus().size() * sizeof(double);
	int32_t diffG = set.gprs().size() * sizeof(intptr_t);
	reserveStack(diffG);
	for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); iter++) {
	    diffG -= sizeof(intptr_t);
	    storePtr(*iter, Address(StackPointer, diffG));
	}
	JS_ASSERT(diffG == 0);
	reserveStack(diffF);
	for (FloatRegisterBackwardIterator iter(set.fpus()); iter.more(); iter++) {
	    diffF -= sizeof(double);
	    storeDouble(*iter, Address(StackPointer, diffF));
	}
	JS_ASSERT(diffF == 0);
}

void
MacroAssembler::PopRegsInMaskIgnore(RegisterSet set, RegisterSet ignore)
{
	int32_t diffG = set.gprs().size() * sizeof(intptr_t);
	int32_t diffF = set.fpus().size() * sizeof(double);
	const int32_t reservedG = diffG;
	const int32_t reservedF = diffF;

	// TODO: We don't need two freeStacks.
	{
	    for (FloatRegisterBackwardIterator iter(set.fpus()); iter.more(); iter++) {
	        diffF -= sizeof(double);
	        if (!ignore.has(*iter))
	            loadDouble(Address(StackPointer, diffF), *iter);
	    }
	    freeStack(reservedF);
	}
	JS_ASSERT(diffF == 0);
	{
	    for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); iter++) {
	        diffG -= sizeof(intptr_t);
	        if (!ignore.has(*iter))
	            loadPtr(Address(StackPointer, diffG), *iter);
	    }
	    freeStack(reservedG);
	}
	JS_ASSERT(diffG == 0);
}

// XXX: This can be MAJORLY improved.
// Note: this function clobbers the input register.
void
MacroAssembler::clampDoubleToUint8(FloatRegister input, Register output)
{
	JS_ASSERT(input != ScratchFloatReg);
	Label positive, done;

	// <= 0 or NaN --> 0
	zeroDouble(ScratchFloatReg);
	branchDouble(DoubleGreaterThan, input, ScratchFloatReg, &positive);
	{
	    move32(Imm32(0), output);
	    jump(&done);
	}
	bind(&positive);

	// Add 0.5 and truncate.
	loadConstantDouble(0.5, ScratchFloatReg);
	addDouble(ScratchFloatReg, input);
	Label outOfRange;
	// Truncate to int32 and ensure the result <= 255. This relies on the
	// processor setting output to a value > 255 for doubles outside the
	// int32 range (for instance 0x80000000). Our simulated cvttsd2si()
	// accomplishes that.
	cvttsd2si(input, output);
	branch32(Assembler::Above, output, Imm32(255), &outOfRange);
	{
	    // Check if we had a tie.
	    convertInt32ToDouble(output, ScratchFloatReg);
	    branchDouble(DoubleNotEqual, input, ScratchFloatReg, &done);
	    // It was a tie. Mask out the ones bit to get an even value.
	    // See also js_TypedArray_uint8_clamp_double.
	    and32(Imm32(~1), output);
	    jump(&done);
	}
	// > 255 --> 255
	bind(&outOfRange);
	{
	    move32(Imm32(255), output);
	}
	bind(&done);
}

#ifdef JSGC_GENERATIONAL

void
MacroAssemblerPPC::branchPtrInNurseryRange(Register ptr, Register temp,
	Label *label)
{
	ispew("branchPtrInNurseryRange(reg, reg, l)");
	const Nursery &nursery = GetIonContext()->runtime->gcNursery();

	// NurserySize is usually 16MB, so load into addressTemp since we
	// won't be able to put it into an immediate. Similarly, do the
	// same for nursery.start() since this is likely a memory location.
	// This is based off x86, since it's simpler.

	x_li32(addressTempRegister, nursery.start());
	subf(tempRegister, addressTempRegister, ptr); // T=B-A
	x_li32(addressTempRegister, Nursery::NurserySize);
	cmplw(tempRegister, addressTempRegister); // UNSIGNED!
	bc(Assembler::Below, label);
}

void
MacroAssemblerPPC::branchValueIsNurseryObject(ValueOperand value,
	Register temp, Label *label)
{
	Label done;
	ispew("branchValueIsNurseryObject(vo, reg, l)");

	branchTestObject(Assembler::NotEqual, value, &done);
	branchPtrInNurseryRange(value.payloadReg(), temp, label);

	bind(&done);
}

#endif

void
MacroAssemblerPPC::emitSet(Assembler::Condition cond, const Register &dest)
{
        // Extract the relevant bits from CR0. This is based on the FP
        // code, but different from ICCompare_Int32.
        // Fast paths.
        if (cond == Assembler::Equal) {
            MFCR0(r0);
            rlwinm(dest, r0, 3, 31, 31); // get CR0[EQ]
        } else if (cond == Assembler::NotEqual) {
            // Same, but inverted with the xori (flip the bit).
            MFCR0(r0);
            rlwinm(r0, r0, 3, 31, 31); // get CR0[EQ]
            xori(dest, r0, 1); // flip sign into the payload reg
        } else if (cond == Assembler::GreaterThan) {
            MFCR0(r0);
            rlwinm(dest, r0, 2, 31, 31); // get CR0[GT]
        } else if (cond == Assembler::LessThanOrEqual) {
            // Inverse (not reverse).
            MFCR0(r0);
            rlwinm(r0, r0, 2, 31, 31); // get CR0[GT]
            xori(dest, r0, 1); // flip sign into the payload reg
        } else if (cond == Assembler::LessThan) {
            MFCR0(r0);
            rlwinm(dest, r0, 1, 31, 31); // get CR0[LT]
        } else if (cond == Assembler::GreaterThanOrEqual) {
            MFCR0(r0);
            rlwinm(r0, r0, 1, 31, 31); // get CR0[LT]
            xori(dest, r0, 1); // flip sign into the payload reg
        } else {
            // Use the emitSet branched version to cover other things.
            emitSetSlow(cond, dest);
        }
}

void
MacroAssemblerPPC::emitSet(Assembler::DoubleCondition cond, const Register &dest)
{
    bool isUnordered;

    // Ripped off from our BaselineIC; we'll probably merge this code.
    // Check for simple ordered/unordered before checking synthetic codes.
    if (cond == Assembler::DoubleUnordered) {
        MFCR0(r0);
        rlwinm(dest, r0, 4, 31, 31); // get CR0[FU]. FU! FUUUUUUUUUU-
    } else if (cond == Assembler::DoubleOrdered) {
        // Same, but with the xori (flip the bit).
        MFCR0(r0);
        rlwinm(r0, r0, 4, 31, 31);
        xori(dest, r0, 1); // flip sign into the payload reg
    } else {
        // This is a synthetic condition code.
        // Extract it into the condition and whether it's "OrUnordered."
        const uint8_t fuBit = crBit(cr0, Assembler::DoubleUnordered);
        const uint8_t condBit = crBit(cr0, cond);
        isUnordered = (cond & Assembler::DoubleUnordered) ? true : false;
        Assembler::DoubleCondition baseDCond = (isUnordered) ?
            (cond & ~Assembler::DoubleUnordered) : cond;

        // Fast paths.
        if (baseDCond == Assembler::DoubleEqual) {
            if (isUnordered) cror(condBit, fuBit, condBit);
            MFCR0(r0);
            rlwinm(dest, r0, 3, 31, 31); // get CR0[FE]
        } else if (baseDCond == Assembler::DoubleNotEqual) {
            // Same, but inverted with the xori (flip the bit).
            if (isUnordered)
                // Flip FU and AND it with condBit.
                crandc(condBit, condBit, fuBit);
            MFCR0(r0);
            rlwinm(r0, r0, 3, 31, 31); // get CR0[FE]
            xori(dest, r0, 1); // flip sign into the payload reg
        } else if (baseDCond == Assembler::DoubleGreaterThan) {
            if (isUnordered) cror(condBit, fuBit, condBit);
            MFCR0(r0);
            rlwinm(dest, r0, 2, 31, 31); // get CR0[FG]
        } else if (baseDCond == Assembler::DoubleLessThanOrEqual) {
            // Inverse (not reverse).
            if (isUnordered) crandc(condBit, condBit, fuBit);
            MFCR0(r0);
            rlwinm(r0, r0, 2, 31, 31); // get CR0[FG]
            xori(dest, r0, 1); // flip sign into the payload reg
        } else if (baseDCond == Assembler::DoubleLessThan) {
            if (isUnordered) cror(condBit, fuBit, condBit);
            MFCR0(r0);
            rlwinm(dest, r0, 1, 31, 31); // get CR0[FL]
        } else if (baseDCond == Assembler::DoubleGreaterThanOrEqual) {
            // Inverse (not reverse).
            if (isUnordered) crandc(condBit, condBit, fuBit);
            MFCR0(r0);
            rlwinm(r0, r0, 1, 31, 31); // get CR0[FL]
            xori(dest, r0, 1); // flip sign into the payload reg
        } else {
            // Use the emitSet branched version to cover other things.
            emitSetSlow(cond, dest);
        }
    }
}

void
MacroAssemblerPPC::cmp32Set(Assembler::Condition cond, Register lhs, Register rhs,
            const Register &dest) {
    // Fast paths, PowerPC Compiler Writer's Guide, Appendix D et al. These
    // avoid use of CR, which could be slow (on G5, mfcr is microcoded).
    // Due to possibly constrained register usage, we don't use the optimals.
    // TODO: Add subfe and subfze to support unsigned Above/Below, though
    // these are probably used a lot less.
    if (cond == Assembler::Equal) {
        subf(r0, rhs, lhs); // p.141
        cntlzw(r0, r0);
        x_srwi(dest, r0, 5);
    } else if (cond == Assembler::NotEqual) {
        subf(r0, lhs, rhs); // p.58
        subf(r12, rhs, lhs);
        or_(r12, r12, r0);
        rlwinm(dest, r12, 1, 31, 31);
    } else if (cond == Assembler::LessThan) { // SIGNED
        subfc(r0, rhs, lhs); // p.200
        eqv(r12,  rhs, lhs);
        x_srwi(r0, r12, 31);
        addze(r12, r0);
        rlwinm(dest, r12, 0, 31, 31);
    } else if (cond == Assembler::GreaterThan) { // SIGNED
        // Reverse (not inverse).
        subfc(r0, lhs, rhs);
        eqv(r12,  lhs, rhs);
        x_srwi(r0, r12, 31);
        addze(r12, r0);
        rlwinm(dest, r12, 0, 31, 31);
    } else if (cond == Assembler::LessThanOrEqual) { // SIGNED
        x_srwi(r0, lhs, 31); // p.200
        srawi(r12, rhs, 31);
        subfc(dest, lhs, rhs); // We can clobber dest here.
        adde(dest, r12, r0);
    } else if (cond == Assembler::GreaterThanOrEqual) { // SIGNED
        // Reverse (not inverse).
        x_srwi(r0, rhs, 31);
        srawi(r12, lhs, 31);
        subfc(dest, rhs, lhs); // We can clobber dest here.
        adde(dest, r12, r0);
    } else {
        // Use the emitSet branched version as a slow path for any condition.
	if (PPC_USE_UNSIGNED_COMPARE(cond)) {
        	cmplw(lhs, rhs);
	} else {
        	cmpw(lhs, rhs);
	}
        emitSet(cond, dest);
    }
}

void
MacroAssemblerPPC::cmpPtrSet(Assembler::Condition cond, const Register &lhs,
            const Register &rhs, const Register &dest) {
    if (cond == Assembler::Equal) {
        subf(r0, rhs, lhs); // p.141
        cntlzw(r0, r0);
        x_srwi(dest, r0, 5);
    } else if (cond == Assembler::NotEqual) {
        subf(r0, lhs, rhs); // p.58
        subf(r12, rhs, lhs);
        or_(r12, r12, r0);
        rlwinm(dest, r12, 1, 31, 31);
    } else {
	// This is an unsigned comparison, so use cmplw and the
	// long form emitSet.
	cmplw(lhs, rhs);
	emitSet(cond, dest);
    }
}

void
MacroAssemblerPPC::branchTestMagicValue(Condition cond, const ValueOperand &val,
            JSWhyMagic why, Label *label)
{
	ispew("branchTestMagicValue(cond, vo, jswhy, l)");
        // Oh, oh, oh, it's MAGIC! Or NO! Then we will branch if it's SO!
        JS_ASSERT(cond == Equal || cond == NotEqual);

	x_li32(r0, JSVAL_TAG_MAGIC);
	x_li32(r12, static_cast<uint32_t>(why));
	xor_(r0, val.typeReg(), r0);
	xor_(r12, val.payloadReg(), r12);
	or_rc(r0, r0, r12);
	bc((cond == Equal) ? Assembler::Zero : Assembler::NonZero, label);
}

void
MacroAssemblerPPC::pushValue(const Address &addr)
{
#if(1)
// For reasons I don't fully understand, the FPU version is slower.
// It shouldn't be, so I'm keeping it around so I can think about why.
        ispew("[[ pushValue(adr)");

        // pushValue does not track framePushed_.
        JS_ASSERT(addr.base != tempRegister);
        JS_ASSERT(addr.base != addressTempRegister);

        if (addr.base == stackPointerRegister) {
            // Eeek. Since r1 is going to be hopelessly serialized, just
            // emit a very unoptimized stream instead.
            if (PPC_IMMOFFS_OK(addr.offset+4)) {
                // Push payload first -- we're big endian!
                lwz(tempRegister, addr.base, 4+addr.offset);
                stwu(tempRegister, stackPointerRegister, -4);
                // The next load is from +4 because we just -4'ed r1.
                lwz(tempRegister, addr.base, 4+addr.offset);
                stwu(tempRegister, stackPointerRegister, -4);
            } else {
                // Get the effective address into a register, which could
                // still be r1.
                Register basebase = computeEAToRegister(addr);
                lwz(tempRegister, basebase, 4);
                stwu(tempRegister, stackPointerRegister, -4);
                // If basebase is still r1, then we need to +4 the offset
                // again for the same reason.
                if (basebase == stackPointerRegister)
                    lwz(tempRegister, addr.base, 4);
                else
                    lwz(tempRegister, addr.base, 0);
                stwu(tempRegister, stackPointerRegister, -4);
            }
            return;
        }

        // r1 is not the address base; do a double-barreled stack roll!
        addi(stackPointerRegister, stackPointerRegister, -8);

        if (PPC_IMMOFFS_OK(addr.offset+4)) {
            // Bake the offset into the loads using the address base. This
            // saves calculating EA at runtime.
            // Push payload on first -- we're big endian!
            lwz(tempRegister, addr.base, 4+addr.offset);
            lwz(addressTempRegister, addr.base, addr.offset);
            stw(tempRegister, stackPointerRegister, 4);
            stw(addressTempRegister, stackPointerRegister, 0);
        } else {
            // Get the effective address into a register. basebase can be
            // r12, which is fine, because we don't need it after r12 is
            // loaded. 
            Register basebase = computeEAToRegister(addr);

            lwz(tempRegister, basebase, 4);
            lwz(addressTempRegister, basebase, 0);
            stw(tempRegister, stackPointerRegister, 4);
            stw(addressTempRegister, stackPointerRegister, 0);
        }

        ispew("   pushValue(adr) ]]");
#else
	ispew("pushValue(adr)");

	// Use the FPU, Luke!
        if (PPC_IMMOFFS_OK(addr.offset)) {
            // Push payload on first -- we're big endian!
            lfd(fpTempRegister, addr.base, addr.offset);
#if _PPC970_
	   // If addr.base is the stackPointer and we're within 16 bytes of
	   // the top of the stack, this will alias. Separate dispatch
	   // groups in that case.
	   // XXX: I suppose it's possible that the base could be a copy ...
	   if (addr.base == stackPointerRegister && addr.offset < 16) {
		x_nop();
		x_nop();
		x_nop();
	   }
#endif
            stfdu(fpTempRegister, stackPointerRegister, -8);
        } else {
            // Get the effective address into a register.
            Register basebase = computeEAToRegister(addr);
	    lfd(fpTempRegister, basebase, 0);
	    // We're not at risk of G5 memory aliasing since the offset is
	    // obviously greater than 16 bytes.
            stfdu(fpTempRegister, stackPointerRegister, -8);
        }
#endif
}

} // jit
} // js
