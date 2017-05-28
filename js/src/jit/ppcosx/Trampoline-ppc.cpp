/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/IonSpewer.h"
#include "jit/ppcosx/BaselineRegisters-ppc.h"
#include "jit/ppcosx/BaselineHelpers-ppc.h"
#include "jscompartment.h"
#include "assembler/assembler/MacroAssembler.h"
#include "jit/IonCompartment.h"
#include "jit/IonLinker.h"
#include "jit/IonFrames.h"
#include "jit/Bailouts.h"
#include "jit/VMFunctions.h"

using namespace js;
using namespace js::jit;

// This is our *original* stack frame.
struct EnterJITStack
{
    void *savedSP;      // 0(r1)
    void *savedCR;
    void *savedLR;
    void *reserved0;
    void *reserved1;
    void *reserved2;    
    // Don't forget the argument area! (TenFourFox issue 179)
    void *arg0;         // 24(r1)
    void *arg1;
    void *arg2;
    void *arg3;
    void *arg4;
    void *arg5;
    void *arg6;
    void *arg7;         
    // Make the stack quadword aligned.
    void *padding0;     // 56(r1)

    // Non-volatile egisters being saved for various purposes.
    void *savedR13;     // AsmJS and BaselineFrameReg
    void *savedR14;     // Baseline Compiler
    void *savedR15;     // Baseline Compiler
    void *savedR16;     // temporary stack for ABI calls
    void *savedR17;     // temporary stack for Trampoline
    void *savedR18;     // temporary LR for ABI calls
    // GPRs. We only save r19 through r25 inclusive to save some stack space.
    void *savedR19;     // 84(r1)
    void *savedR20;
    void *savedR21;
    void *savedR22;
    void *savedR23;
    void *savedR24;
    void *savedR25;
    // 112(r1)

    // We don't need to save any FPRs; we don't let the allocator use
    // any of the non-volatile ones.
};

// When running purely in Baseline Compiler mode, we could consider
// a smaller stack frame where only the Baseline Compiler and temporary
// ABI call registers need be saved; this reduces our stack frame to 80.
// #define PPC_USE_SKINNY_STACK
struct EnterJITStackSkinny
{
    void *savedSP;      // 0(r1)
    void *savedCR;
    void *savedLR;
    void *reserved0;
    void *reserved1;
    void *reserved2;    
    // Don't forget the argument area! (TenFourFox issue 179)
    void *arg0;         // 24(r1)
    void *arg1;
    void *arg2;
    void *arg3;
    void *arg4;
    void *arg5;
    void *arg6;
    void *arg7;         

    // Non-volatile egisters being saved for various purposes.
    void *savedR13;     // AsmJS and BaselineFrameReg
    void *savedR14;     // Baseline Compiler
    void *savedR15;     // Baseline Compiler
    void *savedR16;     // temporary stack for ABI calls
    void *savedR17;     // temporary stack for Trampoline
    void *savedR18;     // temporary LR for ABI calls
    // 80(r1)

    // We don't need to save any FPRs; we don't let the allocator use
    // any of the non-volatile ones.
};

// TODO. We might not even need the rest of the frame, since we're not
// necessarily making any ABI compliant calls. We could strip it down to
// SP/CR/LR and the saved GPRs.

// Assert that the chosen stack frame type is already ABI compliant.
#ifdef PPC_USE_SKINNY_STACK
#define PPC_FRAMETYPE EnterJITStackSkinny
#else
#define PPC_FRAMETYPE EnterJITStack
#endif
// Utility code to give the compiler view of the size of the struct.
#if(0)
template<int s> struct Wow;
struct foo {
    int a,b;
};
Wow<sizeof(PPC_FRAMETYPE)> wow;
#endif
JS_STATIC_ASSERT(sizeof(PPC_FRAMETYPE) % 16 == 0);

// Define this to do a self-test of the trampoline. It always returns zero.
// Use something like js --baseline-eager --no-ion --no-ti -e 'var i=0'
// #define SELF_TEST_TRAMPOLINE

/*
 * This method generates a trampoline for a C++ function with
 * the following signature using standard ABI calling convention (from
 * IonCompartment.h):

typedef JSBool(*EnterIonCode) (void *code, int argc, Value *argv,
                               StackFrame *fp,
                               CalleeToken calleeToken, JSObject *scopeChain,
                               size_t numStackValues, Value *vp);

 *
 */
IonCode *
IonRuntime::generateEnterJIT(JSContext *cx, EnterJitType type)
{
    // PPC calling convention
    const Register reg_code  = r3;
    const Register reg_argc  = r4;
    const Register reg_argv  = r5;
    const Register reg_frame = r6;
    const Register reg_token = r7;
    const Register reg_scope = r8;
    const Register reg_nsv   = r9;
    const Register reg_vp    = r10; // Whew! Just fits!

    JS_ASSERT(OsrFrameReg == reg_frame);

    MacroAssembler masm(cx);
    AutoFlushCache afc("GenerateEnterJIT", cx->runtime()->ionRuntime());
    Register sp = stackPointerRegister;
    Label epilogue;

    // 1. Save non-volatile registers. These must be saved by the trampoline,
    // rather than the JIT'd code, because they are scanned by the conservative
    // scanner. This is essentially a standard PowerPC function prologue
    // except for the stack management, since Ion is not ABI compliant.

    // Keep a copy of sp; we need it to emit bogus stack frames for calls
    // requiring ABI compliance (it will live in r18 in just a moment).
    masm.x_mr(r12, sp);

    // Save LR and CR to the caller's linkage area.
    masm.x_mflr(r0);
    masm.stw(r0, sp, 8);
    masm.mfcr(r0);
    masm.stw(r0, sp, 4);

    // Now create the stack frame, in a fully ABI compliant manner so that
    // VMFunctions we call can walk the stack back.
    masm.stwu(sp, sp, -(sizeof(PPC_FRAMETYPE)));

    // Dump the GPRs in the new frame. Emit an unrolled loop.
    // We don't need to save any FPRs.
    uint32_t j = 60;
    for (uint32_t i = 13; i < 26; i++) {
        masm.stw(Register::FromCode(i), sp, j);
        j+=4;
    }
    JS_ASSERT(j == sizeof(PPC_FRAMETYPE));

    // Save sp to r18 so that we can restore it for ABI calls.
    masm.x_mr(r18, r12);

#ifdef SELF_TEST_TRAMPOLINE
    masm.x_li32(r3, 0);
    masm.tagValue(JSVAL_TYPE_INT32, r3, JSReturnOperand);
    masm.storeValue(JSReturnOperand, Address(reg_vp, 0));
    masm.b(&epilogue);
#endif

    // Registers saved. Prepare our internals.
    if (type == EnterJitBaseline) {
        // Set the baseline to the current stack pointer value so that
        // pushes "make sense."
        masm.x_mr(BaselineFrameReg, sp);
    }

    // 2. Copy arguments from JS's buffer onto the stack so that JIT code
    // can access them. WARNING: we are no longer ABI compliant now.
    {
        Label top, bottom;
        masm.x_slwi(r17, reg_argc, 3); /* times 8 */
        // Do this here so that r17 is already "zero" if argc is zero. See
        // below.

        // If argc is already zero, skip.
        masm.cmplwi(reg_argc, 0);
        masm.x_beq(cr0, 
#if defined(_PPC970_)
            9*4, // count the nops
#else
            7*4,
#endif
            Assembler::NotLikelyB, Assembler::DontLinkB); // forward branch not
            // likely to be taken (thus don't set Lk bit)

        // First, move argc into CTR.
        masm.x_mtctr(reg_argc);

        // Use r17 as a work register. We can't use r0, because addi
        // will wreck the code. We'll use it again in step 3, so we just
        // clobber reg_argc. (If argc was already zero, then r17 was already
        // cleared above.)
        masm.x_mr(reg_argc, r17);

#if defined(_PPC970_)
        // We know mtctr must lead a dispatch group, so this forces everything
        // below into one dispatch group (addi-lfdx-stfdu-bdnz).
        masm.x_nop();
        masm.x_nop();
#endif
        masm.bind(&top);

        // Now, copy from the argv pointer onto the stack with a tight bdnz
        // loop. The arguments are 64-bit, so we use the FPU. It should all
        // fit within one dispatch group on G5. This will push the arguments
        // on in reverse order. Because the 32-bit pushes put the second
        // 32-bit word on the stack first, using the FPU will be equivalent.
        masm.x_subi(reg_argc, reg_argc, 8); // We start out at +1, so sub first.
        masm.lfdx(f0, reg_argv, reg_argc); // Load from argv+argc // aligned?!
        // G5 shouldn't need nops here, since the address isn't aliased.
        masm.stfdu(f0, sp, -8); // Store to 0(sp) // cracked on G5
        masm.x_bdnz(-3*4, Assembler::NotLikelyB,
            Assembler::DontLinkB); // reverse branch, likely
            // to be taken (thus don't set Lk bit)
        
        masm.bind(&bottom);
    }

    // 3. Now that the arguments are on the stack, create the Ion stack
    // frame. The OsrFrameReg is already reg_frame; we asserted such.

    // Push number of actual arguments.
    // We don't need reg_argv anymore, so unbox to that.
    masm.unboxInt32(Address(reg_vp, 0), reg_argv);
    masm.push(reg_argv);
    // Push the callee token.
    masm.push(reg_token);
    // Create an Ion entry frame with r17, which still has the number of
    // bytes actually pushed. We don't bother with realigning the stack until
    // we actually have to call an ABI-compliant routine. Add two more words
    // for argv and token.
    masm.addi(r17, r17, 8);
    masm.makeFrameDescriptor(r17, IonFrame_Entry); // r17 is clobbered
    masm.push(r17); // frame descriptor with encoded argc

    // Save the value pointer in r17, which is now free and is non-volatile.
    // We will need it in the epilogue.
    masm.x_mr(r17, reg_vp);

    Label returnLabel;

    if (type == EnterJitBaseline) {
        // Handle OSR.
        ispew("--==-- OSR CASE --==--");
        Label notOsr;

        // ... but only if we have to.
        masm.branchTestPtr(Assembler::Zero, OsrFrameReg, OsrFrameReg, &notOsr);

        // First, write the return address; extract it from LR. We can
        // clobber LR because we don't need it, and we get our own return
        // address from the stack frame. This needs some footwork to ensure
        // we return to the right location, which is returnLabel below.
        // r4 is now available since we don't need argc anymore.
        masm.ensureSpace(PPC_BRANCH_STANZA_LENGTH + 16);
        masm.x_b(4, Assembler::LinkB); // bl *+4
        masm.x_mflr(r4); // (0)
        masm.x_b(PPC_BRANCH_STANZA_LENGTH + 4, Assembler::DontLinkB);
            // skip exit stanza // (4)
        masm.b(&returnLabel); // (8) // The actual return address is here.

        // Push the return address and Baseline frame pointer.
        masm.addi(r4, r4, 8);
        masm.push2(r4, BaselineFrameReg);

        // Reserve a new frame and update the Baseline frame pointer.
        JS_ASSERT(BaselineFrame::Size() < 65536);
        masm.x_subi(sp, sp, BaselineFrame::Size());
        masm.x_mr(BaselineFrameReg, sp);

        // Reserve space for locals and stack values. These are doubles,
        // so multiply reg_nsv by 8. Use r12 since we need some immediates
        // and r0 won't work.
        masm.x_slwi(r12, reg_nsv, 3);
        masm.subf(sp, r12, sp);

        // Enter the exit frame. 
        JS_ASSERT((BaselineFrame::Size() + BaselineFrame::FramePointerOffset)
            < 65536);
        masm.addi(r12, r12,
            (BaselineFrame::Size() + BaselineFrame::FramePointerOffset));
        masm.makeFrameDescriptor(r12, IonFrame_BaselineJS);
        masm.x_li(r0, 0); // Fake return address
        masm.push2(r12, r0);
        masm.enterFakeExitFrame();

        masm.push2(BaselineFrameReg, reg_code);

        /* Call the following:
           bool InitBaselineFrameForOsr(BaselineFrame *frame,
                StackFrame *interpFrame,
                uint32_t numStackValues) 
        */
        masm.setupUnalignedABICall(3, r3); // XXX: we don't use the 3
        masm.passABIArg(BaselineFrameReg); // this overwrites r3 (we saved it)
        masm.passABIArg(OsrFrameReg); // this overwrites r4 (don't care)
        masm.passABIArg(reg_nsv); // this overwrites r5 (don't care)
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *,
            jit::InitBaselineFrameForOsr));
        
        // We have to keep the return code in r3, but we are confident pretty
        // much all of our other argregs were trashed, so just take r4 to
        // get reg_code back.
        masm.pop2(r4, BaselineFrameReg);

        Label error;
        // If successful, call the code.
        masm.x_mtctr(r4);
        masm.addi(sp, sp, IonExitFrameLayout::SizeWithFooter());
        masm.addi(BaselineFrameReg, BaselineFrameReg, BaselineFrame::Size());
        masm.branchTest32(Assembler::Zero, r3, r3, &error);
        masm.bctr();

        // Otherwise fall through to the error path: initializing the frame
        // failed, probably OOM. Throw and terminate.
        masm.bind(&error);
        masm.x_mr(sp, BaselineFrameReg);
        masm.moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
        masm.addi(sp, sp, 2 * sizeof(uintptr_t));
        masm.b(&returnLabel);

        // Otherwise, this is the much simpler non-OSR path.
        masm.bind(&notOsr);
        // Load the scope chain into Baseline R1. This is already in an
        // argument register.
        masm.x_mr(R1.scratchReg(), reg_scope);
    }

    // 4. Call the function. Note this pushes PC again; this is expected.
    masm.callIon(reg_code);
    // (32)

    if (type == EnterJitBaseline) {
        // Baseline OSR returns here.
        masm.bind(&returnLabel);
    }

    // Insert nop()s so that this looks like a call stanza if we end on a
    // tail VM call.
    masm.x_nop();
    masm.x_nop();
#if defined(_PPC970_)
    masm.x_nop();
    masm.x_nop();
#endif

    // 5. Unwind the Ion stack and return to ABI state.
    // The return address was already popped.
    // Pop off arguments by using the frame descriptor on the stack.
    masm.pop(r4); // since we know it's not in use
    masm.x_srwi(r4, r4, FRAMESIZE_SHIFT);
    masm.add(sp, sp, r4);

    // 6. Store the returned value from JSReturnOperand in the environment.
    // We saved this in r17. This is a 64-bit jsval.
    masm.storeValue(JSReturnOperand, Address(r17, 0));

    // 7. Standard PowerPC epilogue.
    masm.bind(&epilogue);

    // Restore GPRs. (We have no FPRs to restore.)
    j -= 4; // otherwise r25 starts at 112!
    for (uint32_t i = 25; i > 12; i--) {
        masm.lwz(Register::FromCode(i), sp, j);
        j-=4;
    }
    JS_ASSERT(j == 56);

    // Tear down frame.
    masm.addi(sp, sp, sizeof(PPC_FRAMETYPE));

    // Fetch LR and CR from linkage area.
    masm.lwz(r0, sp, 8);
    masm.x_mtlr(r0);
    masm.lwz(r0, sp, 4);
    masm.x_mtcr(r0);

    // Return true.
    masm.x_li32(r3, JS_TRUE);
    masm.blr();

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

IonCode *
IonRuntime::generateInvalidator(JSContext *cx)
{
    AutoIonContextAlloc aica(cx);
    MacroAssembler masm(cx);

    // Baseline should never hit this.
    masm.x_trap();
    masm.lwz(r0, r0, 0); // lwz r0,0(0) // instant death

    // At this point, either of these two situations just happened:
    // 1) Execution has just returned from C code, which left the stack aligned
    // or
    // 2) Execution has just returned from Ion code, which left the stack
    // unaligned (and we need to realign it).
    //
    // We do the minimum amount of work in assembly and shunt the rest
    // off to InvalidationBailout.
    //
    // - Pop the return address from the invalidation epilogue call.
    // - Push the machine state onto the stack.
    // - Call the InvalidationBailout routine with SP.
    // - Now that the frame is bailed out, convert the invalidated
    //   frame into an exit frame.
    // - Do the normal check-return-code-and-thunk-to-the-interpreter dance.

    Linker linker(masm);
    IonCode *code = linker.newCode(cx, JSC::OTHER_CODE);
    IonSpew(IonSpew_Invalidate, "   invalidation thunk created at %p", (void *) code->raw());
    return code;
}

// If we have a function call where the callee has more formal arguments than
// the caller passes in, the JITs call the arguments-rectifier instead. It
// pushes some |undefined| values and re-pushes the other arguments on top of
// it (arguments are pushed last-to-first).
IonCode *
IonRuntime::generateArgumentsRectifier(JSContext *cx, ExecutionMode mode,
    void **returnAddrOut)
{
    MacroAssembler masm(cx);
    ispew("|||| generateArgumentsRectifier ||||");

    // ArgumentsRectifierReg contains the |nargs| pushed onto the current
    // frame. Including |this|, there are (|nargs| + 1) arguments to copy.
    // This is given to us when we are called by the code generator. ARM's
    // choice of r8 implies this should be a non-volatile register, but it
    // does allow it to be allocatable, so choose r19. ARM also freely
    // uses r0-r8 in this routine, so we're safe using r3-r8, even though
    // r8 is notionally our TailCallReg in Baseline.
    JS_ASSERT(ArgumentsRectifierReg == r19); // Assembler-ppc.h

    // Copy number of actual arguments. Keep this; we need it later.
    masm.load32(Address(stackPointerRegister,
        IonRectifierFrameLayout::offsetOfNumActualArgs()), r3);

    // Get the callee token and make a pointer. This should be a JSFunction.
    masm.load32(Address(stackPointerRegister,
       IonRectifierFrameLayout::offsetOfCalleeToken()), r4);
    masm.clearCalleeTag(r4, mode);

    // Get nargs into r8 from the JSFunction for later. This is uint16_t.
    JS_ASSERT(offsetof(JSFunction, nargs) < 65536);
    masm.lhz(r8, r4, offsetof(JSFunction, nargs));

    // Finally, r5 = r8 - r19 (ARM ma_sub is z = x - y), yielding
    // the number of |undefined|s to push on the stack.
    masm.subf(r5, r19, r8);

    // r6 = type, r7 = payload.
    // Using an fpreg for this doesn't really pay off for the number of
    // slots we typically end up pushing.
    masm.moveValue(UndefinedValue(), r6, r7);

    // Push as many |undefined| as we require. 
    {
        Label undefLoopTop;
        // On G5, try to schedule this manually. We know that mtctr will
        // be forced into leading a dispatch group, so the next dispatch
        // starts here. Force the mtctr away so that the branch references
        // a single atomic dispatch group.
        masm.x_mtctr(r5);
        // Save the stack pointer in r12. Try to emit elemental instructions
        // so it doesn't get clobbered.
        masm.x_mr(r12, stackPointerRegister);
#if defined(_PPC970_)
        masm.x_nop();
        masm.x_nop();
#endif
        masm.bind(&undefLoopTop);

        // This should be a single dispatch group, hopefully.
        masm.addi(stackPointerRegister, stackPointerRegister, -8);
        // Push payload first! We're big endian!
        masm.stw(r7, stackPointerRegister, 4);
        masm.stw(r6, stackPointerRegister, 0); // push2(r7,r6)
#if defined(_PPC970_)
        masm.x_nop();
        masm.x_bdnz(-4*4, Assembler::NotLikelyB,
            Assembler::DontLinkB); // reverse branch, likely
            // to be taken (thus don't set Lk bit)
#else
        masm.x_bdnz(-3*4, Assembler::NotLikelyB,
            Assembler::DontLinkB); // reverse branch, likely
            // to be taken (thus don't set Lk bit)
#endif
    }

    // Get a pointer to the topmost argument in r5, which should be
    // sp (i.e., r12) + r19 * 8 + sizeof(IonRectifierFrameLayout). Use
    // r5 again because all the other implementations clobber their own
    // counter registers (we used CTR).
    masm.x_slwi(r5, r19, 3);
    masm.add(r5, r5, r12);
    JS_ASSERT(sizeof(IonRectifierFrameLayout) < 32768);
    masm.addi(r5, r5, sizeof(IonRectifierFrameLayout));

    // Put the number of arguments pushed already + 1 into CTR (to count
    // |this|).
    // Everyone clobbers their rectifier register here, so we will too.
    masm.addi(r19, r19, 1);
    masm.x_mtctr(r19);
#if defined(_PPC970_)
    masm.x_nop();
    masm.x_nop();
    masm.x_nop();
#endif

    // Push again (i.e., copy) from the position on the stack. Since we're
    // copying 64-bit values, use the FPU.
    // Again, optimize for single dispatch group on 970.
    {
        Label copyLoopTop;
        masm.bind(&copyLoopTop);

        masm.lfd(f0, r5, 0);
        masm.stfdu(f0, stackPointerRegister, -8); // cracked
        masm.x_subi(r5, r5, 8);
        masm.x_bdnz(-3*4, Assembler::NotLikelyB,
            Assembler::DontLinkB); // reverse branch, likely
            // to be taken (thus don't set Lk bit)
    }

    // Turn nargs into number of bytes: (nargs + 1) * 8
    // This is still in r8 from before.
    masm.addi(r8, r8, 1);
    masm.x_slwi(r8, r8, 3);
    // Construct sizeDescriptor.
    masm.makeFrameDescriptor(r8, IonFrame_Rectifier);

    // Construct IonJSFrameLayout.
    masm.push3(r3, r4, r8); // nargs, calleeToken; frame descriptor on top

    // Call the target function.
    // Note that this code assumes the function is JITted, so we use
    // callIon.
    masm.load32(Address(r4, JSFunction::offsetOfNativeOrScript()), r3);
    masm.loadBaselineOrIonRaw(r3, r3, mode, NULL);
    masm.callIon(r3);

    uint32_t returnOffset = masm.currentOffset();

    // This is the current state of the stack upon return:
    // arg1
    //  ...
    // argN
    // num actual args
    // callee token
    // sizeDescriptor     <- sp now

    // Unwind the rectifier frame using the descriptor.
    masm.lwz(r3, r1, 0);
    masm.addi(r1, r1, 12); // throw away the token and nargs
    JS_ASSERT(FRAMESIZE_SHIFT < 16);
    masm.x_srwi(r3, r3, FRAMESIZE_SHIFT);

    // arg1
    //  ...
    // argN               <- sp now
    // num actual args
    // callee token
    // sizeDescriptor
    // return address

    // Now, discard pushed arguments, and exit.
    masm.add(r1, r1, r3);
    masm.ret(); // NOT blr

    Linker linker(masm);
    IonCode *code = linker.newCode(cx, JSC::OTHER_CODE);
    CodeOffsetLabel returnLabel(returnOffset);
    returnLabel.fixup(&masm);
    if (returnAddrOut)
        *returnAddrOut = (void *) (code->raw() + returnLabel.offset());
    ispew("^^^^ generateArgumentsRectifier ^^^^");
    return code;
}

static void
GenerateBailoutThunk(MacroAssembler &masm, uint32_t frameClass)
{
    // Baseline should never hit this.
    masm.x_trap();
    masm.lwz(r0, r0, 0); // lwz r0,0(0) // instant death
}

IonCode *
IonRuntime::generateBailoutTable(JSContext *cx, uint32_t frameClass)
{
    MacroAssembler masm(cx);

// Baseline should never hit this.
#if(0)
    Label bailout;
    for (size_t i = 0; i < BAILOUT_TABLE_SIZE; i++)
        masm.b(&bailout); // indirect branch
    masm.bind(&bailout);
#endif

    GenerateBailoutThunk(masm, frameClass);

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

IonCode *
IonRuntime::generateBailoutHandler(JSContext *cx)
{
    MacroAssembler masm(cx);
    GenerateBailoutThunk(masm, NO_FRAME_SIZE_CLASS_ID);

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

/* The VMWrapper is considered IonCode and MUST be called with callIon, or
   there will be no PC for it to pop and return to. */

IonCode *
IonRuntime::generateVMWrapper(JSContext *cx, const VMFunction &f)
{
    typedef MoveResolver::MoveOperand MoveOperand;

    JS_ASSERT(functionWrappers_);
    JS_ASSERT(functionWrappers_->initialized());
    VMWrapperMap::AddPtr p = functionWrappers_->lookupForAdd(&f);
    if (p)
        return p->value;

    ispew("=== generateVMWrapper ===");

    // Generate a separated code for the wrapper.
    MacroAssembler masm(cx);
    GeneralRegisterSet regs = GeneralRegisterSet(Register::Codes::WrapperMask);
    Label failure;

    // The Wrapper register set is a superset of the volatile register set.
    // Avoid conflicts with argument registers while discarding the result
    // after the function call.
    JS_STATIC_ASSERT((Register::Codes::VolatileMask &
        ~Register::Codes::WrapperMask) == 0);

    // An exit frame descriptor (PC followed by a BaselineJS frame descriptor)
    // should be on top of the stack. Link it up and add the footer:
    //
    // linkExitFrame() (ours) -> put sp in compartment runtime ionTop
    // PushWithPatch(-1) | footer part 1
    // Push(f)           | footer part 2
    // loadJSContext (GetIonContext()->runtime, mainThread.ionJSContext) to r3
    Register cxreg = r3;
    masm.enterExitFrameAndLoadContext(&f, cxreg, regs.getAny(),
            f.executionMode);

    // Save the base of the argument set stored on the stack.
    // Use temp registers (which means we have to make sure we only emit
    // elemental instructions that won't clobber them), so we needn't take
    // them either.
    Register argsBase = InvalidReg;
    if (f.explicitArgs) {
        argsBase = r12;
        //regs.take(argsBase);
        masm.addi(argsBase, stackPointerRegister,
                IonExitFrameLayout::SizeWithFooter());
    }

    // Reserve space for the outparameter.
    // TODO: Coalesce back-to-back stack reservations (but still use
    // reserveStack to make the coalesced reservation so that framePushed_
    // matches).
    Register outReg = InvalidReg;
    switch (f.outParam) {
      case Type_Value:
        outReg = r0;
        //regs.take(outReg);
        masm.reserveStack(sizeof(Value));
        masm.x_mr(outReg, stackPointerRegister);
        break;

      case Type_Handle:
        outReg = r0;
        //regs.take(outReg);
        // masm.PushEmptyRooted can clobber r12, so save it temporarily.
        if (f.explicitArgs && f.outParamRootType == VMFunction::RootValue)
            masm.x_mr(r4, argsBase); // it's not in use yet
        masm.PushEmptyRooted(f.outParamRootType);
        if (f.explicitArgs && f.outParamRootType == VMFunction::RootValue)
            masm.x_mr(argsBase, r4); // and it's still not
        masm.x_mr(outReg, stackPointerRegister);
        break;

      case Type_Int32:
      case Type_Pointer:
        outReg = r0;
        //regs.take(outReg);
        masm.reserveStack(sizeof(int32_t));
        masm.x_mr(outReg, stackPointerRegister);
        break;

      default:
        JS_ASSERT(f.outParam == Type_Void);
        break;
    }

    Register temp = regs.getAny();
    masm.setupUnalignedABICall(f.argc(), temp);
    masm.passABIArg(cxreg);

    size_t argDisp = 0;
    IonSpew(IonSpew_Codegen, ">>> VMFn: %i args to copy\n", f.explicitArgs);
    JS_ASSERT(f.explicitArgs < 8);

    // Copy any arguments.
    for (uint32_t explicitArg = 0; explicitArg < f.explicitArgs; explicitArg++) {
        MoveOperand from;
        switch (f.argProperties(explicitArg)) {
          case VMFunction::WordByValue:
            ispew("VMFunction: argument by value");
            masm.passABIArg(MoveOperand(argsBase, argDisp));
            argDisp += sizeof(void *);
            break;
          case VMFunction::DoubleByValue:
            // Values should be passed by reference, not by value, so we
            // assert that the argument is a double-precision float.
            ispew("VMFunction: double by value");
            JS_ASSERT(f.argPassedInFloatReg(explicitArg));
            masm.passABIArg(MoveOperand(argsBase, argDisp, MoveOperand::FLOAT));
            argDisp += 2 * sizeof(void *);
            break;
          case VMFunction::WordByRef:
            ispew("VMFunction: arg by ref");
            masm.passABIArg(
                MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE));
            argDisp += sizeof(void *);
            break;
          case VMFunction::DoubleByRef:
            ispew("VMFunction: double by ref");
            masm.passABIArg(
                MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE));
            // Even though the register is word-sized, the stack uses a
            // 64-bit quantity.
            argDisp += 2 * sizeof(void *);
            break;
        }
    }

    // Copy the implicit outparam, if any.
    if (outReg != InvalidReg)
        masm.passABIArg(outReg);

    masm.callWithABI(f.wrapped);

    // Test for failure, according to the various types.
    switch (f.failType()) {
      case Type_Object:
      case Type_Bool:
        // Called functions return bools, which are 0/false and non-zero/true.
        masm.branch32(Assembler::Equal, r3, Imm32(0), &failure);
        break;
      case Type_ParallelResult:
        masm.branch32(Assembler::NotEqual, r3, Imm32(TP_SUCCESS), &failure);
        break;
      default:
        JS_NOT_REACHED("unknown failure kind");
        break;
    }

    // Load the outparam and free any allocated stack.
    switch (f.outParam) {
      case Type_Handle:
        masm.popRooted(f.outParamRootType, ReturnReg, JSReturnOperand);
        break;

      case Type_Value:
        masm.loadValue(Address(stackPointerRegister, 0), JSReturnOperand);
        masm.freeStack(sizeof(Value));
        break;

      case Type_Int32:
      case Type_Pointer:
        masm.load32(Address(stackPointerRegister, 0), ReturnReg);
        masm.freeStack(sizeof(int32_t));
        break;

      default:
        JS_ASSERT(f.outParam == Type_Void);
        break;
    }

    masm.leaveExitFrame();
    // The PC is already on the stack. DO NOT blr().
    masm.retn(Imm32(sizeof(IonExitFrameLayout) +
                    f.explicitStackSlots() * sizeof(void *) +
                    f.extraValuesToPop * sizeof(Value)));

    masm.bind(&failure);
    masm.handleFailure(f.executionMode);

    Linker linker(masm);
    IonCode *wrapper = linker.newCode(cx, JSC::OTHER_CODE);
    if (!wrapper)
        return NULL;

    // linker.newCode may trigger a GC and sweep functionWrappers_ so we have to
    // use relookupOrAdd instead of add.
    if (!functionWrappers_->relookupOrAdd(p, &f, wrapper))
        return NULL;

    return wrapper;
}

IonCode *
IonRuntime::generatePreBarrier(JSContext *cx, MIRType type)
{
    MacroAssembler masm(cx);
    ispew("|||| generatePreBarrier ||||");

    RegisterSet save = RegisterSet(GeneralRegisterSet(Registers::VolatileMask),
                                   FloatRegisterSet(FloatRegisters::VolatileMask));
    masm.PushRegsInMask(save);

    JS_ASSERT(PreBarrierReg == r4);
    masm.movePtr(ImmWord(cx->runtime()), r3);

    masm.setupUnalignedABICall(2, r5);
    masm.passABIArg(r3);
    masm.passABIArg(r4);
    if (type == MIRType_Value) {
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, MarkValueFromIon));
    } else {
        JS_ASSERT(type == MIRType_Shape);
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, MarkShapeFromIon));
    }

    masm.PopRegsInMask(save);
    masm.ret(); // NOT blr

    ispew("^^^^ generatePreBarrier ^^^^");

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

typedef bool (*HandleDebugTrapFn)(
    JSContext *, BaselineFrame *, uint8_t *, JSBool *);
static const VMFunction HandleDebugTrapInfo =
    FunctionInfo<HandleDebugTrapFn>(HandleDebugTrap);

IonCode *
IonRuntime::generateDebugTrapHandler(JSContext *cx)
{
    MacroAssembler masm;
    ispew("|||| generateDebugTrapHandler ||||");

    Register scratch1 = r3;
    Register scratch2 = r4;

    // Load BaselineFrame pointer in scratch1.
    masm.x_mr(scratch1, BaselineFrameReg);
    masm.subPtr(Imm32(BaselineFrame::Size()), scratch1);

    // Enter a stub frame and call the HandleDebugTrap VM function. Ensure
    // the stub frame has a NULL ICStub pointer, since this pointer is marked
    // during GC.
    masm.movePtr(ImmWord((void *)NULL), BaselineStubReg);
    EmitEnterStubFrame(masm, scratch2);

    IonCompartment *ion = cx->compartment()->ionCompartment();
    IonCode *code = ion->getVMWrapper(HandleDebugTrapInfo);
    if (!code)
        return NULL;

    // EmitEnterStubFrame puts LR into r8, so we can just push it again
    // instead of another mflr and pushing that.
    masm.push2(r8, scratch1);
    EmitCallVM(code, masm);

    EmitLeaveStubFrame(masm);

    // If the stub returns |true|, we have to perform a forced return
    // (return from the JS frame). If the stub returns |false|, just return
    // from the trap stub so that execution continues at the current pc.
    Label forcedReturn;
    masm.branchTest32(Assembler::NonZero, ReturnReg, ReturnReg, &forcedReturn);
    masm.blr(); // ARM: mov pc,lr

    masm.bind(&forcedReturn);
    masm.loadValue(Address(BaselineFrameReg,
                           BaselineFrame::reverseOffsetOfReturnValue()),
                   JSReturnOperand);
    masm.x_mr(r1, BaselineFrameReg);
    masm.pop(BaselineFrameReg);
    masm.ret(); // NOT blr

    ispew("^^^^ generateDebugTrapHandler ^^^^");

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

