/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_cpu_ppc_assembler_h__
#define jsion_cpu_ppc_assembler_h__

#include <cstddef>
#include "jit/shared/Assembler-shared.h"
#include "assembler/assembler/PPCAssembler.h"
#include "jit/CompactBuffer.h"
#include "jit/IonCode.h"

#if DEBUG
#define ispew(x)    IonSpew(IonSpew_Codegen, "== " x " ==")
#else
#define ispew(x)    ;
#endif

namespace js {
namespace jit {

static const Register r0 = { JSC::PPCRegisters::r0 };
static const Register r1 = { JSC::PPCRegisters::r1 };
static const Register r2 = { JSC::PPCRegisters::r2 };
static const Register r3 = { JSC::PPCRegisters::r3 };
static const Register r4 = { JSC::PPCRegisters::r4 };
static const Register r5 = { JSC::PPCRegisters::r5 };
static const Register r6 = { JSC::PPCRegisters::r6 };
static const Register r7 = { JSC::PPCRegisters::r7 };
static const Register r8 = { JSC::PPCRegisters::r8 };
static const Register r9 = { JSC::PPCRegisters::r9 };
static const Register r10 = { JSC::PPCRegisters::r10 };
static const Register r11 = { JSC::PPCRegisters::r11 };
static const Register r12 = { JSC::PPCRegisters::r12 };
static const Register r13 = { JSC::PPCRegisters::r13 };
static const Register r14 = { JSC::PPCRegisters::r14 };
static const Register r15 = { JSC::PPCRegisters::r15 };
static const Register r16 = { JSC::PPCRegisters::r16 };
static const Register r17 = { JSC::PPCRegisters::r17 };
static const Register r18 = { JSC::PPCRegisters::r18 };
static const Register r19 = { JSC::PPCRegisters::r19 };
static const Register r20 = { JSC::PPCRegisters::r20 };
static const Register r21 = { JSC::PPCRegisters::r21 };
static const Register r22 = { JSC::PPCRegisters::r22 };
static const Register r23 = { JSC::PPCRegisters::r23 };
static const Register r24 = { JSC::PPCRegisters::r24 };
static const Register r25 = { JSC::PPCRegisters::r25 };
static const Register r26 = { JSC::PPCRegisters::r26 };
/*
    // Future expansion.
    // As of Firefox 26, JavaScript now enforces a maximum of 27 registers.
static const Register r27 = { JSC::PPCRegisters::r27 };
static const Register r28 = { JSC::PPCRegisters::r28 };
static const Register r29 = { JSC::PPCRegisters::r29 };
static const Register r30 = { JSC::PPCRegisters::r30 };
static const Register r31 = { JSC::PPCRegisters::r31 };
*/

static const FloatRegister f0 = { JSC::PPCRegisters::f0 };
static const FloatRegister f1 = { JSC::PPCRegisters::f1 };
static const FloatRegister f2 = { JSC::PPCRegisters::f2 };
// The rest of the FPRs are the business of the allocator, not the assembler.

static const Register InvalidReg = { JSC::PPCRegisters::invalid_reg };

static const Register stackPointerRegister = r1;
static const Register StackPointer = stackPointerRegister;
static const Register FramePointer = InvalidReg;

// PowerPC ABI invariably requires that the stack be aligned at function
// boundaries. We mostly obey this. Mostly.
static const uint32_t StackAlignment = 16; // quadword
static const uint32_t CodeAlignment = 16; // G4 and G5, but won't hurt G3
static const bool StackKeptAligned = false; // we align the stack ourselves

// These are used by AsmJS. XXX
static const uint32_t NativeFrameSize = 64;
static const uint32_t AlignmentAtPrologue = 16; // quadword
static const uint32_t AlignmentMidPrologue = 16;

static const Register tempRegister = r0;
static const Register addressTempRegister = r12;
// The OS X ABI documentation recommends r2 for this, not r11.
static const Register emergencyTempRegister = r2;

static const CRegisterID cr0 = { JSC::PPCRegisters::cr0 };
static const CRegisterID cr1 = { JSC::PPCRegisters::cr1 };
static const CRegisterID cr7 = { JSC::PPCRegisters::cr7 };

static const Register ReturnReg = r3; // ABI return register
static const Register PreBarrierReg = r4; // ARM uses r1
static const Register CallTempReg0 = r8; // ARM uses r5
static const Register CallTempReg1 = r9;
static const Register CallTempReg2 = r10;
// This is only used for setupUnalignedABICall().
static const Register CallTempReg3 = r7;
// This is used for setupUnalignedABICall() and some lowering operations.
static const Register CallTempReg4 = r5;
// This is only used for lowering.
static const Register CallTempReg5 = r6;
// WTF.
static const Register CallTempReg6 = r4; // Try desperately to keep r3!

// We don't have many.
static const Register CallTempNonArgRegs[] = { r2 };
static const uint32_t NumCallTempNonArgRegs =
    mozilla::ArrayLength(CallTempNonArgRegs);

static const Register ArgumentsRectifierReg = r19; // see Trampoline
static const Register OsrFrameReg = r6; // see Trampoline

static const FloatRegister fpTempRegister = f0;
static const FloatRegister ScratchFloatReg = f0; // ARM only needs one FPR
static const FloatRegister ReturnFloatReg = f1; // ABI return register
static const FloatRegister fpConversionRegister = f2; // for convertInt32ToDbl
static const FloatRegister InvalidFloatReg = {
    JSC::PPCRegisters::invalid_freg };

// Return operand from a JS -> JS call.
// ValueOperand defined in ion/RegisterSets.h; it's just a dyad of the
// two involved registers.
static const Register JSReturnReg_Data = r5;
static const Register JSReturnReg_Type = r6;
static const ValueOperand JSReturnOperand = ValueOperand(JSReturnReg_Type, JSReturnReg_Data);

class ABIArgGenerator
{
    uint32_t stackOffset_;
    uint32_t usedGPRs_;
    uint32_t usedFPRs_;
    ABIArg current_;

  public:
    ABIArgGenerator();
    ABIArg next(MIRType argType);
    ABIArg &current() { return current_; }
    uint32_t stackBytesConsumedSoFar() const { return stackOffset_; }

    // Note: these registers are all guaranteed to be different ...
    // ... which isn't hard when you have 32 registers. :)
    static const Register NonArgReturnVolatileReg1;
    static const Register NonArgReturnVolatileReg2;
    static const Register NonVolatileReg;
};

// We don't do anything special with these.
struct ImmTag : public Imm32
{
    ImmTag(JSValueTag mask)
      : Imm32(int32_t(mask))
    { }
};

struct ImmType : public ImmTag
{
    ImmType(JSValueType type)
      : ImmTag(JSVAL_TYPE_TO_TAG(type))
    { }
};

static const Scale ScalePointer = TimesFour;

class Operand
{
  public:
    enum Kind {
        // The operand references a GPR, not memory.
        REG,
        // The operand references a memory location, the effective address
        // of which is the base register value plus a 32-bit displacement.
        REG_DISP,
        // The operand references an FPR, not memory.
        FPREG
    };

    Kind kind_ : 4;
    int32_t index_ : 5;
    int32_t base_;
    int32_t disp_;

  public:
    explicit Operand(Register reg)
      : kind_(REG),
        base_(reg.code()),
        disp_(0)
    { }
    explicit Operand(FloatRegister reg)
      : kind_(FPREG),
        base_(reg.code()),
        disp_(0)
    { }
    explicit Operand(const Address &address)
      : kind_(REG_DISP),
        base_(address.base.code()),
        disp_(address.offset)
    { }

    Operand(Register reg, int32_t disp)
      : kind_(REG_DISP),
        base_(reg.code()),
        disp_(disp)
    { }

    Kind kind() const {
        return kind_;
    }
    Registers::Code reg() const {
        JS_ASSERT(kind() == REG);
        return (Registers::Code)base_;
    }
    Registers::Code base() const {
        JS_ASSERT(kind() == REG_DISP);
        return (Registers::Code)base_;
    }
    FloatRegisters::Code fpu() const {
        JS_ASSERT(kind() == FPREG);
        return (FloatRegisters::Code)base_;
    }
    int32_t disp() const {
        JS_ASSERT(kind() == REG_DISP);
        return disp_;
    }
    Register toReg() const {
        JS_ASSERT(kind() == REG);
        return Register::FromCode(base_);
    }
    FloatRegister toFReg() const {
        JS_ASSERT(kind() == FPREG);
        return FloatRegister::FromCode(base_);
    }
    Address toAddress() {
        JS_ASSERT(kind() == REG_DISP);
        return Address(Register::FromCode(base()), disp());
    }
};
// Modified from PPCAssembler.h, but for an Operand type
#define PPC_DISP_OK(x) (((x).disp() & 0xffff8000) == 0 || \
    ((x).offset & 0xffff8000) == 0xffff8000)

// This assembler uses a different format of immediate, so our usual
// macros need to be renoberated. (See PPCAssembler.h for what these are.)
#undef PPC_IMM_OK_U
#undef PPC_IMM_OK_S
#undef PPC_USE_UNSIGNED_COMPARE
#define PPC_IMM_OK_U(x) (((x) & 0xffff0000) == 0)
#define PPC_IMM_OK_S(x) (((x) & 0xffff8000) == 0 || \
    ((x) & 0xffff8000) == 0xffff8000)
#define PPC_USE_UNSIGNED_COMPARE(x) (x & JSC::PPCAssembler::ConditionUnsigned)
#define PPC_IMMOFFS_OK(x) PPC_IMM_OK_S(x)

class Assembler
{
  protected:
    struct RelativePatch {
        int32_t offset;
        void *target;
        Relocation::Kind kind;

        RelativePatch(int32_t offset, void *target, Relocation::Kind kind)
          : offset(offset),
            target(target),
            kind(kind)
        { }
    };

    // Big lie: PPCAssembler isn't the macro assembler, but asm's a
    // reserved keyword, so.
    JSC::PPCAssembler masm;

    typedef JSC::PPCAssembler::JmpSrc JmpSrc;
    typedef JSC::PPCAssembler::JmpDst JmpDst;
    typedef HashMap<int32_t, uint32_t, DefaultHasher<int32_t>,
        SystemAllocPolicy> BranchMap;

    js::Vector<CodeLabel, 0, SystemAllocPolicy> codeLabels_;
    js::Vector<RelativePatch, 8, SystemAllocPolicy> jumps_;
    BranchMap branchwords_;
    CompactBufferWriter jumpRelocations_;
    CompactBufferWriter dataRelocations_;
    CompactBufferWriter preBarriers_;
    size_t dataBytesNeeded_;
    bool enoughMemory_;

    bool ensureBranchwordsReady() {
        if (!branchwords_.initialized()) 
            enoughMemory_ &= branchwords_.init();
        return enoughMemory_;
    }

  public:
    typedef JSC::PPCAssembler::BranchLikelihood BLikelyB;
    typedef JSC::PPCAssembler::BranchLink BLinkB;

    static const BLikelyB NotLikelyB = JSC::PPCAssembler::NotLikelyBranch;
    static const BLikelyB LikelyB = JSC::PPCAssembler::LikelyBranch;

    static const BLinkB DontLinkB = JSC::PPCAssembler::DontLinkBranch;
    static const BLinkB LinkB = JSC::PPCAssembler::LinkBranch;

    enum Condition {
        // See MacroAssemblerPPC.h

        Equal = JSC::PPCAssembler::ConditionEQ,
        NotEqual = JSC::PPCAssembler::ConditionNE,
        GreaterThan = JSC::PPCAssembler::ConditionGT,
        GreaterThanOrEqual = JSC::PPCAssembler::ConditionGE,
        LessThan = JSC::PPCAssembler::ConditionLT,
        LessThanOrEqual = JSC::PPCAssembler::ConditionLE,
        Overflow = JSC::PPCAssembler::ConditionXEROV,

        // This is specific to the SO bits in the CR, not the general overflow
        // condition in the way Ion conceives of it. JM never used these.
        SOBit = JSC::PPCAssembler::ConditionSO,
        NSOBit = JSC::PPCAssembler::ConditionNS,

        NotSigned = JSC::PPCAssembler::ConditionGT,
        Signed = JSC::PPCAssembler::ConditionLT,
        Zero = JSC::PPCAssembler::ConditionEQ,
        NonZero = JSC::PPCAssembler::ConditionNE,

        Above = JSC::PPCAssembler::ConditionGT |
                JSC::PPCAssembler::ConditionUnsigned,
        Below = JSC::PPCAssembler::ConditionLT |
                JSC::PPCAssembler::ConditionUnsigned,
        BelowOrEqual = JSC::PPCAssembler::ConditionLE |
                JSC::PPCAssembler::ConditionUnsigned,
        AboveOrEqual = JSC::PPCAssembler::ConditionGE |
                JSC::PPCAssembler::ConditionUnsigned
    };

    enum DoubleCondition {
        // See MacroAssemblerPPC.h

        DoubleOrdered = JSC::PPCAssembler::DoubleConditionFO, // WTFO!
        DoubleUnordered = JSC::PPCAssembler::DoubleConditionFU, // WTFU!

        DoubleEqual = JSC::PPCAssembler::DoubleConditionEQ,
        DoubleNotEqual = JSC::PPCAssembler::DoubleConditionNE,
        DoubleGreaterThan = JSC::PPCAssembler::DoubleConditionGT,
        DoubleGreaterThanOrEqual = JSC::PPCAssembler::DoubleConditionGE,
        DoubleLessThan = JSC::PPCAssembler::DoubleConditionLT,
        DoubleLessThanOrEqual = JSC::PPCAssembler::DoubleConditionLE,

        DoubleEqualOrUnordered = JSC::PPCAssembler::DoubleConditionEQ_U,
        DoubleNotEqualOrUnordered = JSC::PPCAssembler::DoubleConditionNE_U,
        DoubleGreaterThanOrUnordered = JSC::PPCAssembler::DoubleConditionGT_U,
        DoubleGreaterThanOrEqualOrUnordered =
            JSC::PPCAssembler::DoubleConditionGE_U,
        DoubleLessThanOrUnordered = JSC::PPCAssembler::DoubleConditionLT_U,
        DoubleLessThanOrEqualOrUnordered =
            JSC::PPCAssembler::DoubleConditionLE_U
    };

    static inline Condition InvertCondition(Condition cond) {
        // EOR 0x08 to flip the sense (see PPCAssembler.h).
        return (Condition)((uint32_t)cond ^ 0x08);
    }
    static inline DoubleCondition InvertDoubleCondition(DoubleCondition cond) {
        // This *should* be the same.
        return (DoubleCondition)((uint32_t)cond ^ 0x08);
    }

    static void staticAsserts() {
        // Future expansion
    }

    static void TraceDataRelocations(JSTracer *trc, JitCode *code,
        CompactBufferReader &reader);
    static void TraceJumpRelocations(JSTracer *trc, JitCode *code,
        CompactBufferReader &reader);
    // MacroAssemblers hold onto gcthings, so they are traced by the GC.
    void trace(JSTracer *trc);

    bool oom() const {
        return masm.oom() ||
               !enoughMemory_ ||
               jumpRelocations_.oom() ||
               dataRelocations_.oom();
    }

    void ensureSpace(int space) { masm.ensureSpace(space); }

    // Copy the assembly code to the given buffer, and perform any pending
    // relocations relying on the target address.
    void executableCopy(uint8_t *buffer);

    void processCodeLabels(uint8_t *code);
    void copyJumpRelocationTable(uint8_t *buffer);
    void copyDataRelocationTable(uint8_t *buffer);
    void copyPreBarrierTable(uint8_t *dest);

    size_t numAsmJSAbsoluteLinks() const {
        return 0; // bogus
    }
    bool addCodeLabel(CodeLabel label) {
        return codeLabels_.append(label);
    }
    size_t numCodeLabels() const {
        return codeLabels_.length();
    }
    CodeLabel codeLabel(size_t i) {
        return codeLabels_[i];
    }

    // writeDataRelocation(*) needs to be called before the value or GCptr
    // being emitted is, in fact, emitted, or the offset will be wrong.
    void writeDataRelocation(const Value &val) {
        if (val.isMarkable()) {
            ensureSpace(16); // Make sure the load stays where we expect it.
            JS_ASSERT(static_cast<gc::Cell*>(val.toGCThing())->isTenured());
            dataRelocations_.writeUnsigned(currentOffset());
        }
    }
    void writeDataRelocation(const ImmGCPtr &ptr) {
        if (ptr.value) {
            ensureSpace(16); // Make sure the load stays where we expect it.
            dataRelocations_.writeUnsigned(currentOffset());
        }
    }
    void writeCodePointer(AbsoluteLabel *label);

    // Size of the instruction stream, in bytes.
    size_t size() const {
        return masm.size();
    }
    // Size of the jump relocation table, in bytes.
    size_t jumpRelocationTableBytes() const {
        return jumpRelocations_.length();
    }
    size_t dataRelocationTableBytes() const {
        return dataRelocations_.length();
    }
    size_t dataSize() const {
        return dataBytesNeeded_;
    }
    void writePrebarrierOffset(CodeOffsetLabel label) {
        preBarriers_.writeUnsigned(label.offset());
    }
    size_t preBarrierTableBytes() const {
        return preBarriers_.length();
    }

    size_t bytesNeeded() const {
#if !defined(USE_970_BRANCHING)
        // If we are using a constant pool for branches (i.e., not G5), we
        // need to flush the pool or size() will not properly reflect the
        // extra storage needed (and we'll get protection faults, or worse).
        masm.forceFlushConstantPool();
#endif
        return size() +
               dataSize() +
               jumpRelocationTableBytes() +
               dataRelocationTableBytes() +
               preBarrierTableBytes();
    }

    void forceConstantPoolFlushWithBarrier() {
#if !defined(USE_970_BRANCHING)
        masm.forceConstantPoolFlushWithBarrier();
#endif
    }

  protected:

    // These are general branch types used by Ion.

    JmpSrc _p_bc(Condition cond, Label *label) {
        JmpSrc j = masm.m_branch(
            static_cast<JSC::PPCAssembler::Condition>(cond));
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            // Record the offset of the jump against the label and save the
            // original branch word.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            JS_ASSERT(ensureBranchwordsReady());
            branchwords_.put(label->offset(), masm.setNextJump(j, prev));
        }
        return j;
    }

    JmpSrc _p_bc(DoubleCondition cond, Label *label) {
        JmpSrc j = masm.m_fbranch(
            static_cast<JSC::PPCAssembler::DoubleCondition>(cond));
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            // Record the offset of the jump against the label and save the
            // original branch word.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            JS_ASSERT(ensureBranchwordsReady());
            branchwords_.put(label->offset(), masm.setNextJump(j, prev));
        }
        return j;
    }

    JmpSrc _p_b(Label *label) {
        JmpSrc j = masm.m_jump();
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            // Record the offset of the jump against the label and save the
            // original branch word.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            JS_ASSERT(ensureBranchwordsReady());
            branchwords_.put(label->offset(), masm.setNextJump(j, prev));
        }
        return j;
    }

    JmpSrc _p_bc(Condition cond, RepatchLabel *label) {
        JmpSrc j = masm.m_branch(
            static_cast<JSC::PPCAssembler::Condition>(cond));
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            label->use(j.offset());
        }
        return j;
    }

    // XXX A template might be a better idea for this.
    JmpSrc _p_bc(DoubleCondition cond, RepatchLabel *label) {
        JmpSrc j = masm.m_fbranch(
            static_cast<JSC::PPCAssembler::DoubleCondition>(cond));
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            label->use(j.offset());
        }
        return j;
    }

    JmpSrc _p_b(RepatchLabel *label) {
        JmpSrc j = masm.m_jump();
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            label->use(j.offset());
        }
        return j;
    }

  public:
    void x_nop() { masm.x_nop(); }
    void x_trap() { masm.x_trap(); }
    void x_skip_stanza(int stanza_length) {
        // Used mostly by toggledCall and, indirectly, when repatching
        // toggledJumps and toggledCalls.
        masm.emitInt32(0x48000000 | (stanza_length + 4));
    }
    void b(Label *label) { _p_b(label); }
    void bc(Condition cond, Label *label) { _p_bc(cond, label); }
    void bc(DoubleCondition cond, Label *label) { _p_bc(cond, label); }
    void b(RepatchLabel *label) { _p_b(label); }
    void bc(Condition cond, RepatchLabel *label) { _p_bc(cond, label); }
    void bc(DoubleCondition cond, RepatchLabel *label) { _p_bc(cond, label); }
    void b(Register &r) { masm.x_baa(r.code()); }

    // Label methods.
    void bind(Label *label);
    void bind(RepatchLabel *label);
    void retarget(Label *label, Label *target);
    void retarget(Label *label, void *target, Relocation::Kind reloc,
            bool windback = false);
    uint32_t currentOffset() { return masm.label().offset(); }

    static void Bind(uint8_t *raw, AbsoluteLabel *label, const void *address);

    void retn(Imm32 n);
    void ret() {
        // Degenerate case, effectively doing what ARM does (i.e., the
        // long-winded PowerPC equivalent to pop(pc)).
        // Arguably this could pop r0:mtlr r0:blr but that's still going
        // to be two dispatch groups on G5, and we might care about LR
        // in Ion code.
        retn(Imm32(4));
    }
    void blr() { masm.blr(); }

    void call(Label *label) {
        // XXX. These might need to be changed to linkCall.
        if (label->bound()) {
            masm.linkJump(masm.m_call(), JmpDst(label->offset()));
        } else {
            JmpSrc j = masm.m_call();
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            JS_ASSERT(ensureBranchwordsReady());
            branchwords_.put(label->offset(), masm.setNextJump(j, prev));
        }
    }
    void call(void *address) {
        // XXX. These might need to be changed to linkCall.
        masm.linkJump(masm.m_call(), JmpDst((signed int)address));
    }
    void call(ImmPtr ptr) {
        // XXX. These might need to be changed to linkCall.
        masm.linkJump(masm.m_call(), JmpDst((signed int)ptr.value));
    }
    void call(const Register &reg) {
        x_mtctr(reg);
#if defined(_PPC970_)
        x_nop();
        x_nop();
        x_nop();
        x_nop();
#endif
        bctrl();
    }

    // These are for AsmJS, but ...
    void appendCallSite(const CallSiteDesc &desc) {
/*
        enoughMemory_ &= append(CallSite(desc, currentOffset(), framePushed_));
*/
    }

    void call(const CallSiteDesc &desc, const Register reg) {
        call(reg);
        appendCallSite(desc);
    }
    void call(const CallSiteDesc &desc, Label *label) {
        call(label);
        appendCallSite(desc);
    }
    void call(const CallSiteDesc &desc, AsmJSImmPtr imm) {
        call(imm);
        appendCallSite(desc);
    }
    void callExit(AsmJSImmPtr imm, uint32_t stackArgBytes) {
        MOZ_ASSUME_UNREACHABLE("callExit: AsmJS not implemented wtf!!!");
/*
        movePtr(imm, CallReg);
        // calls reg, storing the return address into sp[0]
        ma_callAndStoreRet(CallReg, stackArgBytes);
        appendCallSite(CallSiteDesc::Exit());
*/
    }
    void callIonFromAsmJS(const Register reg) {
        MOZ_ASSUME_UNREACHABLE("callIonFromAsmJS: AsmJS not implemented wtf!!!");
/*
        ma_callIonNoPush(reg);
        appendCallSite(CallSiteDesc::Exit());

        // The Ion ABI has the caller pop the return address off the stack.
        // The asm.js caller assumes that the call leaves sp unchanged, so bump
        // the stack.
        subPtr(Imm32(sizeof(void*)), sp);
*/
    }


    void breakpoint() {
        masm.x_trap();
    }

    // Instruction stubs follow. Essentially we convert Registers to
    // RegisterIDs using reg.code() (and the same for FloatRegisters to
    // FPRegisterIDs, and the same for condregs and SPRs). For example,
    // from x86:
/*
    void ucomisd(const FloatRegister &lhs, const FloatRegister &rhs) {
        masm.ucomisd_rr(rhs.code(), lhs.code());
    }
*/
    // This lets us use our existing codegen in PPCAssembler directly
    // instead of reimplementing it.

#define THUNK_CRCR(op) \
    void op(uint8_t t, uint8_t a, uint8_t b) { masm.op(t, a, b); }

        THUNK_CRCR(crand)
        THUNK_CRCR(crandc)
        THUNK_CRCR(cror)
        THUNK_CRCR(crorc)
        THUNK_CRCR(crxor)
#undef THUNK_CRCR

#define THUNK_ALU2(op) \
    void op(const Register &rd, const Register &ra, const Register &rb) { \
        masm.op(rd.code(), ra.code(), rb.code()); } \
    void op##_rc(const Register &rd, const Register &ra, const Register &rb) {\
        masm.op##_rc(rd.code(), ra.code(), rb.code()); }

        THUNK_ALU2(add)
        THUNK_ALU2(adde)
        THUNK_ALU2(addo)
        // eqv isn't technically part of this group, but the thunk will work.
        THUNK_ALU2(eqv)
        THUNK_ALU2(subf)
        THUNK_ALU2(subfc)
        THUNK_ALU2(subfe)
        THUNK_ALU2(subfo)
        THUNK_ALU2(divw)
        THUNK_ALU2(divwo)
        THUNK_ALU2(mullw)
        THUNK_ALU2(mullwo)
#undef THUNK_ALU2

#define THUNK_ALUI(op) \
    void op(const Register &rd, const Register &ra, int16_t im) { \
        masm.op(rd.code(), ra.code(), im); }

        THUNK_ALUI(addi)
        THUNK_ALUI(addic)
        THUNK_ALUI(x_subi)
        THUNK_ALUI(addis)
        // See x_sr_mulli.
        THUNK_ALUI(mulli)
        THUNK_ALUI(x_sr_mulli)
#undef THUNK_ALUI

#define THUNK_ALUE(op) \
    void op(const Register &rd, const Register &ra) { \
        masm.op(rd.code(), ra.code()); } \
    void op##_rc(const Register &rd, const Register &ra) { \
        masm.op##_rc(rd.code(), ra.code()); } 

        THUNK_ALUE(addme)
        THUNK_ALUE(addze)
        // We can cheat and put cntlzw here too.
        THUNK_ALUE(cntlzw)
#undef THUNK_ALUE

#define THUNK_BITALU2(op) \
    void op(const Register &rd, const Register &rs, const Register &rb) { \
        masm.op(rd.code(), rs.code(), rb.code()); } \
    void op##_rc(const Register &rd, const Register &rs, const Register &rb) {\
        masm.op##_rc(rd.code(), rs.code(), rb.code()); } 
    
        THUNK_BITALU2(andc)
        THUNK_BITALU2(nand)
        THUNK_BITALU2(nor)
        THUNK_BITALU2(slw)
        THUNK_BITALU2(srw)
        THUNK_BITALU2(sraw)
        THUNK_BITALU2(sld)
        THUNK_BITALU2(srd)
        THUNK_BITALU2(srad)
#undef THUNK_BITALU2

// Minor variant of BITALU2 for reserved logical C++ operators of the same name
#define THUNK_AOXN(op) \
    void op(const Register &rd, const Register &rs, const Register &rb) { \
        masm.op(rd.code(), rs.code(), rb.code()); } \
    void op##rc(const Register &rd, const Register &rs, const Register &rb) {\
        masm.op##rc(rd.code(), rs.code(), rb.code()); } 

        THUNK_AOXN(and_)
        THUNK_AOXN(or_)
        THUNK_AOXN(xor_)
#undef THUNK_AOXN

#define THUNK_BITALUI(op) \
    void op(const Register &rd, const Register &ra, uint16_t im) { \
        masm.op(rd.code(), ra.code(), im); }

        THUNK_BITALUI(ori)
        THUNK_BITALUI(oris)
        THUNK_BITALUI(xori)
        THUNK_BITALUI(xoris)
        THUNK_BITALUI(andi_rc)
        THUNK_BITALUI(andis_rc)
#undef THUNK_BITALUI

#define THUNK_ALUEXT(op) \
    void op(const Register &rd, const Register &rs) { \
        masm.op(rd.code(), rs.code()); }

        THUNK_ALUEXT(extsb)
        THUNK_ALUEXT(extsh)
        THUNK_ALUEXT(extsw)
#undef THUNK_ALUEXT

// XXX: See note in PPCAssembler.h
#define THUNK_FPUABC(op) \
    void op(const FloatRegister &rd, const FloatRegister &ra, \
            const FloatRegister &rb, const FloatRegister &rc) { \
        masm.op(rd.code(), ra.code(), rb.code(), rc.code()); } \
    void op##_rc(const FloatRegister &rd, const FloatRegister &ra, \
            const FloatRegister &rb, const FloatRegister &rc) { \
        masm.op##_rc(rd.code(), ra.code(), rb.code(), rc.code()); } 

    THUNK_FPUABC(fsel)
    THUNK_FPUABC(fmadd)
    THUNK_FPUABC(fnmsub)
#undef THUNK_FPUABC

// We're cheating and putting fmul here also even though it's fpuac encoded.
#define THUNK_FPUAB(op) \
    void op(const FloatRegister &rd, const FloatRegister &ra, \
            const FloatRegister &rb) { \
        masm.op(rd.code(), ra.code(), rb.code()); } \
    void op##_rc(const FloatRegister &rd, const FloatRegister &ra, \
            const FloatRegister &rb) { \
        masm.op##_rc(rd.code(), ra.code(), rb.code()); } 

        THUNK_FPUAB(fadd)
        THUNK_FPUAB(fdiv)
        THUNK_FPUAB(fsub)
        THUNK_FPUAB(fmul)
#undef THUNK_FPUAB

#define THUNK_FPUDS(op) \
    void op(const FloatRegister &rd, const FloatRegister &rs) { \
        masm.op(rd.code(), rs.code()); } \
    void op##_rc(const FloatRegister &rd, const FloatRegister &rs) { \
        masm.op##_rc(rd.code(), rs.code()); }

        THUNK_FPUDS(fabs)
        THUNK_FPUDS(fneg)
        THUNK_FPUDS(fmr)
        THUNK_FPUDS(fcfid)
        THUNK_FPUDS(fctiw)
        THUNK_FPUDS(fctiwz)
        THUNK_FPUDS(frsp)
        THUNK_FPUDS(frsqrte)

        // G5 only
        THUNK_FPUDS(fsqrt)
#undef THUNK_FPUDS

// For MEM thunks we introduce an alternative to handle the situation of
// Operands that use codes directly.
#define THUNK_MEMd(op) \
    void op(const Register &rd, const Register &rb, int16_t off) { \
        masm.op(rd.code(), rb.code(), off); } \
    void op(const Register &rd, const Registers::Code rb, int16_t off) { \
        masm.op(rd.code(), rb, off); }

        THUNK_MEMd(lbz)
        THUNK_MEMd(lha)
        THUNK_MEMd(lhz)
        THUNK_MEMd(lwz)
        THUNK_MEMd(ld)

        THUNK_MEMd(stb)
        THUNK_MEMd(stw)
        THUNK_MEMd(stwu)
        THUNK_MEMd(sth)
        THUNK_MEMd(std)
        THUNK_MEMd(stdu)
#undef THUNK_MEMd

#define THUNK_MEMx(op) \
    void op(const Register &rd, const Register &ra, const Register &rb) { \
        masm.op(rd.code(), ra.code(), rb.code()); } \
    void op(const Register &rd, const Registers::Code ra, const Register &rb) { \
        masm.op(rd.code(), ra, rb.code()); }

        THUNK_MEMx(lbzx)
        THUNK_MEMx(lhax)
        THUNK_MEMx(lhzx)
        THUNK_MEMx(lwzx)
        THUNK_MEMx(ldx)

        THUNK_MEMx(stbx)
        THUNK_MEMx(stwx)
        THUNK_MEMx(stwux)
        THUNK_MEMx(sthx)
        THUNK_MEMx(stdx)
        THUNK_MEMx(stdux)
#undef THUNK_MEMx

#define THUNK_FMEMd(op) \
    void op(const FloatRegister &rd, const Register &rb, int16_t off) { \
        masm.op(rd.code(), rb.code(), off); } \
    void op(const FloatRegister &rd, const Registers::Code rb, int16_t off) { \
        masm.op(rd.code(), rb, off); }

        THUNK_FMEMd(lfd)
        THUNK_FMEMd(lfs)
        THUNK_FMEMd(stfd)
        THUNK_FMEMd(stfs)
        THUNK_FMEMd(stfdu)
#undef THUNK_FMEMd

#define THUNK_FMEMx(op) \
    void op(const FloatRegister &rd, const Register &ra, const Register &rb) {\
        masm.op(rd.code(), ra.code(), rb.code()); } \
    void op(const FloatRegister &rd, const Registers::Code ra, const Register &rb) {\
        masm.op(rd.code(), ra, rb.code()); }

        THUNK_FMEMx(lfdx)
        THUNK_FMEMx(lfsx)
        THUNK_FMEMx(stfdx)
        THUNK_FMEMx(stfsx)
#undef THUNK_FMEMx

// mfspr and mtspr are "special." Like, Happy Gilmore special. You know.
// XXX. This *should* work for the registers we defined above, though.
#define THUNK_MTFSPR(op) \
    void op(const SPRegisterID spr, const Register &rr) { \
        masm.op(spr, rr.code()); }

    THUNK_MTFSPR(mtspr)
    THUNK_MTFSPR(mfspr)
#undef THUNK_MTFSPR
// Now all the little mnemonic variants (x_ class).
#define THUNK_MTFXSPR(spr) \
    void x_mt##spr(const Register &rr) { \
        masm.x_mt##spr(rr.code()); } \
    void x_mf##spr(const Register &rr) { \
        masm.x_mf##spr(rr.code()); }

    THUNK_MTFXSPR(ctr)
    THUNK_MTFXSPR(xer)
    THUNK_MTFXSPR(lr)
#undef THUNK_MTFXSPR

// Grab-bags
#define THUNK_2GPR(op) \
    void op(const Register &rd, const Register &rs) {\
        masm.op(rd.code(), rs.code()); }

    THUNK_2GPR(neg)
    THUNK_2GPR(cmpw)
    THUNK_2GPR(cmplw)
#undef THUNK_2GPR

#define THUNK_GPRI16(op) \
    void op(const Register &rd, int16_t imm) { masm.op(rd.code(), imm); }

    THUNK_GPRI16(cmpwi)
    THUNK_GPRI16(cmplwi)
#undef THUNK_GPRI16

// Generalized complex load structures
// Not worth creating macros for
void x_li(const Register &rd, int16_t i) { masm.x_li(rd.code(), i); }
void x_lis(const Register &rd, int16_t i) { masm.x_lis(rd.code(), i); }
void x_li32(const Register &rd, int32_t i) { masm.x_li32(rd.code(), i); }
void x_p_li32(const Register &rd, int32_t i) { masm.x_p_li32(rd.code(), i); }

// "Stuff"
// Not worth creating macros for
void x_mr(const Register &rd, const Register &rs) {
    masm.or_(rd.code(), rs.code(), rs.code());
}
void fcmpu(const FloatRegister &ra, const FloatRegister &rb) {
    masm.fcmpu(ra.code(), rb.code());
}
void srawi(const Register &rd, const Register &rs, int8_t n) {
    masm.srawi(rd.code(), rs.code(), n);
}
void rlwinm(const Register &rd, const Register &rs, int8_t sh, int8_t mb,
        int8_t me) {
    masm.rlwinm(rd.code(), rs.code(), sh, mb, me);
}
void rlwimi(const Register &rd, const Register &rs, int8_t sh, int8_t mb,
        int8_t me) {
    masm.rlwimi(rd.code(), rs.code(), sh, mb, me);
}
void bctr() { masm.bctr(JSC::PPCAssembler::DontLinkBranch); }
void bctrl() { masm.bctr(JSC::PPCAssembler::LinkBranch); }
void mfcr(const Register &rd) { masm.mfcr(rd.code()); }
void mtcrf(uint16_t m, const Register &rs) { masm.mtcrf(m, rs.code()); }
void x_mtcr(const Register &rs) { masm.x_mtcr(rs.code()); }
void mcrf(const CRegisterID t, const CRegisterID s) { masm.mcrf(t, s); }
void mcrfs(const CRegisterID cr, uint8_t a) { masm.mcrfs(cr, a); }
void mtfsb0(uint8_t b) { masm.mtfsb0(b); }
void mtfsb1(uint8_t b) { masm.mtfsb1(b); }
// G5 only
void mfocrf(const Register &rd, const CRegisterID s) {
    masm.mfocrf(rd.code(), s); }

void x_baa(const Register &t) { masm.x_baa(t.code()); }
void x_slwi(const Register &rd, const Register &rs, int16_t n) {
    masm.x_slwi(rd.code(), rs.code(), n);
}
void x_srwi(const Register &rd, const Register &rs, int16_t n) {
    masm.x_srwi(rd.code(), rs.code(), n);
}

// *shiver*
void x_mcrxr(const CRegisterID cr) { masm.x_mcrxr(cr); }

// Ion branches. Methodjit never used these types of branches. They must
// only be used where we are certain the displacement is less than 64K
// because there is no branch stanza, and patchBranch won't work.
// TODO: Notice that these take offsets, not labels. A future project.
// See PPCAssembler.h for what has to be done; in particular, these do
// not handle the case of forward defined labels. We may need a backpatcher
// to handle that case at the time of code generation.
void x_beq(const CRegisterID cr, int32_t offset, BLikelyB lk, BLinkB ln) {
    masm.x_beq(cr, offset, lk, ln);
}
void x_bne(const CRegisterID cr, int32_t offset, BLikelyB lk, BLinkB ln) {
    masm.x_bne(cr, offset, lk, ln);
}
void x_bdnz(int32_t offset, BLikelyB lk, BLinkB ln) {
    masm.x_bdnz(offset, lk, ln);
}
// Unconditional branch, always relative, with a 24-bit displacement.
void x_b(int32_t offset, BLinkB ln) {
    masm.x_b(offset, ln);
}

    uint32_t actualOffset(uint32_t x);
    uint32_t actualIndex(uint32_t x);

    void flushBuffer() {
#if !defined(USE_970_BRANCHING)
        // This only makes a difference for non-G5.
        // CodeGenerator sometimes calls this, so we should barrier it.
        masm.forceConstantPoolFlushWithBarrier();
#endif
    }

    void finish();

    void setPrinter(Sprinter *sp) {
    }

    void align(int alignment) { masm.align(alignment); }

    // Patching.

    static size_t patchWrite_NearCallSize() {
        return 4;
    }
    static uintptr_t getPointer(uint8_t *instPtr) {
        __asm__("trap\n");
        uintptr_t *ptr = ((uintptr_t *) instPtr) - 1;
        return *ptr;
    }
    // Write a relative call at the start location |dataLabel|.
    // Note that this DOES NOT patch data that comes before |label|.
    static void patchWrite_NearCall(CodeLocationLabel startLabel, CodeLocationLabel target);

    static void patchWrite_Imm32(CodeLocationLabel dataLabel, Imm32 toWrite);

    static void patchDataWithValueCheck(CodeLocationLabel data, 
                                        PatchedImmPtr newData,
                                        PatchedImmPtr expectedData);
    static void patchDataWithValueCheck(CodeLocationLabel data, ImmPtr newData,
                                        ImmPtr expectedData);

    static uint32_t nopSize() {
        return 4;
    }

    static uint8_t *nextInstruction(uint8_t *cur, uint32_t *count);

    // Toggle a jmp or cmp emitted by toggledJump() (q.v.). ToggleToJmp
    // puts a nop in the leading instruction, and ToggleToCmp puts a
    // branch-around-it there -- see toggledJump() for the rationale.
    // The default state is nop() (i.e., the jump is enabled).
    static void ToggleToJmp(CodeLocationLabel inst) {
        uint32_t *ptr = (uint32_t *)inst.raw();
        // This better be our branch instruction!
        JS_ASSERT(*ptr == (0x48000000 | (PPC_BRANCH_STANZA_LENGTH + 4)));
            // b stanza+4, accounting for the nop we're about to store
        *ptr = 0x60000000;
        JSC::ExecutableAllocator::cacheFlush(ptr, 4);
    }
    static void ToggleToCmp(CodeLocationLabel inst) {
        uint32_t *ptr = (uint32_t *)inst.raw();
        // This better be a nop!
        JS_ASSERT(*ptr == 0x60000000); // ori r0,r0,0 (nop)
        *ptr = (0x48000000 | (PPC_BRANCH_STANZA_LENGTH + 4));
        JSC::ExecutableAllocator::cacheFlush(ptr, 4);
    }
    // This is the same idea, for toggledCall() (q.v.)
    static void ToggleCall(CodeLocationLabel inst, bool enabled) {
        uint32_t *ptr = (uint32_t *)inst.raw();
        // Assert if it's not one or the other; we could get called to
        // enable something that's already enabled, etc.
        JS_ASSERT(*ptr == (0x48000000 | (PPC_BRANCH_STANZA_LENGTH + 4)) ||
            *ptr == 0x60000000);
        *ptr = (enabled) ? 0x60000000 // don't skip the call if enabled
            : (0x48000000 | (PPC_BRANCH_STANZA_LENGTH + 4));
        JSC::ExecutableAllocator::cacheFlush(ptr, 4);
    }

    void writeRelocation(JmpSrc src) {
        jumpRelocations_.writeUnsigned(src.offset());
    }
    void addPendingJump(JmpSrc src, void *target, Relocation::Kind kind) {
#ifdef DEBUG
        IonSpew(IonSpew_Codegen, "##addPendingJump offs %08x to %08x\n",
            (uint32_t)src.offset(), (uint32_t)target);
#endif
        enoughMemory_ &= jumps_.append(RelativePatch(src.offset(),
            target, kind));
        if (kind == Relocation::JITCODE)
            writeRelocation(src);
    }
    void addPendingCall(JmpSrc src, void *target, Relocation::Kind kind) {
#ifdef DEBUG
        IonSpew(IonSpew_Codegen, "##addPendingCall offs %08x to %08x\n",
            (uint32_t)src.offset() - PPC_CALL_STANZA_LENGTH, (uint32_t)target);
#endif
        enoughMemory_ &= jumps_.append(RelativePatch(src.offset() -
            PPC_CALL_STANZA_LENGTH, target, kind));
        if (kind == Relocation::JITCODE)
            writeRelocation(src);
    }
    void addPendingBJump(JmpSrc src, void *target, Relocation::Kind kind) {
#ifdef DEBUG
        IonSpew(IonSpew_Codegen, "##addPendingBJump offs %08x to %08x\n",
            (uint32_t)src.offset() - PPC_BRANCH_STANZA_LENGTH,
                (uint32_t)target);
#endif
        enoughMemory_ &= jumps_.append(RelativePatch(src.offset() -
            PPC_BRANCH_STANZA_LENGTH, target, kind));
        if (kind == Relocation::JITCODE)
            writeRelocation(src);
    }

    // The buffer is about to be linked. Make sure any constant pools or excess
    // bookkeeping has been flushed to the instruction stream (we do this when
    // size() is called, though, so we don't need to do it here).
    void flush() {
    }

    CodeOffsetLabel pushWithPatch(const ImmWord &word) {
        ensureSpace(PPC_BRANCH_STANZA_LENGTH + 8);
        CodeOffsetLabel l = currentOffset();
        x_p_li32(tempRegister, word.value);
        stwu(tempRegister, stackPointerRegister, -4);
        return l;
    }

    void b(void *target, Relocation::Kind reloc = Relocation::HARDCODED,
            bool windback = true) {
        JmpSrc src = masm.m_jump();
        if (windback)
            addPendingBJump(src, target, reloc);
        else
            addPendingJump(src, target, reloc);
    }
    void bc(Condition cond, void *target,
            Relocation::Kind reloc = Relocation::HARDCODED,
            bool windback = false) {
        JmpSrc src = masm.m_branch(
            static_cast<JSC::PPCAssembler::Condition>(cond));
        if (windback)
            addPendingBJump(src, target, reloc);
        else
            addPendingJump(src, target, reloc);
    }

    void b(JitCode *target) {
        b(target->raw(), Relocation::JITCODE);
    }
    void bc(Condition cond, JitCode *target) {
        bc(cond, target->raw(), Relocation::JITCODE);
    }
    void call(AsmJSImmPtr qqqq) {
        ispew("call(AsmJSImmPtr)");
        /* movePtr(imm, CallReg); call(CallReg); */
        MOZ_ASSUME_UNREACHABLE("AsmJS not implemented wtf!!!");
    }

    void call(JitCode *target) {
        ispew("call(JitCode)");
        // Since the target is known, this is somewhat easier than callIon.
        // Emit the code to push PC. This will clobber LR, like callIon does.
        // Because everything is computed LR-relative, we need to make sure
        // that a constant pool does not break this up.
        ensureSpace(PPC_CALL_STANZA_LENGTH + 24); // paranoia
        x_b(4, LinkB);
        // We have to use r12; r0 will turn the addi into li!
        x_mflr(addressTempRegister); // (0)
        // Make the PC point to the instruction after the call stanza.
        addi(addressTempRegister, addressTempRegister,
            PPC_CALL_STANZA_LENGTH + 12); // (4)
        stwu(addressTempRegister, stackPointerRegister, -4); // cracked (8)
        
        JmpSrc src = masm.m_call(); // (12)
        // Return here (12 + PPC_CALL_STANZA_LENGTH)
        addPendingCall(src, target->raw(), Relocation::JITCODE);
    }
    void call(ImmWord target) {
        JmpSrc src = masm.m_call();
        addPendingCall(src, (void *)target.value, Relocation::HARDCODED);
    }

  public:
    Assembler()
      : dataBytesNeeded_(0),
        enoughMemory_(true)
    {
        if(!ensureBranchwordsReady())
            fprintf(stderr, "TenFourFox: OMG: ensureBranchwordsReady() isn't\n");
    }

}; // class Assembler

// Note: the following methods, etc., are not part of the Assembler class.

void PatchJump(CodeLocationJump jump, CodeLocationLabel label);

// Regalloc stuffies.
static const uint32_t NumIntArgRegs = 8;
static const uint32_t NumFloatArgRegs = 13;

static inline bool
GetIntArgReg(uint32_t usedIntArgs, uint32_t usedFloatArgs, Register *out)
{
    if (usedIntArgs >= NumIntArgRegs)
        return false;
    // Arg regs start at 3!
    *out = Register::FromCode(3+usedIntArgs);
    return true;
}
static inline bool
GetFloatArgReg(uint32_t usedIntArgs, uint32_t usedFloatArgs,
    FloatRegister *out)
{
    if (usedFloatArgs >= NumFloatArgRegs)
        return false;
    // Arg regs start at 1!
    *out = FloatRegister::FromCode(1+usedFloatArgs);
    return true;
}

// Get a register in which we plan to put a quantity that will be used as an
// integer argument. This differs from GetIntArgReg in that if we have no more
// actual argument registers to use we will fall back on using whatever
// CallTempReg* don't overlap the argument registers, and only fail once those
// run out too. Unfortunately, PPC uses a lot of argregs, so this really means
// just r11, and maybe r2 in the future. This doesn't have to be ABI compliant,
// so we're not limited to the standard ABI argument registers.
static inline bool
GetTempRegForIntArg(uint32_t usedIntArgs, uint32_t usedFloatArgs,
    Register *out)
{
    if (GetIntArgReg(usedIntArgs, usedFloatArgs, out))
        return true;
    // Unfortunately, we have to assume things about the point at which
    // GetIntArgReg returns false, because we need to know how many registers
    // it can allocate.
    usedIntArgs -= NumIntArgRegs;
    if (usedIntArgs >= NumCallTempNonArgRegs)
        return false;
    *out = CallTempNonArgRegs[usedIntArgs];
    return true;
}


} // namespace jit
} // namespace js

#endif // jsion_cpu_ppc_assembler_h__

