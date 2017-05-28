/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "mozilla/DebugOnly.h"

#include "jsnum.h"

#include "CodeGenerator-ppc.h"
#include "jit/MIR.h"
#include "jit/MIRGraph.h"
#include "jit/shared/CodeGenerator-shared-inl.h"
#include "vm/Shape.h"

#include "jsscriptinlines.h"
#include "jit/ExecutionModeInlines.h"

#include "jscntxt.h"
#include "jscompartment.h"
#include "jsmath.h"
#include "jit/IonFrames.h"
#include "jit/MoveEmitter.h"
#include "jit/IonCompartment.h"
#include "jit/ParallelFunctions.h"

using namespace js;
using namespace js::jit;

using mozilla::DebugOnly;

namespace js {
namespace jit {

CodeGeneratorPPC::CodeGeneratorPPC(MIRGenerator *gen,
    LIRGraph *graph,
    MacroAssembler *masm)
  : CodeGeneratorShared(gen, graph, masm),
    deoptLabel_(NULL)
{
}

static const uint32_t FrameSizes[] = { 128, 256, 512, 1024 };

FrameSizeClass
FrameSizeClass::FromDepth(uint32_t frameDepth)
{
    for (uint32_t i = 0; i < JS_ARRAY_LENGTH(FrameSizes); i++) {
        if (frameDepth < FrameSizes[i])
            return FrameSizeClass(i);
    }

    return FrameSizeClass::None();
}

FrameSizeClass
FrameSizeClass::ClassLimit()
{
    return FrameSizeClass(JS_ARRAY_LENGTH(FrameSizes));
}

uint32_t
FrameSizeClass::frameSize() const
{
    JS_ASSERT(class_ != NO_FRAME_SIZE_CLASS_ID);
    JS_ASSERT(class_ < JS_ARRAY_LENGTH(FrameSizes));

    return FrameSizes[class_];
}

ValueOperand
CodeGeneratorPPC::ToValue(LInstruction *ins, size_t pos)
{
    Register typeReg = ToRegister(ins->getOperand(pos + TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getOperand(pos + PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

ValueOperand
CodeGeneratorPPC::ToOutValue(LInstruction *ins)
{
    Register typeReg = ToRegister(ins->getDef(TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getDef(PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

ValueOperand
CodeGeneratorPPC::ToTempValue(LInstruction *ins, size_t pos)
{
    Register typeReg = ToRegister(ins->getTemp(pos + TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getTemp(pos + PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

bool
CodeGeneratorPPC::visitValue(LValue *value)
{
    const ValueOperand out = ToOutValue(value);
    masm.moveValue(value->value(), out);
    return true;
}

bool
CodeGeneratorPPC::visitOsrValue(LOsrValue *value)
{
    const LAllocation *frame   = value->getOperand(0);
    const ValueOperand out     = ToOutValue(value);
    const ptrdiff_t frameOffset = value->mir()->frameOffset();

    masm.loadValue(Operand(ToRegister(frame), frameOffset), out);
    return true;
}

bool
CodeGeneratorPPC::visitBox(LBox *box)
{
    const LDefinition *type = box->getDef(TYPE_INDEX);

    DebugOnly<const LAllocation *> a = box->getOperand(0);
    JS_ASSERT(!a->isConstant());

    // On PPC, like x86, the input operand and the output payload have the same
    // virtual register. All that needs to be written is the type tag for the
    // type definition. This is not endian-driven.
    masm.x_li32(ToRegister(type), MIRTypeToTag(box->type()));
    return true;
}

bool
CodeGeneratorPPC::visitBoxDouble(LBoxDouble *box)
{
    const LAllocation *in = box->getOperand(0);
    const ValueOperand out = ToOutValue(box);

    masm.boxDouble(ToFloatRegister(in), out);
    return true;
}

bool
CodeGeneratorPPC::visitUnbox(LUnbox *unbox)
{
    // Note that for unbox, the type and payload indexes are switched on the
    // inputs.
    MUnbox *mir = unbox->mir();

    if (mir->fallible()) {
        masm.cmpPtr(ToOperand(unbox->type()),
            ImmWord(MIRTypeToTag(mir->type())));
        if (!bailoutIf(Assembler::NotEqual, unbox->snapshot()))
            return false;
    }
    return true;
}

bool
CodeGeneratorPPC::visitDouble(LDouble *ins)
{
    // PowerPC can't inline doubles, so we can only reserve them places
    // in memory from which we fetch them later (see linkAbsoluteLabels).
    const LDefinition *out = ins->getDef(0);
    masm.loadConstantDouble(ins->getDouble(), ToFloatRegister(out));
    return true;
}

/* In the below functions:
   V = value
   T = type
   I = int32
   D = double */

bool
CodeGeneratorPPC::visitLoadSlotV(LLoadSlotV *load)
{
    const ValueOperand out = ToOutValue(load);
    Register base = ToRegister(load->input());
    int32_t offset = load->mir()->slot() * sizeof(js::Value);

    masm.loadValue(Operand(base, offset), out);
    return true;
}

bool
CodeGeneratorPPC::visitLoadSlotT(LLoadSlotT *load)
{
    Register base = ToRegister(load->input());
    int32_t offset = load->mir()->slot() * sizeof(js::Value);

    if (load->mir()->type() == MIRType_Double)
        masm.loadInt32OrDouble(Operand(base, offset),
            ToFloatRegister(load->output()));
    else
        masm.load32(Address(base, offset + NUNBOX32_PAYLOAD_OFFSET),
            ToRegister(load->output()));
    return true;
}

bool
CodeGeneratorPPC::visitStoreSlotT(LStoreSlotT *store)
{
    Register base = ToRegister(store->slots());
    int32_t offset = store->mir()->slot() * sizeof(js::Value);

    const LAllocation *value = store->value();
    MIRType valueType = store->mir()->value()->type();

    if (store->mir()->needsBarrier())
        emitPreBarrier(Address(base, offset), store->mir()->slotType());

    if (valueType == MIRType_Double) {
        masm.loadDouble(Address(base, offset), ToFloatRegister(value));
        return true;
    }

    // Store the type tag if needed.
    if (valueType != store->mir()->slotType())
        masm.storeTypeTag(ImmType(ValueTypeFromMIRType(valueType)), Operand(base, offset));

    // Store the payload.
    if (value->isConstant())
        masm.storePayload(*value->toConstant(), Operand(base, offset));
    else
        masm.storePayload(ToRegister(value), Operand(base, offset));

    return true;
}

bool
CodeGeneratorPPC::visitLoadElementT(LLoadElementT *load)
{
    Operand source = createArrayElementOperand(ToRegister(load->elements()), load->index());

    if (load->mir()->needsHoleCheck()) {
        Assembler::Condition cond = masm.testMagic(Assembler::Equal, source);
        if (!bailoutIf(cond, load->snapshot()))
            return false;
    }

    if (load->mir()->type() == MIRType_Double) {
        FloatRegister fpreg = ToFloatRegister(load->output());
        if (load->mir()->loadDoubles()) {
            if (source.kind() == Operand::REG_DISP)
                masm.loadDouble(source.toAddress(), fpreg);
            else {
                JS_NOT_REACHED("unexpected Operand type in visitLoadElement");
                return false;
            }
        } else {
            masm.loadInt32OrDouble(source, fpreg);
        }
    } else {
        masm.loadPayload(masm.ToPayload(source), ToRegister(load->output()));
    }

    return true;
}

void
CodeGeneratorPPC::storeElementTyped(const LAllocation *value, MIRType valueType, MIRType elementType,
                                    const Register &elements, const LAllocation *index)
{
    Operand dest = createArrayElementOperand(elements, index);

    if (valueType == MIRType_Double) {
        JS_ASSERT(dest.kind() == Operand::REG_DISP);
        masm.storeDouble(ToFloatRegister(value), dest.toAddress());
        return;
    }

    // Store the type tag if needed.
    if (valueType != elementType)
        masm.storeTypeTag(ImmType(ValueTypeFromMIRType(valueType)), dest);

    // Store the payload.
    if (value->isConstant())
        masm.storePayload(*value->toConstant(), dest);
    else
        masm.storePayload(ToRegister(value), dest);
}

bool
CodeGeneratorPPC::visitImplicitThis(LImplicitThis *lir)
{
    Register callee = ToRegister(lir->callee());
    const ValueOperand out = ToOutValue(lir);

    // The implicit |this| is always |undefined| if the function's environment
    // is the current global.
    GlobalObject *global = &gen->info().script()->global();
    masm.cmpPtr(Operand(callee, JSFunction::offsetOfEnvironment()), ImmGCPtr(global));

    // TODO: OOL stub path.
    if (!bailoutIf(Assembler::NotEqual, lir->snapshot()))
        return false;

    masm.moveValue(UndefinedValue(), out);
    return true;
}

typedef bool (*InterruptCheckFn)(JSContext *);
static const VMFunction InterruptCheckInfo = FunctionInfo<InterruptCheckFn>(InterruptCheck);

bool
CodeGeneratorPPC::visitInterruptCheck(LInterruptCheck *lir)
{
    OutOfLineCode *ool = oolCallVM(InterruptCheckInfo, lir, (ArgList()), StoreNothing());
    if (!ool)
        return false;

    void *interrupt = (void*)&gen->compartment->rt->interrupt;
    masm.cmp32(AbsoluteAddress(interrupt), Imm32(0));
    masm.bc(Assembler::NonZero, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorPPC::visitCompareB(LCompareB *lir)
{
    MCompare *mir = lir->mir();

    const ValueOperand lhs = ToValue(lir, LCompareB::Lhs);
    const LAllocation *rhs = lir->rhs();
    const Register output = ToRegister(lir->output());

    JS_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    Label notBoolean, done;
    masm.branchTestBoolean(Assembler::NotEqual, lhs, &notBoolean);
    {
        if (rhs->isConstant())
            masm.cmp32(lhs.payloadReg(), Imm32(rhs->toConstant()->toBoolean()));
        else
            masm.cmp32(lhs.payloadReg(), ToRegister(rhs));
        masm.emitSet(JSOpToCondition(mir->compareType(), mir->jsop()), output);
        masm.jump(&done);
    }
    masm.bind(&notBoolean);
    {
        masm.move32(Imm32(mir->jsop() == JSOP_STRICTNE), output);
    }

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorPPC::visitCompareBAndBranch(LCompareBAndBranch *lir)
{
    MCompare *mir = lir->mir();
    const ValueOperand lhs = ToValue(lir, LCompareBAndBranch::Lhs);
    const LAllocation *rhs = lir->rhs();

    JS_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    if (mir->jsop() == JSOP_STRICTEQ)
        masm.branchTestBoolean(Assembler::NotEqual, lhs, lir->ifFalse()->lir()->label());
    else
        masm.branchTestBoolean(Assembler::NotEqual, lhs, lir->ifTrue()->lir()->label());

    if (rhs->isConstant())
        masm.cmp32(lhs.payloadReg(), Imm32(rhs->toConstant()->toBoolean()));
    else
        masm.cmp32(lhs.payloadReg(), ToRegister(rhs));
    emitBranch(JSOpToCondition(mir->compareType(), mir->jsop()),
            lir->ifTrue(),
            lir->ifFalse());
    return true;
}

bool
CodeGeneratorPPC::visitCompareV(LCompareV *lir)
{
    MCompare *mir = lir->mir();
    Assembler::Condition cond = JSOpToCondition(mir->compareType(),
        mir->jsop());
    const ValueOperand lhs = ToValue(lir, LCompareV::LhsInput);
    const ValueOperand rhs = ToValue(lir, LCompareV::RhsInput);
    const Register output = ToRegister(lir->output());

    JS_ASSERT(IsEqualityOp(mir->jsop()));

    Label notEqual, done;
    masm.cmp32(lhs.typeReg(), rhs.typeReg());
    masm.bc(Assembler::NotEqual, &notEqual);
    {
        masm.cmp32(lhs.payloadReg(), rhs.payloadReg());
        masm.emitSet(cond, output);
        masm.jump(&done);
    }
    masm.bind(&notEqual);
    {
        masm.move32(Imm32(cond == Assembler::NotEqual), output);
    }

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorPPC::visitCompareVAndBranch(LCompareVAndBranch *lir)
{
    MCompare *mir = lir->mir();
    Assembler::Condition cond = JSOpToCondition(
        mir->compareType(), mir->jsop());
    const ValueOperand lhs = ToValue(lir, LCompareVAndBranch::LhsInput);
    const ValueOperand rhs = ToValue(lir, LCompareVAndBranch::RhsInput);

    JS_ASSERT(mir->jsop() == JSOP_EQ || mir->jsop() == JSOP_STRICTEQ ||
              mir->jsop() == JSOP_NE || mir->jsop() == JSOP_STRICTNE);

    Label *notEqual;
    if (cond == Assembler::Equal)
        notEqual = lir->ifFalse()->lir()->label();
    else
        notEqual = lir->ifTrue()->lir()->label();

    masm.cmp32(lhs.typeReg(), rhs.typeReg());
    masm.bc(Assembler::NotEqual, notEqual);
    masm.cmp32(lhs.payloadReg(), rhs.payloadReg());
    emitBranch(cond, lir->ifTrue(), lir->ifFalse());

    return true;
}

double
test(double x, double y)
{
    return x + y;
}

bool
CodeGeneratorPPC::generatePrologue()
{
    if (0) { // gen->compilingAsmJS()) {
/*
        masm.Push(lr);
*/
        // Note that this automatically sets MacroAssembler::framePushed().
        masm.reserveStack(frameDepth_);
    } else {
        // Note that this automatically sets MacroAssembler::framePushed().
        masm.reserveStack(frameSize());
        masm.checkStackAlignment(); // XXX?
    }

    // Allocate returnLabel_ on the heap, so we don't run its destructor and
    // assert-not-bound in debug mode on compilation failure.
    returnLabel_ = new HeapLabel();

    return true;
}

bool
CodeGeneratorPPC::generateEpilogue()
{
// XXX
    masm.bind(returnLabel_);

    // Pop the stack we allocated at the start of the function.
    masm.freeStack(frameSize());
    JS_ASSERT(masm.framePushed() == 0);

    masm.ret();
    return true;
}

bool
OutOfLineBailout::accept(CodeGeneratorPPC *codegen)
{
    return codegen->visitOutOfLineBailout(this);
}

void
CodeGeneratorPPC::emitBranch(Assembler::Condition cond, MBasicBlock *mirTrue,
                                   MBasicBlock *mirFalse)
{
    LBlock *ifTrue = mirTrue->lir();
    LBlock *ifFalse = mirFalse->lir();

    if (isNextBlock(ifFalse)) {
        masm.bc(cond, ifTrue->label());
    } else {
        masm.bc(masm.InvertCondition(cond), ifFalse->label());
        if (!isNextBlock(ifTrue))
            masm.b(ifTrue->label());
    }
}
void
CodeGeneratorPPC::emitBranch(Assembler::DoubleCondition cond,
                            MBasicBlock *mirTrue, MBasicBlock *mirFalse)
{
    LBlock *ifTrue = mirTrue->lir();
    LBlock *ifFalse = mirFalse->lir();

    if (isNextBlock(ifFalse)) {
        masm.bc(cond, ifTrue->label());
    } else {
        masm.bc(masm.InvertDoubleCondition(cond), ifFalse->label());
        if (!isNextBlock(ifTrue))
            masm.b(ifTrue->label());
    }
}

bool
CodeGeneratorPPC::visitTestIAndBranch(LTestIAndBranch *test)
{
    const LAllocation *opd = test->input();

    // Test the operand
    masm.and_rc(r0, ToRegister(opd), ToRegister(opd));
    emitBranch(Assembler::NonZero, test->ifTrue(), test->ifFalse());
    return true;
}

bool
CodeGeneratorPPC::visitTestDAndBranch(LTestDAndBranch *test)
{
    const LAllocation *opd = test->input();

    // x86 does this based on ucomisd's behaviour:
    //             Z  P  C
    //            ---------
    //      NaN    1  1  1
    //        >    0  0  0
    //        <    0  0  1
    //        =    1  0  0
    //
    // NaN is falsey, so comparing against 0 and then using the Z flag is
    // enough to determine which branch to take for that processor. Our
    // equivalent is fcmpu and NotEqualOrUnordered.
    masm.zeroDouble(f0);
    masm.fcmpu(ToFloatRegister(opd), ScratchFloatReg);
    emitBranch(Assembler::DoubleNotEqualOrUnordered,
        test->ifTrue(), test->ifFalse());
    return true;
}

void
CodeGeneratorPPC::emitCompare(MCompare::CompareType type, const LAllocation *left, const LAllocation *right)
{
    if (right->isConstant())
        masm.cmp32(ToRegister(left), Imm32(ToInt32(right)));
    else {
        masm.cmp32(ToRegister(left), ToOperand(right));
    }
}

bool
CodeGeneratorPPC::visitCompare(LCompare *comp)
{
    emitCompare(comp->mir()->compareType(), comp->left(), comp->right());
    masm.emitSet(JSOpToCondition(comp->mir()->compareType(),
        comp->jsop()), ToRegister(comp->output()));
    return true;
}

bool
CodeGeneratorPPC::visitCompareAndBranch(LCompareAndBranch *comp)
{
    emitCompare(comp->mir()->compareType(), comp->left(), comp->right());
    Assembler::Condition cond = JSOpToCondition(comp->mir()->compareType(),
            comp->jsop());
    emitBranch(cond, comp->ifTrue(), comp->ifFalse());
    return true;
}

bool
CodeGeneratorPPC::visitCompareD(LCompareD *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());
    masm.compareDouble(cond, lhs, rhs);
    masm.emitSet(cond, ToRegister(comp->output()));
    return true;
}

bool
CodeGeneratorPPC::visitNotI(LNotI *ins)
{
    masm.cmp32(ToRegister(ins->input()), Imm32(0));
    masm.emitSet(Assembler::Equal, ToRegister(ins->output()));
    return true;
}

bool
CodeGeneratorPPC::visitNotD(LNotD *ins)
{
    FloatRegister opd = ToFloatRegister(ins->input());

    masm.zeroDouble(f0);
    //masm.compareDouble(Assembler::DoubleEqualOrUnordered, opd, f0);
    masm.fcmpu(opd, f0); // more efficient
    masm.emitSet(Assembler::DoubleEqualOrUnordered, ToRegister(ins->output()));
    return true;
}

bool
CodeGeneratorPPC::visitCompareDAndBranch(LCompareDAndBranch *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());
    //masm.compareDouble(cond, lhs, rhs);
    masm.fcmpu(lhs, rhs);
    emitBranch(cond, comp->ifTrue(), comp->ifFalse());
    return true;
}

bool
CodeGeneratorPPC::generateOutOfLineCode()
{
    if (!CodeGeneratorShared::generateOutOfLineCode())
        return false;

    if (deoptLabel_) {
        // All non-table-based bailouts will go here.
        masm.bind(deoptLabel_);

        // Push the frame size, so the handler can recover the IonScript.
        masm.push(Imm32(frameSize()));

        IonCompartment *ion = GetIonContext()->compartment->ionCompartment();
        IonCode *handler = ion->getGenericBailoutHandler();

        masm.b(handler->raw(), Relocation::IONCODE);
    }

    return true;
}

class BailoutJump {
    Assembler::Condition cond_;

  public:
    BailoutJump(Assembler::Condition cond) : cond_(cond)
    { }
    void operator()(MacroAssembler &masm, uint8_t *code) const {
        masm.bc(cond_, code, Relocation::HARDCODED);
    }
    void operator()(MacroAssembler &masm, Label *label) const {
        masm.bc(cond_, label);
    }
};

class BailoutLabel {
    Label *label_;

  public:
    BailoutLabel(Label *label) : label_(label)
    { }
    void operator()(MacroAssembler &masm, uint8_t *code) const {
        masm.retarget(label_, code, Relocation::HARDCODED);
    }
    void operator()(MacroAssembler &masm, Label *label) const {
        masm.retarget(label_, label);
    }
};

template <typename T> bool
CodeGeneratorPPC::bailout(const T &binder, LSnapshot *snapshot)
{
// XXX
    CompileInfo &info = snapshot->mir()->block()->info();
    switch (info.executionMode()) {
      case ParallelExecution: {
        // in parallel mode, make no attempt to recover, just signal an error.
/*
        Label *ool;
        if (!ensureOutOfLineParallelAbort(&ool))
            return false;
        binder(masm, ool);
        return true;
*/ 
        return false;
      }

      case SequentialExecution: break;
    }

    if (!encode(snapshot))
        return false;

    // Though the assembler doesn't track all frame pushes, at least make sure
    // the known value makes sense. We can't use bailout tables if the stack
    // isn't properly aligned to the static frame size.
    JS_ASSERT_IF(frameClass_ != FrameSizeClass::None() && deoptTable_,
                 frameClass_.frameSize() == masm.framePushed());

#if(0)
// Not sure if we want this.
    // On x64, bailout tables are pointless, because 16 extra bytes are
    // reserved per external jump, whereas it takes only 10 bytes to encode a
    // a non-table based bailout.
    if (assignBailoutId(snapshot)) {
        binder(masm, deoptTable_->raw() + snapshot->bailoutId() * BAILOUT_TABLE_ENTRY_SIZE);
        return true;
    }
#endif

    // We could not use a jump table, either because all bailout IDs were
    // reserved, or a jump table is not optimal for this frame size or
    // platform. Whatever, we will generate a lazy bailout.
    OutOfLineBailout *ool = new OutOfLineBailout(snapshot);
    if (!addOutOfLineCode(ool))
        return false;

    binder(masm, ool->entry());
    return true;
}

bool
CodeGeneratorPPC::bailoutIf(Assembler::Condition condition, LSnapshot *snapshot)
{
    return bailout(BailoutJump(condition), snapshot);
}

bool
CodeGeneratorPPC::bailoutFrom(Label *label, LSnapshot *snapshot)
{
    JS_ASSERT(label->used() && !label->bound());
    return bailout(BailoutLabel(label), snapshot);
}

bool
CodeGeneratorPPC::bailout(LSnapshot *snapshot)
{
    Label label;
    masm.jump(&label);
    return bailoutFrom(&label, snapshot);
}

bool
CodeGeneratorPPC::visitOutOfLineBailout(OutOfLineBailout *ool)
{
    if (!deoptLabel_)
        deoptLabel_ = new HeapLabel();

    masm.push(Imm32(ool->snapshot()->snapshotOffset()));
    masm.b(deoptLabel_);
    return true;
}


bool
CodeGeneratorPPC::visitMinMaxD(LMinMaxD *ins)
{
    FloatRegister first = ToFloatRegister(ins->first());
    FloatRegister second = ToFloatRegister(ins->second());
    FloatRegister output = ToFloatRegister(ins->output());

    // This is written based on x86 and ARM where they've chosen to ensure
    // that the "first" FPR is also the output FPR. Seems to me we could
    // generalize that, but in the instructions below, first == output.
    JS_ASSERT(first == output);

    Assembler::DoubleCondition cond = ins->mir()->isMax()
                               ? Assembler::DoubleGreaterThan
                               : Assembler::DoubleLessThan;
    Label nan, equal, returnSecond, done;

    masm.fcmpu(second, first);
    masm.bc(Assembler::DoubleUnordered, &nan); // If either is NaN, NaN.
    masm.bc(Assembler::DoubleEqual, &equal);
    masm.bc(cond, &returnSecond);
    masm.b(&done);
    
    // Check for zero, since -0 == 0.
    masm.bind(&equal);

    masm.zeroDouble(f0);
    masm.fcmpu(first, f0);
    masm.bc(Assembler::NotEqual, &done); // not 0

    // So now both operands are either -0 or 0.
    if (ins->mir()->isMax()) {
        // Remember, first == output
        masm.fadd(first, second, first); // -0 + -0 = -0 and -0 + 0 = 0.
    } else {
        // This is gross. Stolen from ARM.
        // The confusing part is that the ARM macroops don't match the
        // actual mnemonics. FU, Moz.
        // Remember, first == output
        masm.fneg(first, first);
        masm.fsub(first, first, second); // first = first - second
        masm.fneg(first, first);
    }
    masm.b(&done);

    // Return NaN (from the constant stack) if NaN.
    masm.bind(&nan);
    masm.loadStaticDouble(&js_NaN, output);
    masm.b(&done);

    // Otherwise, return second.
    masm.bind(&returnSecond);
    masm.fmr(second, output);

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorPPC::visitAbsD(LAbsD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    FloatRegister output = ToFloatRegister(ins->output());

    masm.fabs(output, input);
    return true;
}

// Nothing actually calls this yet. Not fully implemented on non-G5. XXX
bool
CodeGeneratorPPC::visitSqrtD(LSqrtD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    FloatRegister output = ToFloatRegister(ins->output());

    masm.x_fsqrt(output, input);
    return true;
}

// Nothing actually calls this yet. Not fully implemented on non-G5. XXX
bool
CodeGeneratorPPC::visitPowHalfD(LPowHalfD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    FloatRegister output = ToFloatRegister(ins->output());

    Label done, dosqrt;

    // Math.pow(-Inf, 0.5) == Inf
    masm.loadStaticDouble(&js_NegativeInfinity, fpTempRegister);
    masm.fcmpu(input, fpTempRegister);
    // Branch if not -Infinity.
    masm.branchDouble(Assembler::DoubleNotEqualOrUnordered, input,
        fpTempRegister, &dosqrt);
    // Otherwise, flip sign of input, return that.
    masm.fneg(output, input);
    masm.b(&done);

    // Math.pow(-0, 0.5) == 0 == Math.pow(0, 0.5).
    masm.bind(&dosqrt);
    masm.zeroDouble(f0);
    masm.fadd(output, f0, input); // Adding 0 converts any -0 to 0.
    masm.x_fsqrt(output, output);

    masm.bind(&done);
    return true;
}

// See below. ARM doesn't have this, but we implement it because it gets
// infrequently used code out of the main execution path (particularly
// valuable for G5).
class OutOfLineUndoALUOperation : public OutOfLineCodeBase<CodeGeneratorPPC>
{
    LInstruction *ins_;

  public:
    OutOfLineUndoALUOperation(LInstruction *ins)
        : ins_(ins)
    { }

    virtual bool accept(CodeGeneratorPPC *codegen) {
        return codegen->visitOutOfLineUndoALUOperation(this);
    }
    LInstruction *ins() const {
        return ins_;
    }
};

bool
CodeGeneratorPPC::visitAddI(LAddI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);

    // Unlike MMul, MAdd and MSub classes don't have a way of checking if
    // they'll overflow, so we must always assume they may (hence, add32o).
    // Yuck. If this ever changes, consider adding code here to use the
    // simpler add32 forms, because these can serialize XER and reduce
    // throughput.
    if (rhs->isConstant())
        masm.add32o(Imm32(ToInt32(rhs)), ToRegister(lhs), ToRegister(dest));
    else
        masm.add32o(ToRegister(lhs), ToOperand(rhs), ToRegister(dest));

    if (ins->snapshot()) {
        if (ins->recoversInput()) {
            OutOfLineUndoALUOperation *ool = new OutOfLineUndoALUOperation(ins);
            if (!addOutOfLineCode(ool))
                return false;
            masm.bc(Assembler::Overflow, ool->entry());
        } else {
            if (!bailoutIf(Assembler::Overflow, ins->snapshot()))
                return false;
        }
    }
    return true;
}

bool
CodeGeneratorPPC::visitSubI(LSubI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);

    // Again, use the overflow-checking forms, since LIR cannot tell us
    // if this is "overflowable."
    if (rhs->isConstant())
        masm.sub32o(Imm32(ToInt32(rhs)), ToRegister(lhs), ToRegister(rhs));
    else
        masm.sub32o(ToRegister(lhs), ToOperand(rhs), ToRegister(dest));

    if (ins->snapshot()) {
        if (ins->recoversInput()) {
            OutOfLineUndoALUOperation *ool = new OutOfLineUndoALUOperation(ins);
            if (!addOutOfLineCode(ool))
                return false;
            masm.bc(Assembler::Overflow, ool->entry());
        } else {
            if (!bailoutIf(Assembler::Overflow, ins->snapshot()))
                return false;
        }
    }
    return true;
}

// See above
bool
CodeGeneratorPPC::visitOutOfLineUndoALUOperation(OutOfLineUndoALUOperation *ool)
{
    LInstruction *ins = ool->ins();
    Register reg = ToRegister(ins->getDef(0));

    mozilla::DebugOnly<LAllocation *> lhs = ins->getOperand(0);
    LAllocation *rhs = ins->getOperand(1);

    JS_ASSERT(reg == ToRegister(lhs));

    // The net effect is to undo the effect of the ALU operation, which was
    // performed on the output register and overflowed. This only makes a
    // difference if the input register and the output register are the same
    // to satisfy the constraint imposed by any RECOVERED_INPUT operands to
    // the bailout snapshot. If they aren't, silently ignore; the state should
    // be self-recoverable.

    JS_ASSERT(rhs->isGeneralReg()); // FPR case shouldn't get here.
    if (reg != ToRegister(rhs)) {
        if (rhs->isConstant()) {
            Imm32 constant(ToInt32(rhs));
            if (ins->isAddI())
                masm.sub32(constant, reg);
            else
                masm.add32(constant, reg);
        } else {
            if (ins->isAddI())
                masm.sub32o(reg, ToOperand(rhs), reg);
            else
                masm.add32o(reg, ToOperand(rhs), reg);
        }
    }
    // We may now bailout safely.
    return bailout(ool->ins()->snapshot());
}

// This is part of visitMulI (see end of function).
class MulNegativeZeroCheck : public OutOfLineCodeBase<CodeGeneratorPPC>
{
    LMulI *ins_;

  public:
    MulNegativeZeroCheck(LMulI *ins)
      : ins_(ins)
    { }

    virtual bool accept(CodeGeneratorPPC *codegen) {
        return codegen->visitMulNegativeZeroCheck(this);
    }
    LMulI *ins() const {
        return ins_;
    }
};

// This mostly replaces our old x_sr_mulli. We do the strength reduction at
// this level now.
bool
CodeGeneratorPPC::visitMulI(LMulI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);

    MMul *mul = ins->mir();
    JS_ASSERT_IF(mul->mode() == MMul::Integer, !mul->canBeNegativeZero() && !mul->canOverflow());

    if (rhs->isConstant()) {
        // Bailout on -0.0
        int32_t constant = ToInt32(rhs);
        if (mul->canBeNegativeZero() && constant <= 0) {
            Assembler::Condition bailoutCond =
                (constant == 0) ? Assembler::LessThan : Assembler::Equal;
            masm.xor_(tempRegister, tempRegister, tempRegister);
            masm.cmpw(ToRegister(lhs), tempRegister);
            if (!bailoutIf(bailoutCond, ins->snapshot()))
                    return false;
        }

        // Refer to x_sr_mulli for the specific issues around mulli on
        // PowerPC, but in short, depending on the implementation mulli in
        // the worst case can take up to five cycles (G5 is about four).
        // We optimize more trivial cases here than ARM or x86 to desperately
        // avoid this. Assume most integer ops are about one cycle.
        // Only bother if we cannot overflow, because repeated addo may
        // leave XER in an indeterminate state, and serialization problems
        // with XER may cause this to be slower. TODO: Summary overflow
        // would help a lot here.
        switch (constant) {
          // -5 is probably pushing it.
          case -4:
            if (!mul->canOverflow()) {
                masm.neg(ToRegister(dest), ToRegister(lhs));
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(dest));
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(dest));
                break;
            }
            // else fall through
          case -3:
            if (!mul->canOverflow()) {
                masm.neg(ToRegister(dest), ToRegister(lhs));
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(dest));
                // Remember: subf T,A,B is T <= B - A !!
                masm.subf(ToRegister(dest), ToRegister(lhs), ToRegister(dest));
                break;
            }
            // else fall through
          case -2:
            // This will be fine for XER.
            masm.neg(ToRegister(dest), ToRegister(lhs));
            if (mul->canOverflow()) {
                masm.addo(ToRegister(dest), ToRegister(dest), ToRegister(dest));
            } else {
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(dest));
            }
            break;
          case -1:
            masm.neg(ToRegister(dest), ToRegister(lhs));
            return true; // escape overflow check
          case 0:
            masm.xor_(ToRegister(dest), ToRegister(dest), ToRegister(dest));
            return true; // escape overflow check
          case 1:
            if (ToRegister(dest) != ToRegister(lhs))
                masm.x_mr(ToRegister(dest), ToRegister(lhs));
            return true; // escape overflow check
          case 2:
            // This will be fine for XER.
            if (mul->canOverflow()) {
                masm.addo(ToRegister(dest), ToRegister(dest), ToRegister(dest));
            } else {
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(dest));
            }
            break;
          case 3:
            // This is on SunSpider, btw.
            if (!mul->canOverflow()) {
                masm.add(ToRegister(dest), ToRegister(lhs), ToRegister(lhs));
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(lhs));
                break;
            }
            // else fall through
          // 4 and 8 are below
          case 5:
            if (!mul->canOverflow()) {
                masm.add(ToRegister(dest), ToRegister(lhs), ToRegister(lhs));
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(dest));
                masm.add(ToRegister(dest), ToRegister(lhs), ToRegister(dest));
                break;
            }
            // else fall through
          case 6:
            if (!mul->canOverflow()) {
                masm.add(ToRegister(dest), ToRegister(lhs), ToRegister(lhs));
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(lhs));
                masm.add(ToRegister(dest), ToRegister(dest), ToRegister(dest));
                break;
            }
            // else fall through
          default:
            // TODO: See if the general case of 2^x + 1 can be optimized into
            // shifts. We may be able to also use sign extension for the neg
            // case.
            if (!mul->canOverflow() && constant > 0) {
                // Use shift if cannot overflow and constant is power of 2.
                // TODO: ARM tries to recover bits here in the overflow case.
                int32_t shift;
                JS_FLOOR_LOG2(shift, constant);
                if (((1 << shift) == constant) && (shift < 65536)) {
                    masm.x_slwi(ToRegister(dest), ToRegister(lhs), shift); 
                    return true;
                }
            }
            // Oh well, we tried. At least see if we can avoid borking XER.
            if (!mul->canOverflow()) {
                if (PPC_IMM_OK_S(ToInt32(rhs))) {
                    masm.mulli(ToRegister(dest), ToRegister(lhs),
                        ToInt32(rhs)); 
                } else {
                    masm.x_li32(tempRegister, ToInt32(rhs));
                    masm.mullw(ToRegister(dest), ToRegister(lhs),
                         tempRegister);
                }
            } else {
                // Must use mullwo to emit overflow state to XER.
                masm.x_li32(tempRegister, ToInt32(rhs));
                masm.mullwo(ToRegister(dest), ToRegister(lhs), tempRegister);
            }
        }

        // Bailout on overflow
        if (mul->canOverflow() &&
            !bailoutIf(Assembler::Overflow, ins->snapshot()))
                return false;
    } else {
        // Oh, praise God we don't have to implement this for operand! Sheesh!
        if (mul->canOverflow()) {
            masm.mullwo(ToRegister(dest), ToRegister(lhs), ToRegister(rhs));
        } else {
            // Avoid serializing XER.
            masm.mullw(ToRegister(dest), ToRegister(lhs), ToRegister(rhs));
        }

        // Bailout on overflow
        if (mul->canOverflow() &&
            !bailoutIf(Assembler::Overflow, ins->snapshot()))
                return false;

        if (mul->canBeNegativeZero()) {
            // Jump to an OOL path if the result is 0.
            MulNegativeZeroCheck *ool = new MulNegativeZeroCheck(ins);
            if (!addOutOfLineCode(ool))
                return false;

            masm.and_rc(tempRegister, ToRegister(lhs), ToRegister(lhs));
            masm.bc(Assembler::Zero, ool->entry());
            masm.bind(ool->rejoin());
        }
    }

    return true;
}

// From the mul above
bool
CodeGeneratorPPC::visitMulNegativeZeroCheck(MulNegativeZeroCheck *ool)
{
    LMulI *ins = ool->ins();
    Register result = ToRegister(ins->output());
    Operand lhsCopy = ToOperand(ins->lhsCopy());
    Operand rhs = ToOperand(ins->rhs());
    JS_ASSERT_IF(lhsCopy.kind() == Operand::REG,
        lhsCopy.reg() != result.code());

    // Result is -0 if lhs or rhs is negative. Based on the mul above, the
    // lhs and rhs should be in registers.
    JS_ASSERT(lhsCopy.kind() == Operand::REG);
    JS_ASSERT(rhs.kind() == Operand::REG);

    // TODO. Seems we could do better with this.
    masm.xor_(tempRegister, tempRegister, tempRegister); // r0 = 0
    masm.cmpw(Register::FromCode(lhsCopy.reg()), tempRegister);
    if (!bailoutIf(Assembler::LessThan, ins->snapshot()))
        return false;
    masm.cmpw(Register::FromCode(rhs.reg()), tempRegister);
    if (!bailoutIf(Assembler::LessThan, ins->snapshot()))
        return false;
    masm.b(ool->rejoin());
    return true;
}

bool
CodeGeneratorPPC::visitDivI(LDivI *ins)
{
    Register remainder = ToRegister(ins->remainder());
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());
    Register output = ToRegister(ins->output());

    MDiv *mir = ins->mir();

    Label done;

    // Handle divide by zero.
// XXX! divwo will detect this for us!
    if (mir->canBeDivideByZero()) {
        masm.and_rc(tempRegister, rhs, rhs);
        if (mir->isTruncated()) {
            // Truncated division by zero is zero (Infinity|0 == 0)
            Label notzero;
            masm.bc(Assembler::NonZero, &notzero);
            masm.xor_(output, output, output);
            masm.b(&done);
            masm.bind(&notzero);
        } else {
            JS_ASSERT(mir->fallible());
            if (!bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
        }
    }

    // Handle an integer overflow exception from -2147483648 / -1.
// XXX! divwo will detect this for us!
    if (mir->canBeNegativeOverflow()) {
        Label notmin;
        masm.x_li32(output, INT32_MIN); // sign extends
        masm.cmpw(lhs, output);
        masm.bc(Assembler::NotEqual, &notmin);
        masm.cmpwi(rhs, -1); // sign extends (hopefully)
        if (mir->isTruncated()) {
            // (-INT32_MIN)|0 == INT32_MIN and INT32_MIN is already in the
            // output register.
            masm.bc(Assembler::Equal, &done);
        } else {
            JS_ASSERT(mir->fallible());
            if (!bailoutIf(Assembler::Equal, ins->snapshot()))
                return false;
        }
        masm.bind(&notmin);
    }

    // Handle negative 0.
    if (!mir->isTruncated() && mir->canBeNegativeZero()) {
        Label nonzero;
        masm.and_rc(tempRegister, lhs, lhs);
        masm.bc(Assembler::NonZero, &nonzero);
        masm.cmpwi(rhs, 0);
        if (!bailoutIf(Assembler::LessThan, ins->snapshot()))
            return false;
        masm.bind(&nonzero);
    }

    // Do the division. TODO: Support divd on G5.
    // Since we don't test for Assembler::Overflow, divw should be fine.
// XXX! We must use divwo, because we want to use it to intercept!
    masm.divw(output, lhs, rhs);

    // XXX. We don't emit anything to the remainder register if isTruncated
    // is true. Maybe we should.

    if (!mir->isTruncated()) {
        // Recover the remainder. Overflow not needed.
        masm.mullw(tempRegister, output, rhs);
        masm.subf(remainder, tempRegister, lhs);
        // If the remainder is > 0, bail out since this must be a double.
        masm.and_rc(tempRegister, remainder, remainder);
        if (!bailoutIf(Assembler::NonZero, ins->snapshot()))
            return false;
    }

    masm.bind(&done);

    return true;
}

bool
CodeGeneratorPPC::visitModPowTwoI(LModPowTwoI *ins)
{
    Register lhs = ToRegister(ins->getOperand(0));
    int32_t shift = ins->shift();

    Label negative, done;
    // Switch based on sign of the lhs.
    // Positive numbers are just a bitmask.
    masm.branchTest32(Assembler::Signed, lhs, lhs, &negative);
    {
        masm.and32(Imm32((1 << shift) - 1), lhs);
        masm.b(&done);
    }
    // Negative numbers need a negate, bitmask, negate
    {
        masm.bind(&negative);
        // visitModI has an overflow check here to catch INT_MIN % -1, but
        // here the rhs is a power of 2, and cannot be -1, so the check need
        // not be generated.
        masm.neg(lhs, lhs);
        masm.and32(Imm32((1 << shift) - 1), lhs);
        masm.neg(lhs, lhs);
        if (!ins->mir()->isTruncated() &&
            !bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
    }
    masm.bind(&done);
    return true;

}

bool
CodeGeneratorPPC::visitModI(LModI *ins)
{
    Register remainder = ToRegister(ins->remainder());
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());
    //Register temp = ToRegister(ins->getTemp(0));

    Label done;

    // Prevent divide by zero.
// XXX! divwo will detect this for us! See BaselineCompiler
    masm.and_rc(tempRegister, rhs, rhs);
    if (ins->mir()->isTruncated()) {
        Label notzero;
        masm.bc(Assembler::NonZero, &notzero);
        masm.xor_(remainder, remainder, remainder); // x mod 0 == 0
        masm.b(&done);
        masm.bind(&notzero);
    } else {
        if (!bailoutIf(Assembler::Zero, ins->snapshot()))
            return false;
    }

    Label negative;

    // Switch based on sign of the lhs.
    masm.branchTest32(Assembler::Signed, lhs, lhs, &negative);
    // If lhs >= 0 then remainder = lhs % rhs. The remainder must be positive.
    {
        // Since lhs >= 0, the sign-extension will be 0.
        // Compute the remainder.
        masm.divw(remainder, lhs, rhs);
        masm.mullw(remainder, remainder, rhs);
        masm.subf(remainder, remainder, lhs);
        masm.b(&done);
    }

    // Otherwise, we must beware of two special cases (see Div above):
    {
        masm.bind(&negative);

        // Prevent an integer overflow exception from -2147483648 % -1.
// XXX! divwo will detect this for us!
        Label notmin;
        masm.x_li32(tempRegister, INT32_MIN);
        masm.cmpw(lhs, tempRegister);
        masm.bc(Assembler::NotEqual, &notmin);
        masm.cmpwi(rhs, -1);
        if (ins->mir()->isTruncated()) {
            // The remainder is 0.
            masm.bc(Assembler::NotEqual, &notmin);
            masm.xor_(remainder, remainder, remainder);
            masm.b(&done);
        } else {
            if (!bailoutIf(Assembler::Equal, ins->snapshot()))
                return false;
        }
        masm.bind(&notmin);

        // Compute the remainder.
        masm.divw(remainder, lhs, rhs);
        masm.mullw(remainder, remainder, rhs);
        masm.subf(remainder, remainder, lhs);

        if (!ins->mir()->isTruncated()) {
            // A remainder of 0 means that the rval must be -0, which is a
            // double.
            masm.and_rc(tempRegister, remainder, remainder);
            if (!bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
        }
    }

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorPPC::visitBitNotI(LBitNotI *ins)
{
    const LAllocation *input = ins->getOperand(0);
    const LDefinition *dest = ins->getDef(0);
    JS_ASSERT(!input->isConstant());

    //masm.notl(ToOperand(input));
    // The equivalent for logical NOT on PPC is neg followed by adding -1.
    // See OPPCC p.539.
// XXX. Why not xori dest, input, -1 and xoris dest, dest, -1?
    masm.neg(ToRegister(dest), ToRegister(input));
    masm.addi(ToRegister(dest), ToRegister(dest), -1);
    return true;
}

bool
CodeGeneratorPPC::visitBitOpI(LBitOpI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);

    switch (ins->bitop()) {
        case JSOP_BITOR:
            if (rhs->isConstant())
                masm.or32(Imm32(ToInt32(rhs)), ToRegister(lhs));
            else
                masm.or_(ToRegister(dest), ToRegister(lhs), ToRegister(rhs));
            break;
        case JSOP_BITXOR:
            if (rhs->isConstant())
                masm.xor32(Imm32(ToInt32(rhs)), ToRegister(lhs));
            else
                masm.xor_(ToRegister(dest), ToRegister(lhs), ToRegister(rhs));
            break;
        case JSOP_BITAND:
            if (rhs->isConstant())
                masm.and32(Imm32(ToInt32(rhs)), ToRegister(lhs));
            else
                masm.and_(ToRegister(dest), ToRegister(lhs), ToRegister(rhs));
            break;
        default:
            JS_NOT_REACHED("unexpected binary opcode");
    }

    return true;
}

bool
CodeGeneratorPPC::visitShiftI(LShiftI *ins)
{
    Register lhs = ToRegister(ins->lhs());
    const LAllocation *rhs = ins->rhs();
    Register dest = ToRegister(ins->output());

    if (rhs->isConstant()) {
        int32_t shift = ToInt32(rhs) & 0x1F;
        // Since we know that the shift will never be > 0x1f, we can use
        // direct instructions rather than checking quantity boilerplate.
        switch (ins->bitop()) {
          case JSOP_LSH:
            if (shift)
                // There's no slwi, but we have a synthetic instruction.
                masm.x_slwi(dest, lhs, shift);
            break;
          case JSOP_RSH:
            if (shift)
                masm.srawi(dest, lhs, shift);
            break;
          case JSOP_URSH:
            if (shift) {
                masm.x_srwi(dest, lhs, shift);
            } else if (ins->mir()->toUrsh()->canOverflow()) {
                // x >>> 0 can overflow.
                masm.and_rc(tempRegister, lhs, lhs);
                if (!bailoutIf(Assembler::Signed, ins->snapshot()))
                    return false;
            }
            break;
          default:
            JS_NOT_REACHED("Unexpected shift op");
        }
    } else {
        // Seems like clamping should be unnecessary, but let's be safe.
        masm.andi_rc(tempRegister, ToRegister(rhs), 31);
        switch (ins->bitop()) {
          case JSOP_LSH:
            masm.slw(dest, lhs, tempRegister);
            break;
          case JSOP_RSH:
            masm.sraw(dest, lhs, tempRegister);
            break;
          case JSOP_URSH:
            masm.srw(dest, lhs, tempRegister);
            if (ins->mir()->toUrsh()->canOverflow()) {
                // x >>> 0 can overflow.
                masm.and_rc(tempRegister, lhs, lhs);
                if (!bailoutIf(Assembler::Signed, ins->snapshot()))
                    return false;
            }
            break;
          default:
            JS_NOT_REACHED("Unexpected shift op");
        }
    }
    return true;
}

bool
CodeGeneratorPPC::visitUrshD(LUrshD *ins)
{
    Register lhs = ToRegister(ins->lhs());
    const LAllocation *rhs = ins->rhs();
    FloatRegister out = ToFloatRegister(ins->output());

    // Use addressTempRegister, since convertUInt32ToDouble uses tempRegister
    // and will assert.
    if (rhs->isConstant()) {
        int32_t shift = ToInt32(rhs) & 0x1F;
        if (shift) {
            masm.srawi(addressTempRegister, lhs, shift);
            masm.convertUInt32ToDouble(addressTempRegister, out);
            return true;
        } else {
            // Save an instruction, ride a cowboy. Wait, what?
            masm.convertUInt32ToDouble(lhs, out);
            return true;
        }
    } 
    masm.andi_rc(tempRegister, ToRegister(rhs), 31); // clamp
    masm.sraw(addressTempRegister, lhs, tempRegister);
    masm.convertUInt32ToDouble(addressTempRegister, out);
    return true;
}

typedef MoveResolver::MoveOperand MoveOperand;

MoveOperand
CodeGeneratorPPC::toMoveOperand(const LAllocation *a) const
{
    if (a->isGeneralReg())
        return MoveOperand(ToRegister(a));
    if (a->isFloatReg())
        return MoveOperand(ToFloatRegister(a));
    return MoveOperand(StackPointer, ToStackOffset(a));
}

bool
CodeGeneratorPPC::visitMoveGroup(LMoveGroup *group)
{
    if (!group->numMoves())
        return true;

    MoveResolver &resolver = masm.moveResolver();

    for (size_t i = 0; i < group->numMoves(); i++) {
        const LMove &move = group->getMove(i);

        const LAllocation *from = move.from();
        const LAllocation *to = move.to();

        // No bogus moves.
        JS_ASSERT(*from != *to);
        JS_ASSERT(!from->isConstant());
        JS_ASSERT(from->isDouble() == to->isDouble());

        MoveResolver::Move::Kind kind = from->isDouble()
                                        ? MoveResolver::Move::DOUBLE
                                        : MoveResolver::Move::GENERAL;

        if (!resolver.addMove(toMoveOperand(from), toMoveOperand(to), kind))
            return false;
    }

    if (!resolver.resolve())
        return false;

    MoveEmitter emitter(masm);
    emitter.emit(resolver);
    emitter.finish();

    return true;
}

class OutOfLineTableSwitch : public OutOfLineCodeBase<CodeGeneratorPPC>
{
    MTableSwitch *mir_;
    CodeLabel jumpLabel_;

    bool accept(CodeGeneratorPPC *codegen) {
        return codegen->visitOutOfLineTableSwitch(this);
    }

  public:
    OutOfLineTableSwitch(MTableSwitch *mir)
      : mir_(mir)
    {}

    MTableSwitch *mir() const {
        return mir_;
    }

    CodeLabel *jumpLabel() {
        return &jumpLabel_;
    }
};

bool
CodeGeneratorPPC::visitOutOfLineTableSwitch(OutOfLineTableSwitch *ool)
{
// XXX
    JS_NOT_REACHED("visitOutOfLineTableSwitch: no writeCodePointer");
    return false;
#if(0)
    MTableSwitch *mir = ool->mir();

    masm.align(sizeof(void*));
    masm.bind(ool->jumpLabel()->src());
    if (!masm.addCodeLabel(*ool->jumpLabel()))
        return false;

    for (size_t i = 0; i < mir->numCases(); i++) {
        LBlock *caseblock = mir->getCase(i)->lir();
        Label *caseheader = caseblock->label();
        uint32_t caseoffset = caseheader->offset();

        // The entries of the jump table need to be absolute addresses and thus
        // must be patched after codegen is finished.
        CodeLabel cl;
        masm.writeCodePointer(cl.dest());
        cl.src()->bind(caseoffset);
        if (!masm.addCodeLabel(cl))
            return false;
    }

    return true;
#endif
}

bool
CodeGeneratorPPC::emitTableSwitchDispatch(MTableSwitch *mir, const Register &index,
                                                const Register &base)
{
// XXX
// I have no idea if this is even what we want; it's just a direct port of
// X86. Maybe ARM's is better?
    Label *defaultcase = mir->getDefault()->lir()->label();

    // Lower value with low value
    if (mir->low() != 0)
        masm.sub32(Imm32(mir->low()), index);

    // Jump to default case if input is out of range
    int32_t cases = mir->numCases();
    masm.cmp32(index, Imm32(cases));
    masm.bc(Assembler::AboveOrEqual, defaultcase);

    // To fill in the CodeLabels for the case entries, we need to first
    // generate the case entries (we don't yet know their offsets in the
    // instruction stream).
    OutOfLineTableSwitch *ool = new OutOfLineTableSwitch(mir);
    if (!addOutOfLineCode(ool))
        return false;

    // Compute the position where a pointer to the right case stands.
/*
This is totally wrong, but I see what it's trying to do.
    masm.mov(ool->jumpLabel()->dest(), base);
    Operand pointer = Operand(base, index, ScalePointer);

    // Jump to the right case
    masm.b(pointer);
*/
    JS_NOT_REACHED("figure out how we want the tableSwitch set up");

    return true;
}

bool
CodeGeneratorPPC::visitMathD(LMathD *math)
{
    FloatRegister lhs = ToFloatRegister(math->lhs());
    FloatRegister rhs = ToFloatRegister(math->rhs());
    FloatRegister out = ToFloatRegister(math->output());

    // Yo dawg, I heard you like to add after you multiply, so I put ...
    // ... wait, this doesn't support that? WHAT? WHAT THE F#$%^!@NO CARRIER

    switch (math->jsop()) {
      case JSOP_ADD:
        masm.fadd(out, lhs, rhs);
        break;
      case JSOP_SUB:
        // This is regular order, not wacky ol' subf on GPRs.
        masm.fsub(out, lhs, rhs);
        break;
      case JSOP_MUL:
        masm.fmul(out, lhs, rhs);
        break;
      case JSOP_DIV:
        masm.fdiv(out, lhs, rhs);
        break;
      default:
        JS_NOT_REACHED("unexpected opcode");
        return false;
    }
    return true;
}

bool
CodeGeneratorPPC::visitFloor(LFloor *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    Register output = ToRegister(lir->output());
    Label bail;

    masm.branchTruncateDouble(input, output, &bail);
    if (!bailoutFrom(&bail, lir->snapshot()))
        return false;
    return true;
}

bool
CodeGeneratorPPC::visitRound(LRound *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    FloatRegister temp = ToFloatRegister(lir->temp());
    Register output = ToRegister(lir->output());

    // Basically add 0.5 to the float, and convert that. This is probably
    // faster than dorking around with setting rounding modes on the FPSCR.
    Label negative, end, bail;

    // Load 0.5 in the temp register.
    static const double PointFive = 0.5;
    masm.loadStaticDouble(&PointFive, temp);

    // Check zero.
    masm.zeroDouble(fpTempRegister);
    // Branch to a slow path for negative inputs. Doesn't catch NaN or -0.
    masm.branchDouble(Assembler::DoubleLessThan, input, fpTempRegister,
        &negative);

    // Bail on negative-zero.
    Assembler::Condition bailCond = masm.testNegativeZero(input, output);
    if (!bailoutIf(bailCond, lir->snapshot()))
        return false;

    // Input is non-negative. Add 0.5 and truncate, rounding down. Note that we
    // have to add the input to the temp register (which contains 0.5) because
    // we're not allowed to modify the input register. We have to grab a
    // temp FPR here; fpTempRegister is used by branchTruncateDouble.
    masm.fadd(temp, temp, input);
    masm.branchTruncateDouble(temp, output, &bail);
    if (!bailoutFrom(&bail, lir->snapshot()))
        return false;
    masm.b(&end);

    masm.bind(&negative);
    // Input is negative, but isn't -0. Now we *do* have to mess around
    // with the FPSCR. TODO: OOL? This really can't be that frequent and I
    // don't like all this branching.

    // Add 0.5.
    masm.fadd(fpTempRegister, input, temp);
    // Set Round To Infinity mode. These are separate dispatch groups.
    masm.mtfsb1(30);
    masm.mtfsb1(31);
    // Round towards infinity.
    masm.fctiw(fpTempRegister, fpTempRegister);
    // Stuff in a temporary frame.
    masm.stfdu(fpTempRegister, stackPointerRegister, -8);
#ifdef _PPC970_
    // G5 and POWER4+ do better if the stfd and the lwz aren't in the
    // same dispatch group. Since stfdu is cracked, only one nop is needed.
    masm.x_nop();
#endif
    // Pull out the lower 32 bits. This is the result.
    masm.lwz(output, stackPointerRegister, 4);
    // Destroy the temporary frame before testing, since we might branch.
    masm.addi(stackPointerRegister, stackPointerRegister, 8);

    // Inline the bailouts from branchTruncateDouble here.
    masm.x_li32(tempRegister, 0x7fffffff);
    masm.cmplw(output, tempRegister);
    if (!bailoutIf(Assembler::Equal, lir->snapshot())) return false;
    masm.x_li32(tempRegister, 0x80000000); // sign extends!
    masm.cmplw(output, tempRegister);
    if (!bailoutIf(Assembler::Equal, lir->snapshot())) return false;

    // If the result is positive zero, then the actual result is -0. Bail.
    masm.and_rc(tempRegister, output, output);
    if (!bailoutIf(Assembler::Zero, lir->snapshot())) return false;

    // Done.
    masm.bind(&end);
    return true;
}

bool
CodeGeneratorPPC::visitGuardShape(LGuardShape *guard)
{
    Register obj = ToRegister(guard->input());
    masm.cmpPtr(Operand(obj, JSObject::offsetOfShape()), ImmGCPtr(guard->mir()->shape()));
    if (!bailoutIf(Assembler::NotEqual, guard->snapshot()))
        return false;
    return true;
}

bool
CodeGeneratorPPC::visitGuardClass(LGuardClass *guard)
{
    Register obj = ToRegister(guard->input());
    Register tmp = ToRegister(guard->tempInt());

    masm.loadPtr(Address(obj, JSObject::offsetOfType()), tmp);
    masm.cmpPtr(Operand(tmp, offsetof(types::TypeObject, clasp)), ImmWord(guard->mir()->getClass()));
    if (!bailoutIf(Assembler::NotEqual, guard->snapshot()))
        return false;
    return true;
}

class OutOfLineTruncate : public OutOfLineCodeBase<CodeGeneratorPPC>
{
    LTruncateDToInt32 *ins_;

  public:
    OutOfLineTruncate(LTruncateDToInt32 *ins)
      : ins_(ins)
    { }

    bool accept(CodeGeneratorPPC *codegen) {
        return codegen->visitOutOfLineTruncate(this);
    }
    LTruncateDToInt32 *ins() const {
        return ins_;
    }
};

bool
CodeGeneratorPPC::visitTruncateDToInt32(LTruncateDToInt32 *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    Register output = ToRegister(ins->output());

    OutOfLineTruncate *ool = new OutOfLineTruncate(ins);
    if (!addOutOfLineCode(ool))
        return false;

    masm.branchTruncateDouble(input, output, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorPPC::visitOutOfLineTruncate(OutOfLineTruncate *ool)
{
    LTruncateDToInt32 *ins = ool->ins();
    FloatRegister input = ToFloatRegister(ins->input());
    FloatRegister temp = ToFloatRegister(ins->tempFloat());
    Register output = ToRegister(ins->output());

    // This is the bailout from the above (when branchTruncateDouble freaked).
    // Try to convert doubles representing integers within 2^32 of a signed
    // integer, by adding/subtracting 2^32 and then trying to convert to int32.
    // This has to be an exact conversion, as otherwise the truncation works
    // incorrectly on the modified value.
    Label fail;

    // Is this NaN? Test for that first.
    masm.zeroDouble(fpTempRegister);
    masm.fcmpu(input, fpTempRegister);
    // If it was zero, we should probably fail too since it shouldn't be here.
    // Also, this lets us use fsel (see below).
    masm.bc(Assembler::DoubleEqualOrUnordered, &fail);

    // Non-zero, non-NaN. Figure out the sign and add the right constant.
    // We can use fsel, because we've already accounted for zero.
    static const double shiftNeg = 4294967296.0;
    masm.loadStaticDouble(&shiftNeg, temp);
    static const double shiftPos = -4294967296.0;
    masm.loadStaticDouble(&shiftPos, fpTempRegister);
    // XXX: all our FPUABC instructions transpose B and C, so this is
    // not canonical operand order. We should fix that sometime.
    masm.fsel(fpTempRegister, input, temp, fpTempRegister); // neg, pos
    masm.fadd(temp, input, fpTempRegister);
    // Now try again.
    masm.branchTruncateDouble(temp, output, &fail);
    masm.jump(ool->rejoin());

    // It still didn't work. Call to the interpreter for help.
    masm.bind(&fail);
    {
        saveVolatile(output);

        masm.setupUnalignedABICall(1, output);
        masm.passABIArg(input);
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, js::ToInt32));
        masm.storeCallResult(output);

        restoreVolatile(output);
    }
    return true;
}

Operand
CodeGeneratorPPC::createArrayElementOperand(Register elements, const LAllocation *index)
{
// XXX
/*
    if (index->isConstant())
        return Operand(elements, ToInt32(index) * sizeof(js::Value));

    return Operand(elements, ToRegister(index), TimesEight);
*/
    JS_NOT_REACHED("createArrayElementOperand NYI");
}
bool
CodeGeneratorPPC::generateInvalidateEpilogue()
{
// XXX
    // Ensure that there is enough space in the buffer for the OsiPoint
    // patching to occur. Otherwise, we could overwrite the invalidation
    // epilogue.
    for (size_t i = 0; i < sizeof(void *); i+= Assembler::nopSize())
        masm.x_nop();

    masm.bind(&invalidate_);

    // Push the return address of the point that we bailed out at onto the stack
// XXX fix this to use mflr etc.
/*
    masm.Push(lr);
*/

    // Push the Ion script onto the stack (when we determine what that pointer is).
    invalidateEpilogueData_ = masm.pushWithPatch(ImmWord(uintptr_t(-1)));
    IonCode *thunk = GetIonContext()->compartment->ionCompartment()->getInvalidationThunk();

    masm.b(thunk);

    // We should never reach this point in JIT code -- the invalidation thunk should
    // pop the invalidated JS frame and return directly to its caller.
    masm.x_trap();
    return true;
}

void
ParallelGetPropertyIC::initializeAddCacheState(LInstruction *ins, AddCacheState *addState)
{
    // Can always use the temp register on PowerPC.
    JS_ASSERT(ins->isGetPropertyCacheV() || ins->isGetPropertyCacheT());
    addState->dispatchScratch = tempRegister;
}

} // namespace jit
} // namespace js
