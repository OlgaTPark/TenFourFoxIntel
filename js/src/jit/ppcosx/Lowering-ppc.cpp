/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/MIR.h"
#include "jit/Lowering.h"
#include "Assembler-ppc.h"
#include "jit/shared/Lowering-shared-inl.h"

using namespace js;
using namespace js::jit;

bool
LIRGeneratorPPC::useBox(LInstruction *lir, size_t n, MDefinition *mir,
                        LUse::Policy policy, bool useAtStart)
{
    JS_ASSERT(mir->type() == MIRType_Value);
    if (!ensureDefined(mir))
        return false;
    lir->setOperand(n, LUse(mir->virtualRegister(), policy, useAtStart));
    lir->setOperand(n + 1, LUse(VirtualRegisterOfPayload(mir), policy, useAtStart));
    return true;
}

bool
LIRGeneratorPPC::useBoxFixed(LInstruction *lir, size_t n, MDefinition *mir, Register reg1,
                             Register reg2)
{
    JS_ASSERT(mir->type() == MIRType_Value);
    JS_ASSERT(reg1 != reg2);

    if (!ensureDefined(mir))
        return false;
    lir->setOperand(n, LUse(reg1, mir->virtualRegister()));
    lir->setOperand(n + 1, LUse(reg2, VirtualRegisterOfPayload(mir)));
    return true;
}

bool
LIRGeneratorPPC::lowerConstantDouble(double d, MInstruction *mir)
{
    return define(new(alloc()) LDouble(d), mir);
}

#if(0) // see Lowering-ppc.h
bool
LIRGeneratorPPC::lowerConstantFloat32(float f, MInstruction *mir)
{
    return define(new(alloc()) LFloat32(f), mir);
}
#endif

LAllocation
LIRGeneratorPPC::useByteOpRegister(MDefinition *mir)
{
    return useRegister(mir);
}

LAllocation
LIRGeneratorPPC::useByteOpRegisterOrNonDoubleConstant(MDefinition *mir)
{
    return useRegisterOrNonDoubleConstant(mir);
}

bool
LIRGeneratorPPC::lowerForBitAndAndBranch(LBitAndAndBranch *baab, MInstruction *mir,
                                         MDefinition *lhs, MDefinition *rhs)
{
    baab->setOperand(0, useRegister(lhs));
    baab->setOperand(1, useRegisterOrConstant(rhs));
    return add(baab, mir);
}

bool
LIRGeneratorPPC::visitConstant(MConstant *ins)
{
    if (ins->type() == MIRType_Double)
        return lowerConstantDouble(ins->value().toDouble(), ins);

/*
NYI. ARM doesn't seem to implement this yet.
    if (ins->type() == MIRType_Float32)
        return lowerConstantFloat32(ins->value().toDouble(), ins);
*/

    // Emit non-double constants at their uses.
    if (ins->canEmitAtUses())
        return emitAtUses(ins);

    return LIRGeneratorShared::visitConstant(ins);
}

bool
LIRGeneratorPPC::visitBox(MBox *box)
{
    MDefinition *inner = box->getOperand(0);

    // If the box wrapped a double, it needs a new register.
    if (IsFloatingPointType(inner->type()))
        return defineBox(new(alloc())
                     LBoxFloatingPoint(useRegisterAtStart(inner),
                                               tempCopy(inner, 0),
                                               inner->type()), box);

    if (box->canEmitAtUses())
        return emitAtUses(box);

    if (inner->isConstant())
        return defineBox(new(alloc())
            LValue(inner->toConstant()->value()), box);

    LBox *lir = new(alloc()) LBox(use(inner), inner->type());

    // Otherwise, we should not define a new register for the payload portion
    // of the output, so bypass defineBox().
    uint32_t vreg = getVirtualRegister();
    if (vreg >= MAX_VIRTUAL_REGISTERS)
        return false;

    // Note that because we're using PASSTHROUGH, we do not change the type of
    // the definition. We also do not define the first output as "TYPE",
    // because it has no corresponding payload at (vreg + 1). Also note that
    // although we copy the input's original type for the payload half of the
    // definition, this is only for clarity. PASSTHROUGH definitions are
    // ignored.
    lir->setDef(0, LDefinition(vreg, LDefinition::GENERAL));
    lir->setDef(1, LDefinition(inner->virtualRegister(), LDefinition::TypeFrom(inner->type()),
                               LDefinition::PASSTHROUGH));
    box->setVirtualRegister(vreg);
    return add(lir);
}

bool
LIRGeneratorPPC::visitUnbox(MUnbox *unbox)
{
    // An unbox on PPC reads in a type tag (either in memory or a register) and
    // a payload. Unlike most instructions consuming a box, we ask for the type
    // second, so that the result can reuse the first input.
    MDefinition *inner = unbox->getOperand(0);

    if (!ensureDefined(inner))
        return false;

    if (IsFloatingPointType(unbox->type())) {
        LUnboxFloatingPoint *lir = new(alloc())
             LUnboxFloatingPoint(unbox->type());
        if (unbox->fallible() && !assignSnapshot(lir, unbox->bailoutKind()))
            return false;
        if (!useBox(lir, LUnboxFloatingPoint::Input, inner))
            return false;
        return define(lir, unbox);
    }

    // Swap the order we use the box pieces to reuse the payload register.
    LUnbox *lir = new(alloc()) LUnbox;
    lir->setOperand(0, usePayloadInRegisterAtStart(inner));
    lir->setOperand(1, useType(inner, LUse::REGISTER));

    if (unbox->fallible() && !assignSnapshot(lir, unbox->bailoutKind()))
        return false;

    // Note that PASSTHROUGH here is illegal, since types and payloads form two
    // separate intervals. If the type becomes dead before the payload, it
    // could be used as a Value without the type being recoverable. Unbox's
    // purpose is to eagerly kill the definition of a type tag, so keeping both
    // alive (for the purpose of gcmaps) is unappealing. Instead, we create a
    // new virtual register.
    return defineReuseInput(lir, unbox, 0);
}

bool
LIRGeneratorPPC::visitReturn(MReturn *ret)
{
    MDefinition *opd = ret->getOperand(0);
    JS_ASSERT(opd->type() == MIRType_Value);

    LReturn *ins = new(alloc()) LReturn;
    ins->setOperand(0, LUse(JSReturnReg_Type));
    ins->setOperand(1, LUse(JSReturnReg_Data));
    return fillBoxUses(ins, 0, opd) && add(ins);
}

// ops of: x = !y (neg, etc.)
bool
LIRGeneratorPPC::lowerForALU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir, MDefinition *input)
{
    ins->setOperand(0, useRegister(input));
    return define(ins, mir,
                  LDefinition(LDefinition::TypeFrom(mir->type()), LDefinition::DEFAULT));
}

// most other ops: z = x+y (add, subf, etc.)
bool
LIRGeneratorPPC::lowerForALU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegister(lhs));
    ins->setOperand(1, useRegisterOrConstant(rhs));
    return define(ins, mir,
                  LDefinition(LDefinition::TypeFrom(mir->type()), LDefinition::DEFAULT));
}

// fneg, etc.
bool
LIRGeneratorPPC::lowerForFPU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir, MDefinition *input)
{
    ins->setOperand(0, useRegister(input));
    return define(ins, mir,
                  LDefinition(LDefinition::TypeFrom(mir->type()), LDefinition::DEFAULT));

}

// fadd
bool
LIRGeneratorPPC::lowerForFPU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegister(lhs));
    ins->setOperand(1, useRegister(rhs));
    return define(ins, mir,
                  LDefinition(LDefinition::TypeFrom(mir->type()), LDefinition::DEFAULT));
}

// TODO: fmadd, fmnadd

bool
LIRGeneratorPPC::lowerTruncateDToInt32(MTruncateToInt32 *ins)
{
    MDefinition *opd = ins->input();
    JS_ASSERT(opd->type() == MIRType_Double);

    return define(new(alloc())
        LTruncateDToInt32(useRegister(opd), LDefinition::BogusTemp()), ins);
}

bool
LIRGeneratorPPC::defineUntypedPhi(MPhi *phi, size_t lirIndex)
{
    LPhi *type = current->getPhi(lirIndex + VREG_TYPE_OFFSET);
    LPhi *payload = current->getPhi(lirIndex + VREG_DATA_OFFSET);

    uint32_t typeVreg = getVirtualRegister();
    if (typeVreg >= MAX_VIRTUAL_REGISTERS)
        return false;

    phi->setVirtualRegister(typeVreg);

    uint32_t payloadVreg = getVirtualRegister();
    if (payloadVreg >= MAX_VIRTUAL_REGISTERS)
        return false;
    JS_ASSERT(typeVreg + 1 == payloadVreg);

    type->setDef(0, LDefinition(typeVreg, LDefinition::TYPE));
    payload->setDef(0, LDefinition(payloadVreg, LDefinition::PAYLOAD));
    annotate(type);
    annotate(payload);
    return true;
}

void
LIRGeneratorPPC::lowerUntypedPhiInput(MPhi *phi, uint32_t inputPosition, LBlock *block, size_t lirIndex)
{
    // oh god, what is this code?
    // I didn't write that comment. I have no idea either, though -- Cameron
    MDefinition *operand = phi->getOperand(inputPosition);

    // Note that these are endian-independent. We fix this up on the backend.
    LPhi *type = block->getPhi(lirIndex + VREG_TYPE_OFFSET);
    LPhi *payload = block->getPhi(lirIndex + VREG_DATA_OFFSET);

    type->setOperand(inputPosition, LUse(operand->virtualRegister() + VREG_TYPE_OFFSET, LUse::ANY));
    payload->setOperand(inputPosition, LUse(VirtualRegisterOfPayload(operand), LUse::ANY));
}

bool
LIRGeneratorPPC::lowerForShift(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegister(lhs));
    ins->setOperand(1, useRegisterOrConstant(rhs));
    return define(ins, mir);
}

bool
LIRGeneratorPPC::lowerDivI(MDiv *div)
{
// XXX: Get constant divisions working
#if(0)
    if (div->isUnsigned())
        return lowerUDiv(div);
    
    // Division instructions are slow. Division by constant denominators can be
    // rewritten to use other instructions.
    if (div->rhs()->isConstant()) {
        int32_t rhs = div->rhs()->toConstant()->value().toInt32();
        // Check for division by a positive power of two, which is an easy and
        // important case to optimize. Note that other optimizations are also
        // possible; division by negative powers of two can be optimized in a
        // similar manner as positive powers of two, and division by other
        // constants can be optimized by a reciprocal multiplication technique.
        // (And then there's PPC frsp, but we can think about that later.)
        int32_t shift = FloorLog2(rhs);
        if (rhs > 0 && 1 << shift == rhs) {
            LDivPowTwoI *lir = new(alloc())
                     LDivPowTwoI(useRegisterAtStart(div->lhs()),
                shift);
            if (div->fallible() && !assignSnapshot(lir))
                return false;
            return define(lir, div);
        }
    }
#endif

    LDivI *lir = new(alloc())
             LDivI(useRegister(div->lhs()), useRegister(div->rhs()),
        temp());
    if (div->fallible() && !assignSnapshot(lir))
        return false;
    return define(lir, div);
}

bool
LIRGeneratorPPC::lowerMulI(MMul *mul, MDefinition *lhs, MDefinition *rhs)
{
    LMulI *lir = new(alloc()) LMulI;
    if (mul->fallible() && !assignSnapshot(lir))
        return false;
    return lowerForALU(lir, mul, lhs, rhs);
}

bool
LIRGeneratorPPC::lowerModI(MMod *mod)
{
// XXX: get constant mods working a la div above
#if(0)
    if (mod->rhs()->isConstant()) {
        int32_t rhs = mod->rhs()->toConstant()->value().toInt32();
        int32_t shift;
        JS_FLOOR_LOG2(shift, rhs);
        if (1 << shift == rhs) {
            LModPowTwoI *lir = new(alloc())
                 LModPowTwoI(useRegister(mod->lhs()), shift);
            return (assignSnapshot(lir) && define(lir, mod));
        } else if (shift < 31 && (1 << (shift+1)) - 1 == rhs) {
            LModMaskI *lir = new(alloc())
                 LModMaskI(useRegister(mod->lhs()),
                    temp(LDefinition::GENERAL), shift+1);
            return (assignSnapshot(lir) && define(lir, mod));
        }
    }

#endif
    LModI *lir = new(alloc())
             LModI(useRegister(mod->lhs()), useRegister(mod->rhs()),
        temp());
    if (mod->fallible() && !assignSnapshot(lir))
        return false;
    return define(lir, mod);
}

bool
LIRGeneratorPPC::visitPowHalf(MPowHalf *ins)
{
    MDefinition *input = ins->input();
    JS_ASSERT(input->type() == MIRType_Double);
    LPowHalfD *lir = new(alloc()) LPowHalfD(useRegisterAtStart(input));
    return defineReuseInput(lir, ins, 0);
}

LTableSwitch *
LIRGeneratorPPC::newLTableSwitch(const LAllocation &in, const LDefinition &inputCopy,
                                       MTableSwitch *tableswitch)
{
    return new(alloc()) LTableSwitch(in, inputCopy, tableswitch);
}

LTableSwitchV *
LIRGeneratorPPC::newLTableSwitchV(MTableSwitch *tableswitch)
{
    return new(alloc()) LTableSwitchV(temp(), tempDouble(), tableswitch);
}

LGetPropertyCacheT *
LIRGeneratorPPC::newLGetPropertyCacheT(MGetPropertyCache *ins)
{       
    return new(alloc())
        LGetPropertyCacheT(useRegister(ins->object()),
            LDefinition::BogusTemp());
}

LGetElementCacheT *
LIRGeneratorPPC::newLGetElementCacheT(MGetElementCache *ins)
{
    return new(alloc()) LGetElementCacheT(useRegister(ins->object()),
                                 useRegister(ins->index()),
                                 LDefinition::BogusTemp());
}

bool
LIRGeneratorPPC::visitGuardShape(MGuardShape *ins)
{
    JS_ASSERT(ins->obj()->type() == MIRType_Object);

    LDefinition tempObj = temp(LDefinition::OBJECT);
    LGuardShape *guard = new(alloc())
         LGuardShape(useRegister(ins->obj()), tempObj);
    if (!assignSnapshot(guard, ins->bailoutKind()))
        return false;
    if (!add(guard, ins))
        return false;
    return redefine(ins, ins->obj());
}

bool
LIRGeneratorPPC::visitGuardObjectType(MGuardObjectType *ins)
{
    JS_ASSERT(ins->obj()->type() == MIRType_Object);

    LDefinition tempObj = temp(LDefinition::OBJECT);
    LGuardObjectType *guard = new(alloc())
         LGuardObjectType(useRegister(ins->obj()), tempObj);
    if (!assignSnapshot(guard))
        return false;
    if (!add(guard, ins))
        return false;
    return redefine(ins, ins->obj());
}

bool
LIRGeneratorPPC::visitStoreTypedArrayElement(MStoreTypedArrayElement *ins)
{
    JS_ASSERT(ins->elements()->type() == MIRType_Elements);
    JS_ASSERT(ins->index()->type() == MIRType_Int32);

    if (ins->isFloatArray())
        JS_ASSERT(ins->value()->type() == MIRType_Double);
    else
        JS_ASSERT(ins->value()->type() == MIRType_Int32);

    LUse elements = useRegister(ins->elements());
    LAllocation index = useRegisterOrConstant(ins->index());
    LAllocation value = useRegisterOrNonDoubleConstant(ins->value());
    return add(new(alloc())
         LStoreTypedArrayElement(elements, index, value), ins);
}

bool
LIRGeneratorPPC::visitInterruptCheck(MInterruptCheck *ins)
{
    LInterruptCheck *lir = new(alloc()) LInterruptCheck();
    if (!add(lir))
        return false;
    if (!assignSafepoint(lir, ins))
        return false;
    return true;
}

bool
LIRGeneratorPPC::lowerUrshD(MUrsh *mir)
{
    MDefinition *lhs = mir->lhs();
    MDefinition *rhs = mir->rhs();

    JS_ASSERT(lhs->type() == MIRType_Int32);
    JS_ASSERT(rhs->type() == MIRType_Int32);

    LUrshD *lir = new(alloc())
         LUrshD(useRegister(lhs), useRegisterOrConstant(rhs), temp());
    return define(lir, mir);
}

