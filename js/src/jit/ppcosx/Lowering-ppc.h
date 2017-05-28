/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_ion_lowering_ppc_h__
#define jsion_ion_lowering_ppc_h__

#include "jit/shared/Lowering-shared.h"

namespace js {
namespace jit {

class LIRGeneratorPPC : public LIRGeneratorShared
{
  public:
    LIRGeneratorPPC(MIRGenerator *gen, MIRGraph &graph, LIRGraph &lirGraph)
      : LIRGeneratorShared(gen, graph, lirGraph)
    { }

  protected:
    // Adds a box input to an instruction, setting operand |n| to the type and
    // |n+1| to the payload.
    bool useBox(LInstruction *lir, size_t n, MDefinition *mir,
                LUse::Policy policy = LUse::REGISTER, bool useAtStart = false);
    bool useBoxFixed(LInstruction *lir, size_t n, MDefinition *mir, Register reg1, Register reg2);

    bool needTempForPostBarrier() { return false; }
    inline LDefinition tempToUnbox() {
        return LDefinition::BogusTemp();
    }
    LDefinition tempForDispatchCache(MIRType outputType = MIRType_None) {
        return LDefinition::BogusTemp();
    }

    void lowerUntypedPhiInput(MPhi *phi, uint32_t inputPosition, LBlock *block, size_t lirIndex);
    bool defineUntypedPhi(MPhi *phi, size_t lirIndex);
    bool lowerForShift(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs, 
                       MDefinition *rhs);
    bool lowerUrshD(MUrsh *mir);

    bool lowerForALU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir,
                     MDefinition *input);
    bool lowerForALU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir,
                     MDefinition *lhs, MDefinition *rhs);

    bool lowerForFPU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir,
                     MDefinition *src);
    bool lowerForFPU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir,
                     MDefinition *lhs, MDefinition *rhs);

    bool lowerTruncateDToInt32(MTruncateToInt32 *ins);

    bool lowerConstantDouble(double d, MInstruction *ins);
    bool lowerDivI(MDiv *div);
    bool lowerModI(MMod *mod);
    bool lowerMulI(MMul *mul, MDefinition *lhs, MDefinition *rhs);
    bool visitPowHalf(MPowHalf *ins);

    LTableSwitch *newLTableSwitch(const LAllocation &in, const LDefinition &inputCopy,
                                  MTableSwitch *ins);
    LTableSwitchV *newLTableSwitchV(MTableSwitch *ins);
    LGetPropertyCacheT *newLGetPropertyCacheT(MGetPropertyCache *ins);
    LGetElementCacheT *newLGetElementCacheT(MGetElementCache *ins);
    bool lowerConstantFloat32(float d, MInstruction *ins) {
        MOZ_ASSUME_UNREACHABLE("NYI");
    }
    bool lowerTruncateFToInt32(MTruncateToInt32 *ins) {
        MOZ_ASSUME_UNREACHABLE("NYI");
    }
    LAllocation useByteOpRegister(MDefinition *mir);
    LAllocation useByteOpRegisterOrNonDoubleConstant(MDefinition *mir);
    bool lowerForBitAndAndBranch(LBitAndAndBranch *baab, MInstruction *mir,
                                         MDefinition *lhs, MDefinition *rhs);

  public:
    bool visitConstant(MConstant *ins);
    bool visitBox(MBox *box);
    bool visitUnbox(MUnbox *unbox);
    bool visitReturn(MReturn *ret);
    bool lowerPhi(MPhi *phi);
    bool visitGuardShape(MGuardShape *ins);
    bool visitGuardObjectType(MGuardObjectType *ins);
    bool visitStoreTypedArrayElement(MStoreTypedArrayElement *ins);
    bool visitInterruptCheck(MInterruptCheck *ins);
};

typedef LIRGeneratorPPC LIRGeneratorSpecific;

} // namespace jit
} // namespace js

#endif // jsion_ion_lowering_ppc_h__

