/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_lir_ppc_h__
#define jsion_lir_ppc_h__

namespace js {
namespace jit {

// LInstructionHelper<defs, operands, temps>

class LBox : public LInstructionHelper<2, 1, 0>
{
    MIRType type_;

  public:
    LIR_HEADER(Box);

    LBox(const LAllocation &in_payload, MIRType type)
      : type_(type)
    {
        setOperand(0, in_payload);
    }

    MIRType type() const {
        return type_;
    }
};

// Replaces LBoxDouble
class LBoxFloatingPoint : public LInstructionHelper<2, 1, 1>
{
    MIRType type_;

  public:
    LIR_HEADER(BoxFloatingPoint);

    LBoxFloatingPoint(const LAllocation &in, const LDefinition &temp, MIRType type)
      : type_(type)
    {
        setOperand(0, in);
        setTemp(0, temp);
    }

    MIRType type() const {
        return type_;
    }
};

class LUnbox : public LInstructionHelper<1, 2, 0>
{
  public:
    LIR_HEADER(Unbox);

    MUnbox *mir() const {
        return mir_->toUnbox();
    }
    const LAllocation *payload() {
        return getOperand(0);
    }
    const LAllocation *type() {
        return getOperand(1);
    }
};

class LUnboxFloatingPoint : public LInstructionHelper<1, 2, 0>
{
    MIRType type_;

  public:
    LIR_HEADER(UnboxFloatingPoint);

    static const size_t Input = 0;

    LUnboxFloatingPoint(MIRType type)
      : type_(type)
    { }

    MUnbox *mir() const {
        return mir_->toUnbox();
    }

    MIRType type() const {
        return type_;
    }
};

// LDouble is now in LIR-Common.h.

class LDivI : public LBinaryMath<1>
{
  public:
    LIR_HEADER(DivI)

    LDivI(const LAllocation &lhs, const LAllocation &rhs, const LDefinition &temp) {
        setOperand(0, lhs);
        setOperand(1, rhs);
        setTemp(0, temp);
    }

    const LDefinition *remainder() {
        return getTemp(0);
    }
    MDiv *mir() const {
        return mir_->toDiv();
    }
};


class LModI : public LBinaryMath<1>
{
  public:
    LIR_HEADER(ModI)

    LModI(const LAllocation &lhs, const LAllocation &rhs, const LDefinition &temp) {
        setOperand(0, lhs);
        setOperand(1, rhs);
        setTemp(0, temp);
    }

    const LDefinition *remainder() {
        return getDef(0);
    }
    MMod *mir() const {
        return mir_->toMod();
    }
};

class LModPowTwoI : public LInstructionHelper<1,1,0>
{
    const int32_t shift_;

  public:
    LIR_HEADER(ModPowTwoI)

    LModPowTwoI(const LAllocation &lhs, int32_t shift)
      : shift_(shift)
    {
        setOperand(0, lhs);
    }

    int32_t shift() const {
        return shift_;
    }
    const LDefinition *remainder() {
        return getDef(0);
    }
    MMod *mir() const {
        return mir_->toMod();
    }
};


class LModMaskI : public LInstructionHelper<1, 1, 1>
{
    const int32_t shift_;

  public:
    LIR_HEADER(ModMaskI);

    LModMaskI(const LAllocation &lhs, const LDefinition &temp1, int32_t shift)
      : shift_(shift)
    {
        setOperand(0, lhs);
        setTemp(0, temp1);
    }

    int32_t shift() const {
        return shift_;
    }
};

class LPowHalfD : public LInstructionHelper<1, 1, 0>
{
  public:
    LIR_HEADER(PowHalfD);
    LPowHalfD(const LAllocation &input) {
        setOperand(0, input);
    }

    const LAllocation *input() {
        return getOperand(0);
    }
    const LDefinition *output() {
        return getDef(0);
    }
};

// Takes a tableswitch with an integer to decide
class LTableSwitch : public LInstructionHelper<0, 1, 1>
{
  public:
    LIR_HEADER(TableSwitch);

    LTableSwitch(const LAllocation &in, const LDefinition &inputCopy, MTableSwitch *ins) {
        setOperand(0, in);
        setTemp(0, inputCopy);
        setMir(ins);
    }

    MTableSwitch *mir() const {
        return mir_->toTableSwitch();
    }

    const LAllocation *index() {
        return getOperand(0);
    }
    const LDefinition *tempInt() {
        return getTemp(0);
    }
    // This is added to share the same CodeGenerator prefixes.
    const LDefinition *tempPointer() {
        return nullptr;
    }
};

// Takes a tableswitch with an integer to decide
class LTableSwitchV : public LInstructionHelper<0, BOX_PIECES, 2>
{
  public:
    LIR_HEADER(TableSwitchV);

    LTableSwitchV(const LDefinition &inputCopy, const LDefinition &floatCopy,
                  MTableSwitch *ins)
    {
        setTemp(0, inputCopy);
        setTemp(1, floatCopy);
        setMir(ins);
    }

    MTableSwitch *mir() const {
        return mir_->toTableSwitch();
    }

    static const size_t InputValue = 0;

    const LDefinition *tempInt() {
        return getTemp(0);
    }
    const LDefinition *tempFloat() {
        return getTemp(1);
    }
    const LDefinition *tempPointer() {
        return nullptr;
    }
};

// Guard against an object's shape.
class LGuardShape : public LInstructionHelper<0, 1, 1>
{
  public:
    LIR_HEADER(GuardShape);

    LGuardShape(const LAllocation &in, const LDefinition &temp) {
        setOperand(0, in);
        setTemp(0, temp);
    }
    const MGuardShape *mir() const {
        return mir_->toGuardShape();
    }
    const LDefinition *tempInt() {
        return getTemp(0);
    }
};

class LGuardObjectType : public LInstructionHelper<0, 1, 1>
{
  public:
    LIR_HEADER(GuardObjectType);

    LGuardObjectType(const LAllocation &in, const LDefinition &temp) {
        setOperand(0, in);
        setTemp(0, temp);
    }
    const MGuardObjectType *mir() const {
        return mir_->toGuardObjectType();
    }
    const LDefinition *tempInt() {
        return getTemp(0);
    }
};

class LInterruptCheck : public LInstructionHelper<0, 0, 0>
{
  public:
    LIR_HEADER(InterruptCheck);
};

class LMulI : public LBinaryMath<0>
{
  public:
    LIR_HEADER(MulI);

    MMul *mir() {
        return mir_->toMul();
    }

    const LAllocation *lhsCopy() {
        return this->getOperand(2);
    }
};

} // namespace jit
} // namespace js

#endif // jsion_lir_ppc_h__
