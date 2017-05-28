/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/BaselineJIT.h"
#include "jit/BaselineIC.h"
#include "jit/BaselineCompiler.h"
#include "jit/BaselineHelpers.h"
#include "jit/IonLinker.h"

using namespace js;
using namespace js::jit;

namespace js {
namespace jit {

// ICCompare_Int32

bool
ICCompare_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    // Guard that R0 is an integer and R1 is an integer.
    Label failure;
/*
    masm.branchTestInt32(Assembler::NotEqual, R0, &failure);
    masm.branchTestInt32(Assembler::NotEqual, R1, &failure);
*/
    // Oh, we can eliminate one of those branches ... and, we can statically
    // predict it too. This should fit into a single G5 dispatch group.
    // We can't use xori because tags may be negative and xori zero-extends.
    masm.x_li32(r0, JSVAL_TAG_INT32);
    masm.xor_(r12, R0.typeReg(), r0);
    masm.xor_(r0,  R1.typeReg(), r0);
    masm.or_rc(r0, r0, r12); // r0 == R0.typeReg() == R1.typeReg()
    masm.bc(Assembler::NonZero, &failure);

    // Compare payload regs of R0 and R1, moving 1 to R0 if they are the
    // same and 0 if they are not.
    Assembler::Condition cond = JSOpToCondition(op, /* signed = */true);
    Register dest = R0.payloadReg();

    // Fast paths, PowerPC Compiler Writer's Guide, Appendix D et al. These
    // avoid use of CR, which could be slow (on G5, mfcr is microcoded).
    // Due to possibly constrained register usage, we don't use the optimals.
    // TODO: Add subfe and subfze to support unsigned Above/Below, though
    // these are probably used a lot less.
    if (cond == Assembler::Equal) {
        masm.subf(r0, R1.payloadReg(), R0.payloadReg()); // p.141
        masm.cntlzw(r0, r0);
        masm.x_srwi(dest, r0, 5);
    } else if (cond == Assembler::NotEqual) {
        masm.subf(r0, R0.payloadReg(), R1.payloadReg()); // p.58
        masm.subf(r12, R1.payloadReg(), R0.payloadReg());
        masm.or_(r12, r12, r0);
        masm.rlwinm(dest, r12, 1, 31, 31);
    } else if (cond == Assembler::LessThan) { // SIGNED
        masm.subfc(r0, R1.payloadReg(), R0.payloadReg()); // p.200
        masm.eqv(r12,  R1.payloadReg(), R0.payloadReg());
        masm.x_srwi(R0.payloadReg(), r12, 31);
        masm.addze(r12, R0.payloadReg());
        masm.rlwinm(R0.payloadReg(), r12, 0, 31, 31);
    } else if (cond == Assembler::GreaterThan) { // SIGNED
        // Reverse (not inverse).
        masm.subfc(r0, R0.payloadReg(), R1.payloadReg());
        masm.eqv(r12,  R0.payloadReg(), R1.payloadReg());
        masm.x_srwi(R0.payloadReg(), r12, 31);
        masm.addze(r12, R0.payloadReg());
        masm.rlwinm(R0.payloadReg(), r12, 0, 31, 31);
    } else if (cond == Assembler::LessThanOrEqual) { // SIGNED
        masm.x_srwi(r0, R0.payloadReg(), 31); // p.200
        masm.srawi(r12, R1.payloadReg(), 31);
        masm.subfc(R0.payloadReg(), R0.payloadReg(), R1.payloadReg());
        masm.adde(R0.payloadReg(), r12, r0);
    } else if (cond == Assembler::GreaterThanOrEqual) { // SIGNED
        // Reverse (not inverse).
        masm.x_srwi(r0, R1.payloadReg(), 31);
        masm.srawi(r12, R0.payloadReg(), 31);
        masm.subfc(R0.payloadReg(), R1.payloadReg(), R0.payloadReg());
        masm.adde(R0.payloadReg(), r12, r0);
    } else {
        // Use the emitSet branched version as a slow path for any condition.
        // Since these are mostly unsigned conditions, use cmplw.
        masm.cmplw(R0.payloadReg(), R1.payloadReg());
        masm.emitSet(cond, dest);
    }

    // Result is implicitly boxed already.
    masm.tagValue(JSVAL_TYPE_BOOLEAN, R0.payloadReg(), R0);
    EmitReturnFromIC(masm);

    // In the failure case, jump to the next stub.
    masm.bind(&failure);
    EmitStubGuardFailure(masm);

    return true;
}

static uint8_t crBit(CRegisterID cr, Assembler::DoubleCondition cond)
{
    return (cr << 2) + ((cond & 0xf0) >> 4);
}

bool
ICCompare_Double::Compiler::generateStubCode(MacroAssembler &masm)
{
    bool isUnordered;
    Label failure, isNaN;
    masm.ensureDouble(R0, FloatReg0, &failure);
    masm.ensureDouble(R1, FloatReg1, &failure);

    Register dest = R0.scratchReg();

    // We can't avoid mfcr here, I don't think (the FXU tricks we use
    // for GPRs will not work for FPRs, natch), so just do the compare now.
    masm.fcmpu(FloatReg0, FloatReg1);
    Assembler::DoubleCondition doubleCond = JSOpToDoubleCondition(op);

    // Check for simple ordered/unordered before checking synthetic codes.
    if (doubleCond == Assembler::DoubleUnordered) {
        masm.mfcr(r0);
        masm.rlwinm(dest, r0, 4, 31, 31); // get CR0[FU]. FU! FUUUUUUUUUU-
    } else if (doubleCond == Assembler::DoubleOrdered) {
        // Same, but with the xori (flip the bit).
        masm.mfcr(r0);
        masm.rlwinm(r0, r0, 4, 31, 31);
        masm.xori(dest, r0, 1); // flip sign into the payload reg
    } else {
        // This is a synthetic condition code.
        // Extract it into the condition and whether it's "OrUnordered."
        const uint8_t fuBit = crBit(cr0, Assembler::DoubleUnordered);
        const uint8_t condBit = crBit(cr0, doubleCond);
        isUnordered = (doubleCond & Assembler::DoubleUnordered) ? true : false;
        Assembler::DoubleCondition baseDCond = (isUnordered) ?
            (doubleCond & ~Assembler::DoubleUnordered) : doubleCond;

        // Fast paths.
        if (baseDCond == Assembler::DoubleEqual) {
            if (isUnordered) masm.cror(condBit, fuBit, condBit);
            masm.mfcr(r0); // warning: microcoded on G5
            masm.rlwinm(dest, r0, 3, 31, 31); // get CR0[FE]
        } else if (baseDCond == Assembler::DoubleNotEqual) {
            // Same, but inverted with the xori (flip the bit).
            if (isUnordered)
                // Flip FU and AND it with condBit.
                masm.crandc(condBit, condBit, fuBit);
            masm.mfcr(r0); // warning: microcoded on G5
            masm.rlwinm(r0, r0, 3, 31, 31); // get CR0[FE]
            masm.xori(dest, r0, 1); // flip sign into the payload reg
        } else if (baseDCond == Assembler::DoubleGreaterThan) {
            if (isUnordered) masm.cror(condBit, fuBit, condBit);
            masm.mfcr(r0);
            masm.rlwinm(dest, r0, 2, 31, 31); // get CR0[FG]
        } else if (baseDCond == Assembler::DoubleLessThanOrEqual) {
            // Inverse (not reverse).
            if (isUnordered) masm.crandc(condBit, condBit, fuBit);
            masm.mfcr(r0);
            masm.rlwinm(r0, r0, 2, 31, 31); // get CR0[FG]
            masm.xori(dest, r0, 1); // flip sign into the payload reg
        } else if (baseDCond == Assembler::DoubleLessThan) {
            if (isUnordered) masm.cror(condBit, fuBit, condBit);
            masm.mfcr(r0);
            masm.rlwinm(dest, r0, 1, 31, 31); // get CR0[FL]
        } else if (baseDCond == Assembler::DoubleGreaterThanOrEqual) {
            // Inverse (not reverse).
            if (isUnordered) masm.crandc(condBit, condBit, fuBit);
            masm.mfcr(r0);
            masm.rlwinm(r0, r0, 1, 31, 31); // get CR0[FL]
            masm.xori(dest, r0, 1); // flip sign into the payload reg
        } else {
            // Use the emitSet branched version to cover other things.
            masm.emitSet(doubleCond, dest);
        }
    }
    masm.tagValue(JSVAL_TYPE_BOOLEAN, dest, R0);
    EmitReturnFromIC(masm);

    // Failure case - jump to next stub
    masm.bind(&failure);
    EmitStubGuardFailure(masm);
    return true;
}

// ICBinaryArith_Int32

bool
ICBinaryArith_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    // Guard that R0 is an integer and R1 is an integer.
    Label failure;
/*
    masm.branchTestInt32(Assembler::NotEqual, R0, &failure);
    masm.branchTestInt32(Assembler::NotEqual, R1, &failure);
*/
    masm.x_li32(r0, JSVAL_TAG_INT32);
    masm.xor_(r12, R0.typeReg(), r0);
    masm.xor_(r0,  R1.typeReg(), r0);
    masm.or_rc(r0, r0, r12); // r0 == R0.typeReg() == R1.typeReg()
    masm.bc(Assembler::NonZero, &failure);

    Register scratchReg = R2.payloadReg();

    Label maybeNegZero, revertRegister;
    switch(op_) {
      // Because this can overflow, we must use the overflow-enabled math
      // instructions. (Hi, TraceMonkey!)
      case JSOP_ADD:
        masm.addo(scratchReg, R0.payloadReg(), R1.payloadReg());

        // Just jump to failure on overflow.  R0 and R1 are preserved,
        // so we can just jump to the next stub.
        masm.bc(Assembler::Overflow, &failure);

        // Box the result and return. We know R0.typeReg() already contains
        // the integer tag, so we just need to move the result value into
        // place.
        masm.x_mr(R0.payloadReg(), scratchReg);
        break;
      case JSOP_SUB:
        // Remember that subfo has weird operand order.
        masm.subfo(scratchReg, R1.payloadReg(), R0.payloadReg()); // D=B-A
        masm.bc(Assembler::Overflow, &failure);
        masm.x_mr(R0.payloadReg(), scratchReg);
        break;
      case JSOP_MUL: {
        masm.mullwo(scratchReg, R0.payloadReg(), R1.payloadReg());
        masm.bc(Assembler::Overflow, &failure);

        // If zero, it could be -0.
        masm.cmpwi(scratchReg, 0);
        masm.bc(Assembler::Equal, &maybeNegZero);

        masm.x_mr(R0.payloadReg(), scratchReg);
        break;
      }
      case JSOP_DIV:
      case JSOP_MOD: {
        // divwo will automatically set the Overflow bit if INT_MIN/-1 is
        // performed, or if we divide by zero. So we only need to check for
        // possible negative zero.
        Label ok;

        // Check for 0/x with x<0 (results in -0).
        masm.cmpwi(R0.payloadReg(), 0);
        masm.x_bne(cr0, PPC_BRANCH_STANZA_LENGTH + 8, // "ok", count the cmpwi!
            Assembler::LikelyB, // forward branch that IS likely
            Assembler::DontLinkB);
        masm.cmpwi(R1.payloadReg(), 0); 
        masm.bc(Assembler::LessThan, &failure);

        // Passed. Continue with the division.
        masm.bind(&ok); // target of x_bne
        masm.divwo(scratchReg, R0.payloadReg(), R1.payloadReg());
        masm.bc(Assembler::Overflow, &failure);
        
        // We need to compute the remainder to know if the result is not
        // integral. Here, divw is fine. Use r12 for the remainder.
        masm.divw(r12, R0.payloadReg(), R1.payloadReg());
        masm.mullw(r12, r12, R1.payloadReg());
        masm.subf(r12, r12, R0.payloadReg());

        if (op_ == JSOP_DIV) {
            // Result is a double if the remainder != 0.
            masm.branch32(Assembler::NotEqual, r12, Imm32(0), &failure);
            masm.tagValue(JSVAL_TYPE_INT32, scratchReg, R0);
        } else {
            // If X % Y == 0 and X < 0, the result is -0.
            Label done;
            masm.branch32(Assembler::NotEqual, r12, Imm32(0), &done);
            masm.branch32(Assembler::LessThan,
                R0.payloadReg(), Imm32(0), &failure);
            masm.bind(&done);
            masm.tagValue(JSVAL_TYPE_INT32, r12, R0);
        }
        break;
      }
      case JSOP_BITOR:
        masm.or_(R0.payloadReg(), R1.payloadReg(), R0.payloadReg());
        break;
      case JSOP_BITXOR:
        masm.xor_(R0.payloadReg(), R1.payloadReg(), R0.payloadReg());
        break;
      case JSOP_BITAND:
        masm.and_(R0.payloadReg(), R1.payloadReg(), R0.payloadReg());
        break;
      case JSOP_LSH:
        // Some PowerPC implementations may merrily try to shift by
        // more than 0x1f.
        masm.andi_rc(r0, R1.payloadReg(), 0x1f);
        masm.slw(R0.payloadReg(), R0.payloadReg(), r0);
        break;
      case JSOP_RSH:
        masm.andi_rc(r0, R1.payloadReg(), 0x1f);
        masm.sraw(R0.payloadReg(), R0.payloadReg(), r0);
        break;
      case JSOP_URSH:
        masm.andi_rc(scratchReg, R1.payloadReg(), 0x1f);
        masm.srw(scratchReg, R0.payloadReg(), scratchReg);
        // Check for negative sign. Use a signed compare here.
        masm.cmpwi(scratchReg, 0);
        if (allowDouble_) {
            Label toUint;
            masm.bc(Assembler::LessThan, &toUint);

            // Move result and box for return.
            masm.x_mr(R0.payloadReg(), scratchReg);
            EmitReturnFromIC(masm);

            masm.bind(&toUint);
            masm.convertUInt32ToDouble(scratchReg, ScratchFloatReg);
            masm.boxDouble(ScratchFloatReg, R0);
        } else {
            masm.bc(Assembler::LessThan, &failure);
            // Move result for return.
            masm.x_mr(R0.payloadReg(), scratchReg);
        }
        break;
      default:
        JS_NOT_REACHED("Unhandled op for BinaryArith_Int32.");
        return false;
    }

    EmitReturnFromIC(masm);

    // For future expansion.
    switch (op_) {
      case JSOP_MUL: {
        Label ok3, r0pos, pass;
        masm.bind(&maybeNegZero);

        // Result is -0 if exactly one of lhs or rhs is negative.
        masm.cmpwi(R0.payloadReg(), 0);
        masm.bc(Assembler::GreaterThanOrEqual, &r0pos);
        // If R1 is also negative, we did not fail; the result is zero.
        masm.cmpwi(R1.payloadReg(), 0);
        masm.bc(Assembler::LessThan, &pass);
        masm.b(&failure);
        // If R0 is positive or zero, and if R1 is negative, we failed.
        masm.bind(&r0pos);
        masm.cmpwi(R1.payloadReg(), 0);
        masm.bc(Assembler::LessThan, &failure);

        // Result is +0 (already boxed).
        masm.bind(&pass);
        masm.x_li32(R0.payloadReg(), 0);
        EmitReturnFromIC(masm);
        break;
      }
      default:
        break;
    }

    // Failure case - jump to next stub
    masm.bind(&failure);
    EmitStubGuardFailure(masm);

    return true;
}

bool
ICUnaryArith_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    Label failure;
    // Guard on int.
    masm.branchTestInt32(Assembler::NotEqual, R0, &failure);

    switch (op) {
      case JSOP_BITNOT:
        // We need to xori and xoris both.
        masm.xori(R0.payloadReg(), R0.payloadReg(), -1);
        masm.xoris(R0.payloadReg(), R0.payloadReg(), -1);
        break;
      case JSOP_NEG:
        // Guard against 0 and MIN_INT, since both result in a double.
        masm.branchTest32(Assembler::Equal, R0.payloadReg(),
            Imm32(0x7fffffff), &failure);
        masm.cmpwi(R0.payloadReg(), 0);
        masm.bc(Assembler::Equal, &failure);

        // Compile -x as 0 - x.
        masm.neg(R0.payloadReg(), R0.payloadReg());
        break;
      default:
        JS_NOT_REACHED("Unexpected op");
        return false;
    }

    EmitReturnFromIC(masm);

    masm.bind(&failure);
    EmitStubGuardFailure(masm);
    return true;
}

} // namespace jit
} // namespace js
