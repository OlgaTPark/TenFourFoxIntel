/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/* XXX TODO: We don't spill registers, we have frickin' 64 of them.
   Remove the temp register and spilled reg code. */

#include "MoveEmitter-ppc.h"

using namespace js;
using namespace js::jit;

#define PPC_OK_DISP(x) JS_ASSERT(((x.disp() & 0xffff8000) == 0 || \
    (x.disp() & 0xffff8000) == 0xffff8000))

MoveEmitterPPC::MoveEmitterPPC(MacroAssemblerPPC &masm)
  : inCycle_(false),
    masm(masm),
    pushedAtCycle_(-1),
    pushedAtSpill_(-1),
    spilledReg_(InvalidReg), // XXX KILL IT WITH FIRE
    spilledFloatReg_(InvalidFloatReg) // XXX KILL IT WITH FIRE
{
    pushedAtStart_ = masm.framePushed();
}

void
MoveEmitterPPC::emit(const MoveResolver &moves)
{
    if (moves.hasCycles()) {
        // Reserve stack for cycle resolution
        masm.reserveStack(sizeof(double));
        pushedAtCycle_ = masm.framePushed();
    }

    for (size_t i = 0; i < moves.numMoves(); i++)
        emit(moves.getMove(i));
}

MoveEmitterPPC::~MoveEmitterPPC()
{
    assertDone();
}

// We have no linkage area that we can rely on, so (sigh) spill to stack.

Operand
MoveEmitterPPC::cycleSlot() const
{
    int offset =  masm.framePushed() - pushedAtCycle_;
    JS_ASSERT(offset < 4096 && offset > -4096);
    return Operand(StackPointer, offset);
}

// XXX remove
Operand
MoveEmitterPPC::spillSlot() const
{
    JS_ASSERT(0);
    int offset =  masm.framePushed() - pushedAtSpill_;
    JS_ASSERT(offset < 4096 && offset > -4096);
    return Operand(StackPointer, offset);
}

// Convert a generic MoveOperand, which we don't control, to an Operand,
// which we do. (See Assembler-ppc.h.)
Operand
MoveEmitterPPC::toOperand(const MoveOperand &operand, bool isFloat) const
{
    if (operand.isMemory() || operand.isEffectiveAddress()) {
        // Convert to REG_DISP.
        if (operand.base() != StackPointer) {
            JS_ASSERT(operand.disp() < 1024 && operand.disp() > -1024);
            Operand o = Operand(operand.base(), operand.disp());
            PPC_OK_DISP(o);
            return o;
        }

        JS_ASSERT(operand.disp() >= 0);
        
        // Otherwise, the stack offset may need to be adjusted.
        Operand o = Operand(StackPointer, operand.disp() + (masm.framePushed() - pushedAtStart_));
        PPC_OK_DISP(o);
    }

    if (operand.isGeneralReg())
        return Operand(operand.reg());

    JS_ASSERT(operand.isFloatReg());
    return Operand(operand.floatReg());
}

// Grab a temporary register.
Register
MoveEmitterPPC::tempReg()
{
    // XXX Remove this function, nothing calls it
    JS_ASSERT(0);
    return InvalidReg;
#if(0)

    // If already spilled, just use that.
    if (spilledReg_ != InvalidReg)
        return spilledReg_;

    // For now, just pick r10. XXX
    spilledReg_ = r10;
    if (pushedAtSpill_ == -1) {
        masm.Push(spilledReg_);
        pushedAtSpill_ = masm.framePushed();
    } else {
        JS_ASSERT(spillSlot().kind() != Operand::FPREG);
        masm.stw(spilledReg_, spillSlot().base(), spillSlot().disp());
    }
    return spilledReg_;
#endif
}

void
MoveEmitterPPC::breakCycle(const MoveOperand &from, const MoveOperand &to, Move::Kind kind)
{
    // Consider the possible code pattern below:
    //   (A -> B)
    //   (B -> A)
    //
    // This case handles (A -> B), which we reach first. We save B, then allow
    // the original move to continue.

#ifdef _PPC970_
    // XXX: G5 probably needs a hazard detector here to see if the lfd and
    // stfd/lwz-stw pairs are subject to aliasing.
#warning MoveEmitter needs G5 hazard detector
#endif

    PPC_OK_DISP(cycleSlot());
    if (kind == Move::DOUBLE) {
        // FPR
        if (to.isMemory()) {
            FloatRegister temp = ScratchFloatReg;
            Operand o = toOperand(to, true);
            masm.lfd(temp, o.base(), o.disp());
            masm.stfd(temp, cycleSlot().base(), cycleSlot().disp());
        } else {
            masm.stfd(to.floatReg(), cycleSlot().base(), cycleSlot().disp());
        }
    } else {
        // GPR
        if (to.isMemory()) {
            Register temp = r0;
            Operand o = toOperand(to, false);
            masm.lwz(temp, o.base(), o.disp());
            masm.stw(temp, cycleSlot().base(), cycleSlot().disp());
        } else {
            if (to.reg() == spilledReg_) { /// XXX REMOVE
                // If the destination was spilled, restore it first.
                masm.lwz(spilledReg_, spillSlot().base(), spillSlot().disp());
                spilledReg_ = InvalidReg;
            }
            masm.stw(to.reg(), cycleSlot().base(), cycleSlot().disp());
        }
    }
}

void
MoveEmitterPPC::completeCycle(const MoveOperand &from, const MoveOperand &to, Move::Kind kind)
{
    // Continuing the pattern above,
    //   (A -> B)
    //   (B -> A)
    //
    // This case handles (B -> A), which we reach last. We emit a move from the
    // saved value of B to A.

#ifdef _PPC970_
#warning MoveEmitter needs a hazard G5 detector badly
#endif

    PPC_OK_DISP(cycleSlot()); // this shouldn't ever happen
    if (kind == Move::DOUBLE) {
        if (to.isMemory()) {
            FloatRegister temp = ScratchFloatReg;
            Operand o = toOperand(to, true);
            masm.lfd(temp, cycleSlot().base(), cycleSlot().disp());
            masm.stfd(temp, o.base(), o.disp());
        } else {
            masm.lfd(to.floatReg(), cycleSlot().base(), cycleSlot().disp());
        }
    } else {
        if (to.isMemory()) {
            Register temp = r0;
            Operand o = toOperand(to, false);
            masm.lwz(temp, cycleSlot().base(), cycleSlot().disp());
            masm.stw(temp, o.base(), o.disp());
        } else {
            if (to.reg() == spilledReg_) { // XXX REMOVE
                // Make sure we don't re-clobber the spilled register later.
                spilledReg_ = InvalidReg;
            }
            masm.lwz(to.reg(), cycleSlot().base(), cycleSlot().disp());
        }
    }
}

void
MoveEmitterPPC::emitMove(const MoveOperand &from, const MoveOperand &to)
{
    if (to.isGeneralReg() && to.reg() == spilledReg_) { // XXX REMOVE
        // If the destination is the spilled register, make sure we
        // don't re-clobber its value.
        spilledReg_ = InvalidReg;
    }

    if (from.isGeneralReg()) {
        if (from.reg() == spilledReg_) { // XXX REMOVE
            // If the source is a register that has been spilled, make sure
            // to load the source back into that register.
            masm.lwz(spilledReg_, spillSlot().base(), spillSlot().disp());
            spilledReg_ = InvalidReg;
        }
        Operand o = toOperand(to, false);
        switch (o.kind()) {
          case Operand::REG:
            // secretly must be a register
            masm.x_mr(to.reg(), from.reg());
            break;
          case Operand::REG_DISP:
            masm.stw(from.reg(), o.base(), o.disp());
            break;
          default:
            JS_NOT_REACHED("strange move!");
        }
    } else if (to.isGeneralReg()) {
        JS_ASSERT(from.isMemory() || from.isEffectiveAddress());
        Operand o = toOperand(from, false);
        if (from.isMemory()) {
            masm.lwz(to.reg(), o.base(), o.disp());
        } else // compute effective address and put in to
            masm.add32(Imm32(from.disp()), from.base(), to.reg());
    } else {
        // Memory-to-memory GPR move
        Register reg = r0;
        Operand ofr = toOperand(from, false);
        Operand oto = toOperand(to, false);

        JS_ASSERT(from.isMemory() || from.isEffectiveAddress());
        if (from.isMemory()) {
            masm.lwz(reg, ofr.base(), ofr.disp());
        } else
            masm.add32(Imm32(from.disp()), from.base(), to.reg());
        JS_ASSERT(to.base() != reg);
        masm.stw(reg, oto.base(), oto.disp());
    }
}

void
MoveEmitterPPC::emitDoubleMove(const MoveOperand &from, const MoveOperand &to)
{
    // This can only be FPR-to-FPR or FPR to-from memory. (I hope.)
    if (from.isFloatReg()) {
        if (to.isFloatReg()) {
            // FPR to FPR
            masm.fmr(to.floatReg(), from.floatReg());
        } else {
            // FPR to memory
            JS_ASSERT(to.isMemory());
            Operand o = toOperand(to, true);
            masm.stfd(from.floatReg(), o.base(), o.disp());
        }
    } else if (to.isFloatReg()) {
        // Memory to FPR
        JS_ASSERT(from.isMemory());
        Operand o = toOperand(from, true);
        masm.lfd(to.floatReg(), o.base(), o.disp());
    } else {
        // Memory to memory float
        JS_ASSERT(from.isMemory());
        JS_ASSERT(to.isMemory());
        FloatRegister reg = ScratchFloatReg;
        Operand ofr = toOperand(from, true);
        Operand oto = toOperand(to, true);
        masm.lfd(reg, ofr.base(), ofr.disp());
#ifdef _PPC970_
#warning emitDoubleMove not optimized for 970
#endif
        masm.stfd(reg, oto.base(), oto.disp());
    }
}

void
MoveEmitterPPC::emit(const Move &move)
{
    const MoveOperand &from = move.from();
    const MoveOperand &to = move.to();

    if (move.inCycle()) {
        if (inCycle_) {
            completeCycle(from, to, move.kind());
            inCycle_ = false;
            return;
        }

        breakCycle(from, to, move.kind());
        inCycle_ = true;
    }

    if (move.kind() == Move::DOUBLE)
        emitDoubleMove(from, to);
    else
        emitMove(from, to);
}

void
MoveEmitterPPC::assertDone()
{
    JS_ASSERT(!inCycle_);
}

void
MoveEmitterPPC::finish()
{
    assertDone();

    if (pushedAtSpill_ != -1 && spilledReg_ != InvalidReg)
        masm.lwz(spilledReg_, spillSlot().base(), spillSlot().disp());
    masm.freeStack(masm.framePushed() - pushedAtStart_);
}
