/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_move_resolver_ppc_shared_h__
#define jsion_move_resolver_ppc_shared_h__

#include "jit/MoveResolver.h"
#include "jit/IonMacroAssembler.h"

namespace js {
namespace jit {

class CodeGenerator;

class MoveEmitterPPC
{
    typedef MoveResolver::Move Move;
    typedef MoveResolver::MoveOperand MoveOperand;

    bool inCycle_;
    MacroAssemblerPPC &masm;

    // Original stack push value.
    uint32_t pushedAtStart_;

    // These store stack offsets to spill locations, snapshotting
    // codegen->framePushed_ at the time they were allocated. They are -1 if no
    // stack space has been allocated for that particular spill.
    int32_t pushedAtCycle_;
    int32_t pushedAtSpill_;
    int32_t pushedAtDoubleSpill_;

    // These are registers that are available for temporary use. They may be
    // assigned InvalidReg. If no corresponding spill space has been assigned,
    // then these registers do not need to be spilled.
    Register spilledReg_;
    FloatRegister spilledFloatReg_;

    void assertDone();
    Register tempReg();
    FloatRegister tempFloatReg();
    Operand cycleSlot() const;
    Operand spillSlot() const;
    Operand doubleSpillSlot() const;
    Operand toOperand(const MoveOperand &operand, bool isFloat) const;

    void emitMove(const MoveOperand &from, const MoveOperand &to);
    void emitDoubleMove(const MoveOperand &from, const MoveOperand &to);
    void breakCycle(const MoveOperand &from, const MoveOperand &to, Move::Kind kind);
    void completeCycle(const MoveOperand &from, const MoveOperand &to, Move::Kind kind);
    void emit(const Move &move);

  public:
    MoveEmitterPPC(MacroAssemblerPPC &masm);
    ~MoveEmitterPPC();
    void emit(const MoveResolver &moves);
    void finish();
};

typedef MoveEmitterPPC MoveEmitter;

} // namespace jit
} // namespace js

#endif // jsion_move_resolver_ppc_shared_h__

