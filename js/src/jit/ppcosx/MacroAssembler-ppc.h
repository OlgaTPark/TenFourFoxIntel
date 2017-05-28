/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_macro_assembler_ppc_h__
#define jsion_macro_assembler_ppc_h__

#include "jit/ppcosx/Assembler-ppc.h"
#include "jit/IonSpewer.h"
#include "jit/IonFrames.h"
#include "jit/MoveResolver.h"
#include "jscompartment.h"
#include "mozilla/DebugOnly.h"
#include "jit/IonFrames.h"
#include "jsopcode.h"
#include "jit/IonCaches.h"

/* TenFourFox's 32-bit PowerPC Ion MacroAssembler. We rock. */

namespace js {
namespace jit {

class MacroAssemblerPPC : public Assembler
{
  protected:
    // Bytes pushed onto the frame by the callee; includes frameDepth_. This is
    // needed to compute offsets to stack slots while temporary space has been
    // reserved for unexpected spills or C++ function calls. It is maintained
    // by functions which track stack alignment, which for clear distinction
    // use StudlyCaps (for example, Push, Pop).
    uint32_t framePushed_;

    // Number of bytes the stack is adjusted inside a call to C. Calls to C may
    // not be nested.
    bool inCall_;
    uint32_t args_;
    uint32_t stackForCall_;
    bool dynamicAlignment_;
    bool enoughMemory_;

    // Used by the routines that pass GPRs and FPRs.
    uint32_t passedGPRs_;
    uint32_t passedFPRs_;

    struct Double {
        double value;
        AbsoluteLabel uses;
        Double(double value) : value(value) {}
    };  
    Vector<Double, 0, SystemAllocPolicy> doubles_;
    typedef HashMap<double,
                    size_t,
                    DefaultHasher<double>,
                    SystemAllocPolicy> DoubleMap;
    DoubleMap doubleMap_;

    // Emit code to load the equivalent displacement for a generic base index
    // into addressTempRegister. This may clobber emergencyTempRegister too.
    // If we have another register already ready, then just use that.
    Register baseIndexToRegister(const BaseIndex &address) {
        if (address.scale == TimesOne) {
            if (address.offset == 0) 
                return address.index;
            add32(Imm32(address.offset), address.index, addressTempRegister);
        } else {
            if (PPC_OFFS_OK(address)) {
                x_slwi(addressTempRegister, address.index,
                                    address.scale);
                if (address.offset != 0)
                    addi(addressTempRegister, addressTempRegister,
                                        address.offset);
            } else {
                // Can't use r0 as a displacement, so we have to swap things up.
                ispew("!!baseIndexToRegister needed emergencyTempRegister");
                x_slwi(emergencyTempRegister,
                    address.index, address.scale);
                x_li32(addressTempRegister, address.offset);
                add(addressTempRegister, addressTempRegister,
                                    emergencyTempRegister);
            }
        }
        return addressTempRegister;
    }

  public:
    MacroAssemblerPPC()
      : framePushed_(0),
        inCall_(false),
        enoughMemory_(true),
        passedFPRs_(0),
        passedGPRs_(0)
    { }

    void compareDouble(DoubleCondition cond, const FloatRegister &lhs, const FloatRegister &rhs) {
        ispew("compareDouble(cond, fp, fp)");
        fcmpu(lhs, rhs); // CR0
    }

    void branchDouble(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, Label *label)
    {
        ispew("[[ branchDouble(cond, fp, fp, l)");
        compareDouble(cond, lhs, rhs);
        bc(cond, label);
        ispew("   branchDouble(cond, fp, fp, l) ]]");
    }

    void move32(const Imm32 &imm, const Register &dest) {
        ispew("move32(imm, reg)");
        if (imm.value == 0)
            xor_(dest, dest, dest);
        else
            x_li32(dest, imm.value);
    }
    void mov(const Imm32 &imm, const Register &dest) { move32(imm, dest); }
    void mov(ImmWord imw, const Register &dest) { move32(Imm32(imw.value),
        dest); }
    CodeOffsetLabel movWithPatch(const ImmWord &word, const Register &dest) {
        ispew("movWithPatch(immw, reg)");

        ensureSpace(PPC_BRANCH_STANZA_LENGTH + 8);
        CodeOffsetLabel j = currentOffset();
        x_p_li32(dest, word.value);
        return j;
    }

    void mov(const Register &src, const Register &dest) {
        ispew("mov(reg, reg)");
        x_mr(dest, src);
    }

    void and32(const Imm32 &imm, const Register &dest) {
        ispew("and32(imm, reg)");

        // XXX See if the same issues with i/is optimization apply here
        // as they did with methodjit.
        if (PPC_IMM_OK_U(imm.value)) {
            andi_rc(dest, dest, uint16_t(imm.value & 0xffff));
        } else if (!(imm.value & 0xffff)) {
            andis_rc(dest, dest, uint16_t(uint32_t(imm.value) >> 16));
        } else {
            // dest can safely be tempRegister.
            move32(imm, tempRegister);
            and_rc(dest, dest, tempRegister);
        }
    }
    void and32(const Imm32 &imm, const Address &dest) {
        ispew("[[ and32(imm, adr)");

        JS_ASSERT(dest.base != addressTempRegister);

        load32(dest, addressTempRegister);
        and32(imm, addressTempRegister);
        store32(addressTempRegister, dest); // from ARM
        ispew("   and32(imm, adr) ]]");
    }

    void or32(const Imm32 &imm, const Register &dest) {
        ispew("or32(imm, reg)");

        // XXX See if we can still get away with this.
        ori(dest, dest, uint16_t(imm.value & 0xffff));
        if (!PPC_IMM_OK_U(imm.value)) // also need to handle bits 0-15
            oris(dest, dest, uint16_t(uint32_t(imm.value) >> 16));
    }
    void or32(const Imm32 &imm, const Address &dest) {
        ispew("[[ or32(imm, adr)");

        JS_ASSERT(dest.base != addressTempRegister);

        load32(dest, addressTempRegister);
        or32(imm, addressTempRegister);
        store32(addressTempRegister, dest);
        ispew("   or32(imm, adr) ]]");
    }


    void xor32(const Imm32 &imm, const Register &dest) {
        ispew("xor32(imm, reg)");

        // XXX See if we can still get away with this.
        xori(dest, dest, uint16_t(imm.value & 0xffff));
        if (!PPC_IMM_OK_U(imm.value)) // also need to handle bits 0-15
            xoris(dest, dest, uint16_t(uint32_t(imm.value) >> 16));
    }
    void xor32(const Imm32 &imm, const Address &dest) {
        ispew("[[ xor32(imm, adr)");

        JS_ASSERT(dest.base != addressTempRegister);

        load32(dest, addressTempRegister);
        xor32(imm, addressTempRegister);
        store32(addressTempRegister, dest);
        ispew("   xor32(imm, adr) ]]");
    }


    void neg32(const Register &reg) {
        ispew("neg32(reg)");
        neg(reg, reg);
    }
    void notBoolean(const ValueOperand &val) {
        ispew("notBoolean(vo)");
        // Just toggle the lowest bit.
        xori(val.payloadReg(), val.payloadReg(), 1);
    }

    void test32(const Register &lhs, const Register &rhs) {
        ispew("test32(reg, reg)");

        // There's no PPC test instruction, but we can emulate it by doing
        // an and with Rc=1 to a throwaway register, which updates CR0.
        JS_ASSERT(lhs != tempRegister);
        JS_ASSERT(rhs != tempRegister);
        and_rc(tempRegister, lhs, rhs);
    }
    void test32(const Address &addr, const Imm32 &imm) {
        ispew("[[ test32(adr, imm)");

        JS_ASSERT(addr.base != tempRegister);
        JS_ASSERT(addr.base != addressTempRegister);
        load32(addr, tempRegister);
        x_li32(addressTempRegister, imm.value);
        and_rc(tempRegister, tempRegister, addressTempRegister);

        ispew("   test32(adr, imm) ]]");
    }

    // Signed compares.
    void cmp32(const Register &lhs, const Imm32 &imm) {
        ispew("cmp32(SIGNED reg,imm)");

        if (PPC_IMM_OK_S(imm.value)) {
            cmpwi(lhs, int16_t(imm.value & 0xffff));
        } else {
            JS_ASSERT(lhs != tempRegister);
            x_li32(tempRegister, imm.value);
            cmpw(lhs, tempRegister);
        }
    }
    void cmp32(const AbsoluteAddress &lhs, const Imm32 &imm) {
        ispew("cmp32(SIGNED adr,imm)");

        load32(lhs, addressTempRegister);
        if (PPC_IMM_OK_S(imm.value)) {
            cmpwi(addressTempRegister, int16_t(imm.value & 0xffff));
        } else {
            x_li32(tempRegister, imm.value);
            cmpw(addressTempRegister, tempRegister);
        }
    }
    void cmp32(const BaseIndex &bi, const Imm32 &imm) {
        ispew("cmp32(SIGNED bi,imm)");

        lwzx(tempRegister, bi.base, baseIndexToRegister(bi));
        if (PPC_IMM_OK_S(imm.value)) {
            cmpwi(tempRegister, int16_t(imm.value & 0xffff));
        } else {
            x_li32(addressTempRegister, imm.value);
            cmpw(tempRegister, addressTempRegister);
        }
    }
    void cmp32(Register a, Register b) {
        ispew("cmp32(SIGNED reg, reg)");
        cmpw(a, b);
    }
    void cmp32(const Operand &lhs, const Register &rhs) {
        ispew("cmp32(SIGNED op, reg)");

        switch(lhs.kind()) {
            case Operand::REG:
                cmpw(Register::FromCode(lhs.reg()), rhs);
                break;
            case Operand::REG_DISP:
                JS_ASSERT(rhs != addressTempRegister);
                JS_ASSERT(lhs.base() != addressTempRegister.code());

                if (PPC_IMMOFFS_OK(lhs.disp())) {
                    lwz(addressTempRegister, lhs.base(), lhs.disp());
                } else {
                    x_li32(addressTempRegister, lhs.disp());
                    lwzx(addressTempRegister, lhs.base(), addressTempRegister);
                }
                cmpw(addressTempRegister, rhs);
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("can't cmp GPR with FPR");
                return;
            default:
                JS_NOT_REACHED("unexpected operand type");
                return;
        }
    }
    void cmp32(const Register &lhs, const Operand &rhs) {
        ispew("cmp32(SIGNED reg, op)");

        switch(rhs.kind()) {
            case Operand::REG:
                cmpw(lhs, Register::FromCode(rhs.reg()));
                break;
            case Operand::REG_DISP:
                JS_ASSERT(lhs != addressTempRegister);
                JS_ASSERT(rhs.base() != addressTempRegister.code());

                if (PPC_IMMOFFS_OK(rhs.disp())) {
                    lwz(addressTempRegister, rhs.base(), rhs.disp());
                } else {
                    x_li32(addressTempRegister, rhs.disp());
                    lwzx(addressTempRegister, rhs.base(), addressTempRegister);
                }
                cmpw(lhs, addressTempRegister);
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("can't cmp GPR with FPR");
                return;
            default:
                JS_NOT_REACHED("unexpected operand type");
                return;
        }
    }
    void cmp32(const Operand &lhs, const Imm32 &rhs) {
        ispew("cmp32(SIGNED op, imm)");

        x_li32(tempRegister, rhs.value);
        cmp32(lhs, tempRegister);
    }
    // We also need cmp32(r, o)

    // Baseline version. Also used by some internal functions.
    // This is not const &.
    void add32(Imm32 imm, Register src, Register dest) {
        ispew("add32(imm, reg, reg)");

        if (PPC_IMM_OK_S(imm.value) && src != tempRegister) {
            addi(dest, src, int16_t(imm.value & 0xffff));
        } else if (src == addressTempRegister) {
            x_li32(tempRegister, imm.value);
            add(dest, src, tempRegister);
        } else {
            x_li32(addressTempRegister, imm.value);
            add(dest, src, addressTempRegister);
        }
    }
    void add32(Imm32 imm, Register dest) {
        ispew("add32(imm, reg)");
        add32(imm, dest, dest);
    }
    void add32(Imm32 imm, const Address &dest) {
        ispew("add32(imm, adr)");

        JS_ASSERT(dest.base != addressTempRegister);

        load32(dest, addressTempRegister);
        add32(imm, addressTempRegister, addressTempRegister);
        store32(addressTempRegister, dest);
    }
    void add32(Register src, Register dest) { // implied
        ispew("add32(reg, reg)");
        add(dest, dest, src);
    }

    // These are used in situations where overflow is possible. It is
    // less preferred to the above situations since it can serialize XER.
    void add32o(Imm32 imm, Register src, Register dest) {
        ispew("add32o(imm, reg, reg OVERFLOW)");

        // No "addio."
        if (src == addressTempRegister) {
            x_li32(tempRegister, imm.value);
            addo(dest, src, tempRegister);
        } else {
            x_li32(addressTempRegister, imm.value);
            addo(dest, src, addressTempRegister);
        }
    }
    void add32o(Register src, Operand rhs, Register dest) {
        ispew("add32o(reg, o, reg OVERFLOW)");

        switch(rhs.kind()) {
            case Operand::REG:
                addo(dest, src, Register::FromCode(rhs.reg()));
                break;
            case Operand::REG_DISP:
                JS_ASSERT(src != addressTempRegister);
                JS_ASSERT(rhs.base() != addressTempRegister.code());

                if (PPC_IMMOFFS_OK(rhs.disp())) {
                    lwz(addressTempRegister, rhs.base(), rhs.disp());
                } else {
                    x_li32(addressTempRegister, rhs.disp());
                    lwzx(addressTempRegister, rhs.base(), addressTempRegister);
                }
                addo(dest, src, addressTempRegister);
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("integer add not supported for FPR");
                return;
            default:
                JS_NOT_REACHED("unexpected operand type");
                return;
        }
    }

    void sub32(Imm32 imm, Register dest) {
        ispew("sub32(imm, reg)");

        // There is no subi. We do have an x_subi, but this is simpler.
        add32(Imm32(-(imm.value)), dest, dest);
    }
    void sub32(Imm32 imm, Register src, Register dest) {
        ispew("sub32(imm, reg, reg)");
        add32(Imm32(-(imm.value)), src, dest);
    }
    void sub32(Register src, Register dest) { // implied
        ispew("sub32(reg, reg)");
        // Intel uses subl(src, dest) which maps internally to subl_rr(src,
        // dest). Thus, this should be dest = dest - src. Remember, subf
        // T,A,B is T <= B-A!!
        subf(dest, src, dest);
    }

    // Like above, these allow for overflow, but can serialize XER. The other
    // forms are preferred.
    void sub32o(Imm32 imm, Register src, Register dest) {
        ispew("sub32o(imm, reg, reg OVERFLOW)");
        add32o(Imm32(-(imm.value)), src, dest);
    }
    void sub32o(Register src, Operand rhs, Register dest) {
        ispew("sub32o(reg, o, reg OVERFLOW)");

        switch(rhs.kind()) {
            case Operand::REG:
                // Remember: subfo T,A,B is T <= B-A !!!
                subfo(dest, Register::FromCode(rhs.reg()), src);
                break;
            case Operand::REG_DISP:
                JS_ASSERT(src != addressTempRegister);
                JS_ASSERT(rhs.base() != addressTempRegister.code());

                if (PPC_IMMOFFS_OK(rhs.disp())) {
                    lwz(addressTempRegister, rhs.base(), rhs.disp());
                } else {
                    x_li32(addressTempRegister, rhs.disp());
                    lwzx(addressTempRegister, rhs.base(), addressTempRegister);
                }
                subfo(dest, addressTempRegister, src);
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("integer sub not supported for FPR");
                return;
            default:
                JS_NOT_REACHED("unexpected operand type");
                return;
        }
    }

    // These branches emit cmpw or cmplw depending on the condition.
    // As is appropriate for a RISC architecture, this reg-reg method is the
    // "basis" for the other variations.
    void branch32(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        ispew("branch32(cond, reg, reg, l)");

        // Always CR0
        if (PPC_USE_UNSIGNED_COMPARE(cond)) {
            cmplw(lhs, rhs);
        } else {
            cmpw(lhs, rhs);
        }
        bc(cond, label);
    }
    void branch32(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        ispew("branch32(cond, reg, imm, l)");

        // Always CR0
        if (PPC_USE_UNSIGNED_COMPARE(cond)) {
            if (PPC_IMM_OK_U(imm.value)) {
                cmplwi(lhs, int16_t(imm.value & 0xffff));
            } else {
                JS_ASSERT(lhs != tempRegister);
                x_li32(tempRegister, imm.value);
                cmplw(lhs, tempRegister);
            }
        } else {
            if (PPC_IMM_OK_S(imm.value)) {
                cmpwi(lhs, int16_t(imm.value & 0xffff));
            } else {
                JS_ASSERT(lhs != tempRegister);
                x_li32(tempRegister, imm.value);
                cmpw(lhs, tempRegister);
            }
        }
        bc(cond, label);
    }
    void branch32(Condition cond, const Address &lhs, const Register &rhs, Label *label) {
        ispew("branch32(cond, adr, reg, l)");
        JS_ASSERT(rhs != addressTempRegister);
        JS_ASSERT(lhs.base != addressTempRegister);

        load32(lhs, addressTempRegister);
        branch32(cond, addressTempRegister, rhs, label);
    }
    void branch32(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        ispew("branch32(cond, adr, imm, l)");
        JS_ASSERT(lhs.base != addressTempRegister);

        load32(lhs, addressTempRegister);
        branch32(cond, addressTempRegister, imm, label);
    }
    void branch32(Condition cond, const AbsoluteAddress &lhs, Imm32 imm, Label *label) {
        ispew("[[ branch32(cond, aadr, imm, l)");

        load32(lhs, addressTempRegister);
        branch32(cond, addressTempRegister, imm, label);
        ispew("   branch32(cond, aadr, imm, l) ]]");
    }
    void branch32(Condition cond, Operand o, const Register &r, Label *label) {
        ispew("branch32(cond, o, reg, l)");

        // Figure out how to get the operand into a temp register, then
        // do the compare.
        switch (o.kind()) {
          case Operand::REG_DISP:
            // Get the result into addressTempRegister, and compare that.
            JS_ASSERT(o.base() != addressTempRegister.code());
            JS_ASSERT(o.base() != tempRegister.code());
            JS_ASSERT(r != addressTempRegister);
            JS_ASSERT(r != tempRegister);

            if (PPC_IMMOFFS_OK(o.disp())) {
                lwz(addressTempRegister, o.base(), o.disp());
            } else {
                x_li32(tempRegister, o.disp());
                lwzx(addressTempRegister, o.base(), tempRegister);
            }
            cmpw(Register::FromCode(o.reg()), r);
            break;

          case Operand::REG:
            // Direct reg-to-reg comparison.
            JS_ASSERT(o.reg() != r.code()); // Hmm. Why do the comp?
            cmpw(Register::FromCode(o.reg()), r);
            break;

          case Operand::FPREG:
            JS_NOT_REACHED("FPReg not supported for branch32 reg comp");
            return;

          default:
            JS_NOT_REACHED("unexpected operand kind");
            return;
        }
        bc(cond, label);
    }
    void branch32(Condition cond, Operand o, Imm32 imm, Label *label) {
        ispew("branch32(cond, o, imm32, l)");
        Register r;

        // Pick the temporary register we want to use based on the operand.
        switch(o.kind()) {
            case Operand::REG_DISP:
                JS_ASSERT(o.base() != addressTempRegister.code());
                JS_ASSERT(o.base() != tempRegister.code());
                if (PPC_IMMOFFS_OK(o.disp())) {
                    lwz(addressTempRegister, o.base(), o.disp());
                } else {
                    x_li32(tempRegister, o.disp());
                    lwzx(addressTempRegister, o.base(), tempRegister);
                }
                x_li32(tempRegister, imm.value);
                cmpw(Register::FromCode(o.reg()), tempRegister);
                bc(cond, label);
                return;

            case Operand::REG:
                r = (o.reg() == tempRegister.code()) ? addressTempRegister :
                    tempRegister;
                x_li32(r, imm.value);
                cmpw(Register::FromCode(o.reg()), r);
                bc(cond, label);
                return;

            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported for branch32 imm comp");
                return;
            default:
                JS_NOT_REACHED("unexpected operand kind");
                return;
        }
    }
    // SPS appears to be the only consumer of this, but for the record ...
    void branch32(Condition cond, const AbsoluteAddress &lhs, Register &rhs, Label *label) {
        ispew("[[ branch32(cond, aadr, reg, l)");

        load32(lhs, addressTempRegister);
        branch32(cond, addressTempRegister, rhs, label);
        ispew("   branch32(cond, aadr, reg, l) ]]");
    }

    void branchTest32(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        ispew("branchTest32(cond, reg, reg, l)");

        JS_ASSERT(lhs != tempRegister);
        JS_ASSERT(rhs != tempRegister);
        and_rc(tempRegister, lhs, rhs);
        bc(cond, label);
    }
    void branchTest32(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        ispew("branchTest32(cond, reg, imm, l)");

        JS_ASSERT(lhs != tempRegister);
        if (imm.value != -1) {
            if (PPC_IMM_OK_U(imm.value)) {
                andi_rc(tempRegister, lhs, imm.value);
            } else {
                x_li32(tempRegister, imm.value);
                and_rc(tempRegister, lhs, tempRegister);
            }
        } else {
            // Testing every bit; no mask needed.
            and_rc(tempRegister, lhs, lhs);
        }
        bc(cond, label); 
    }
    void branchTest32(Condition cond, const Address &address, Imm32 imm, Label *label) {
        ispew("branchTest32(cond, adr, imm, l)");
        JS_ASSERT(address.base != addressTempRegister);

        // Load the address into addressTempRegister.
        load32(address, addressTempRegister);
        branchTest32(cond, addressTempRegister, imm, label);
    }

    void branchTestBool(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        ispew("branchTestBool(cond, reg, reg, l)");

        // Fortunately, booleans on PowerPC are native word sized, so this is
        // just branchTest32.
        branchTest32(cond, lhs, rhs, label);
    }

    void push(const Register &r) {
        ispew("push(reg)");

        // cracked on G5
        stwu(r, stackPointerRegister, -4);
    }
    void push(Imm32 imm) {
        ispew("push(imm)");

        x_li32(tempRegister, imm.value);
        // cracked on G5
        stwu(tempRegister, stackPointerRegister, -4);
    }
    void push(ImmWord imm) { push(Imm32(imm.value)); }
    void push(ImmGCPtr ptr) {
        ispew("push(immgcptr)");

        writeDataRelocation(ptr);
        push(Imm32(ptr.value));
    }

    void push(const FloatRegister &r) {
        ispew("push(fpr)");

        // cracked on G5
        stfdu(r, stackPointerRegister, -8);
    }

    void pop(const Register &r) {
        ispew("pop(reg)");

        lwz(r, stackPointerRegister, 0);
        addi(stackPointerRegister, stackPointerRegister, 4);
    }
    void pop(const FloatRegister &r) {
        ispew("pop(fpr)");

        lfd(r, stackPointerRegister, 0);
        addi(stackPointerRegister, stackPointerRegister, 8);
    }

    // The following functions are exposed for use in platform-shared code.
    // We must have templates for Imm32, ImmWord and ImmGCPtr. (We do above.)
    template <typename T>
    void Push(const T &t) {
        push(t);
        framePushed_ += STACK_SLOT_SIZE;
    }
    void Push(const FloatRegister &t) {
        push(t);
        framePushed_ += sizeof(double);
    }
    CodeOffsetLabel PushWithPatch(const ImmWord &word) {
        framePushed_ += sizeof(word.value);
        return pushWithPatch(word);
    }

    void Pop(const Register &reg) {
        pop(reg);
        framePushed_ -= STACK_SLOT_SIZE;
    }
    void implicitPop(uint32_t args) {
        JS_ASSERT(args % STACK_SLOT_SIZE == 0);
        framePushed_ -= args;
    }
    uint32_t framePushed() const {
        return framePushed_;
    }
    void setFramePushed(uint32_t framePushed) {
        framePushed_ = framePushed;
    }

    // Optimized multi-push routines. These avoid cracking on G5. The
    // sp gets serialized slightly here, but the stores can occur in
    // a superscalar manner since the effective addresses are independent.
    // Storing into the red zone is unsafe for this purpose, so we don't,
    // and SysV ABI doesn't have one anyway.
    void push2(const Register &rr0, const Register &rr1) {
        addi(stackPointerRegister, stackPointerRegister, -8);
        stw(rr0, stackPointerRegister, 4);
        stw(rr1, stackPointerRegister, 0);
    }
    void Push2(const Register &rr0, const Register &rr1) {
        framePushed_ += 8; push2(rr0, rr1);
    }
    void push3(const Register &rr0, const Register &rr1, const Register &rr2) {
        addi(stackPointerRegister, stackPointerRegister, -12);
        stw(rr0, stackPointerRegister, 8);
        stw(rr1, stackPointerRegister, 4);
        stw(rr2, stackPointerRegister, 0);
    }
    void Push3(const Register &rr0, const Register &rr1, const Register &rr2) {
        framePushed_ += 12; push3(rr0, rr1, rr2);
    }
    void push4(const Register &rr0, const Register &rr1, const Register &rr2,
        const Register &rr3) {
        addi(stackPointerRegister, stackPointerRegister, -16);
        stw(rr0, stackPointerRegister, 12);
        stw(rr1, stackPointerRegister, 8);
        stw(rr2, stackPointerRegister, 4);
        stw(rr3, stackPointerRegister, 0);
    }
    void Push4(const Register &rr0, const Register &rr1, const Register &rr2,
        const Register &rr3) {
        framePushed_ += 16; push4(rr0, rr1, rr2, rr3);
    }
    void push5(const Register &rr0, const Register &rr1, const Register &rr2,
        const Register &rr3, const Register &rr4) {
        addi(stackPointerRegister, stackPointerRegister, -20);
        stw(rr0, stackPointerRegister, 16);
        stw(rr1, stackPointerRegister, 12);
        stw(rr2, stackPointerRegister, 8);
        stw(rr3, stackPointerRegister, 4);
        stw(rr4, stackPointerRegister, 0);
    }
    void Push5(const Register &rr0, const Register &rr1, const Register &rr2,
        const Register &rr3, const Register &rr4) {
        framePushed_ += 20; push5(rr0, rr1, rr2, rr3, rr4);
    }

    // And, similarly, multipop.
    void pop2(const Register &rr0, const Register &rr1) {
        lwz(rr0, stackPointerRegister, 0);
        lwz(rr1, stackPointerRegister, 4);
        addi(stackPointerRegister, stackPointerRegister, 8);
    }
    void Pop2(const Register &rr0, const Register &rr1) {
        framePushed_ -= 8; pop2(rr0, rr1);
    }
    void pop3(const Register &rr0, const Register &rr1, const Register &rr2) {
        lwz(rr0, stackPointerRegister, 0);
        lwz(rr1, stackPointerRegister, 4);
        lwz(rr2, stackPointerRegister, 8);
        addi(stackPointerRegister, stackPointerRegister, 12);
    }
    void Pop3(const Register &rr0, const Register &rr1, const Register &rr2) {
        framePushed_ -= 12; pop3(rr0, rr1, rr2);
    }
    void pop4(const Register &rr0, const Register &rr1, const Register &rr2,
        const Register &rr3) {
        lwz(rr0, stackPointerRegister, 0);
        lwz(rr1, stackPointerRegister, 4);
        lwz(rr2, stackPointerRegister, 8);
        lwz(rr3, stackPointerRegister, 12);
        addi(stackPointerRegister, stackPointerRegister, 16);
    }
    void Pop4(const Register &rr0, const Register &rr1, const Register &rr2,
        const Register &rr3) {
        framePushed_ -= 16; pop4(rr0, rr1, rr2, rr3);
    }


    void jump(Label *label) {
        ispew("jump(l)");
        b(label);
    }
    void jump(RepatchLabel *label) {
        ispew("jump(rl)");
        b(label);
    }
    void jump(Register reg) {
        ispew("jump(reg)");
        b(reg);
    }
    // Sigh.
    void j(Label *label) { jump(label); }
    void j(RepatchLabel *label) { jump(label); }
    void j(Register reg) { jump(reg); }
    void j(Condition c, Label *l) { bc(c, l); }

    void nop() { x_nop(); }

    // This has reduced safeties, and is a convenience for the functions below.
    void dangerous_convertInt32ToDouble(const Register &src, const FloatRegister &dest, const FloatRegister &tempfp) {
        // PowerPC has no GPR<->FPR moves, so we need a memory intermediate.
        // Since we might not have a linkage area, we do this on the stack.
        // See also OPPCC chapter 8 p.156.
        // Because Baseline expects to use f0, our normal temporary FPR,
        // we dedicate f2 to conversions (except when it isn't).
        JS_ASSERT(src != tempRegister);

        // Build zero double-precision FP constant.
        x_lis(tempRegister, 0x4330);  // 1 instruction
        // cracked on G5
        stwu(tempRegister, stackPointerRegister, -8);
        x_lis(tempRegister, 0x8000); // 1 instruction

        // 2nd G5 dispatch group
        stw(tempRegister, stackPointerRegister, 4); 
#ifdef _PPC970_
        // G5 and POWER4+ do better if the lfd and the stws aren't in the
        // same dispatch group.
        x_nop();
        x_nop();
        x_nop();
#endif
        
        // 3rd G5 dispatch group
        // Build intermediate float from zero constant.
        lfd(tempfp, stackPointerRegister, 0);
        // Flip sign of integer value and use as integer component.
        xoris(tempRegister, src, 0x8000);
#ifdef _PPC970_
        // G5 and POWER4+ do better if the lfd and the stws aren't in the
        // same dispatch group.
        x_nop();
        x_nop();
#endif

        // 4rd G5 dispatch group
        stw(tempRegister, stackPointerRegister, 4);
#ifdef _PPC970_
        x_nop();
        x_nop();
        x_nop();
#endif

        // Load and normalize with a subtraction operation.
        lfd(dest, stackPointerRegister, 0);
        fsub(dest, dest, tempfp);

        // Destroy temporary frame.
        addi(stackPointerRegister, stackPointerRegister, 8);
    }
    void convertInt32ToDouble(const Register &src, const FloatRegister &dest) {
        ispew("convertInt32ToDouble(reg, reg)");

        JS_ASSERT(src != tempRegister);
        dangerous_convertInt32ToDouble(src, dest,
            (dest == fpTempRegister) ? fpConversionRegister : // Baseline
                fpTempRegister);
    }
    void convertInt32ToDouble(const Address &addr, const FloatRegister &dest) {
        // XXX: This ass-U-mes that the address is pointing to a jsval,
        // which means that we need to convert the 32-bit word at addr+4
        // because type is at addr+0 in our superior big-endian world.
        // In all current uses the address is always 0(r1), so assert that
        // in case Mozilla one day ever fixes it, or I fix it as part of
        // something else.
        ispew("[[ convertInt32ToDouble(adr, fpr) (ASSUMING ADR IS JSVAL)");
        JS_ASSERT(addr.offset == 0 && addr.base == stackPointerRegister);
        lwz(addressTempRegister, addr.base, 4);

        dangerous_convertInt32ToDouble(addressTempRegister, dest,
            (dest == fpTempRegister) ? fpConversionRegister : // Baseline
                fpTempRegister);

        ispew("   convertInt32ToDouble(adr, fpr) ]]");
    }

    static uint8_t crBit(CRegisterID cr, Assembler::DoubleCondition cond)
    {
        return (cr << 2) + ((cond & 0xf0) >> 4);
    }

    // Return the condition needed to test the register for "truth" based
    // on zero or non zero. Note that this does NOT return a DoubleCondition.
    Condition testDoubleTruthy(bool truthy, const FloatRegister &reg) {
        ispew("testDoubleTruthy(bool, fpreg)");

        // Make a zero FPR.
        JS_ASSERT(reg != fpTempRegister);
        zeroDouble(fpTempRegister);

        // Now compare the register to the zero FPR.
        fcmpu(fpTempRegister, reg);

        // Disordered results must evaluate to "not equal to zero" (thus
        // boolean true); force the CR FE bit to true if the FU bit is true.
        const uint8_t fuBit = crBit(cr0, Assembler::DoubleUnordered); 
        const uint8_t eqBit = crBit(cr0, Assembler::DoubleEqual);
        cror(eqBit, eqBit, fuBit);

        // Fortunately, even though this is returning an integer condition,
        // we have now made the CR bits the same for the double comparison.
        return truthy ? NotEqual : Equal;
    }

    // Truncate src to 32 bits and move to dest. If not representable in 32
    // bits, branch to the failure label.
    void branchTruncateDouble(const FloatRegister &src, const Register &dest, Label *fail) {
        ispew("branchTruncateDouble(fpreg, reg, l)");
        JS_ASSERT(src != fpTempRegister);
        JS_ASSERT(dest != tempRegister);

        // Again, no GPR<->FPR moves! So, back to the stack.
        // Turn into a fixed-point integer (i.e., truncate).
        fctiwz(fpTempRegister, src);
        // Stuff in a temporary frame.
        // cracked on G5
        stfdu(fpTempRegister, stackPointerRegister, -8);
        // Load this constant here (see below); this spaces the dispatch
        // group out and can be parallel. Even if the stfdu leads the dispatch
        // group, it's cracked, so two slots, and this load must use two, and
        // the lwz will not go in the branch slot.
        x_li32(tempRegister, 0x7fffffff);
        // Pull out the lower 32 bits. This is the result.
        lwz(dest, stackPointerRegister, 4);

        // We don't care if a truncation occurred. In fact, all we care is
        // that the integer result fits in 32 bits. Fortunately, fctiwz will
        // tip us off: if src > 2^31-1, then dest becomes 0x7fffffff, the
        // largest 32-bit positive integer. If src < -2^31, then dest becomes
        // 0x80000000, the largest 32-bit negative integer. So we just test
        // for those two values. If either value is found, fail-branch. Use
        // unsigned compares, since these are logical values.

        // (tempRegister was loaded already)
        cmplw(dest, tempRegister);
        // Destroy the temporary frame before testing, since we might branch.
        // The cmplw will execute in parallel for "free."
        addi(stackPointerRegister, stackPointerRegister, 8);
        bc(Equal, fail);

        x_li32(tempRegister, 0x80000000); // sign extends!
        cmplw(dest, tempRegister);
        bc(Equal, fail);
    }

    // We do something similar for load32 (read on).
    void load8ZeroExtend(const Address &src, const Register &dest) {
        ispew("load8ZeroExtend(adr, reg)");

        if (PPC_OFFS_OK(src)) {
            lbz(dest, src.base, src.offset);
        } else {
            // We don't have to assert on dest not being addressTempRegister.
            x_li32(addressTempRegister, src.offset);
            lbzx(dest, src.base, addressTempRegister);
        }
    }
    void load8ZeroExtend(const BaseIndex &src, const Register &dest) {
        ispew("load8ZeroExtend(bi, reg)");

        JS_ASSERT(dest != emergencyTempRegister);
        lbzx(dest, src.base, baseIndexToRegister(src));
    }
    void load8SignExtend(const Address &src, const Register &dest) {
        ispew("load8SignExtend(adr, reg)");

        load8ZeroExtend(src, dest);
        extsb(dest, dest);
    }
    void load8SignExtend(const BaseIndex &src, const Register &dest) {
        ispew("load8SignExtend(bi, reg)");

        load8ZeroExtend(src, dest);
        extsb(dest, dest);
    }

    // All generalized loads and stores need to implement
    // reg-address
    // imm-address
    // reg-baseindex
    // imm-baseindex
    // Don't even bother with the template; it's not effective here.

    void store8(const Register &src, const Address &address) {
        ispew("store8(reg, adr)");

        if (PPC_OFFS_OK(address)) {
            stb(src, address.base, address.offset);
        } else {
            ASSERT(src != addressTempRegister);

            x_li32(addressTempRegister, address.offset);
            stbx(src, address.base, addressTempRegister);
        }
    }
    void store8(const Imm32 &src, const Address &address) {
        ispew("store8(imm, adr)");

        x_li32(tempRegister, src.value);
        store8(tempRegister, address);
    }
    void store8(const Register &src, const BaseIndex &dest) {
        ispew("store8(reg, bi)");

        stbx(src, dest.base, baseIndexToRegister(dest));
    }
    void store8(const Imm32 &src, const BaseIndex &dest) {
        ispew("store8(imm, bi)");

        x_li32(tempRegister, src.value);
        store8(tempRegister, dest);
    }

    void load16ZeroExtend(const Address &address, const Register &dest) {
        ispew("load16ZeroExtend(adr, reg)");

        if (PPC_OFFS_OK(address))
            lhz(dest, address.base, address.offset);
        else {
            x_li32(addressTempRegister, address.offset);
            lhzx(dest, address.base, addressTempRegister);
        }
    }
    void load16ZeroExtend(const BaseIndex &src, const Register &dest) {
        ispew("load16ZeroExtend(bi, reg)");

        JS_ASSERT(dest != emergencyTempRegister);
        lhzx(dest, src.base, baseIndexToRegister(src));
    }
    void load16SignExtend(const Address &src, const Register &dest) {
        ispew("load16SignExtend(adr, reg)");

        load16ZeroExtend(src, dest);
        extsh(dest, dest);
    }
    void load16SignExtend(const BaseIndex &src, const Register &dest) {
        ispew("load16SignExtend(bi, reg)");

        load16ZeroExtend(src, dest);
        extsh(dest, dest);
    }

    // See store8(*, *)
    // reg-address
    // imm-address
    // reg-baseindex
    // imm-baseindex
    void store16(const Register &src, const Address &address) {
        ispew("store16(reg, adr)");

        if (PPC_OFFS_OK(address)) {
            sth(src, address.base, address.offset);
        } else {
            ASSERT(src != addressTempRegister);

            x_li32(addressTempRegister, address.offset);
            sthx(src, address.base, addressTempRegister);
        }
    }
    void store16(const Imm32 &src, const Address &address) {
        ispew("store16(imm, adr)");

        x_li32(tempRegister, src.value);
        store16(tempRegister, address);
    }
    void store16(const Register &src, const BaseIndex &dest) {
        ispew("store16(reg, bi)");

        sthx(src, dest.base, baseIndexToRegister(dest));
    }
    void store16(const Imm32 &src, const BaseIndex &dest) {
        ispew("store16(imm, bi)");

        x_li32(tempRegister, src.value);
        store16(tempRegister, dest);
    }

    void load32(const Address &address, Register dest) {
        ispew("load32(adr, reg)");

        JS_ASSERT(address.base != tempRegister);
        if (PPC_OFFS_OK(address))
            lwz(dest, address.base, address.offset);
        else {
            x_li32(addressTempRegister, address.offset);
            lwzx(dest, address.base, addressTempRegister);
        }
    }
    void load32(const AbsoluteAddress &address, Register dest) {
        ispew("load32(aadr, reg)");

        x_li32(addressTempRegister, (uint32_t)address.addr);
        lwz(dest, addressTempRegister, 0);
    }
    void load32(const BaseIndex &src, Register dest) {
        ispew("load32(bi, reg)");

        JS_ASSERT(src.base != tempRegister);
        lwzx(dest, src.base, baseIndexToRegister(src));
    }

    // See store8(*, *)
    // reg-address
    // imm-address
    // reg-baseindex
    // imm-baseindex
    void store32(const Register &src, const Address &address) {
        ispew("store32(reg, adr)");

        if (PPC_OFFS_OK(address)) {
            stw(src, address.base, address.offset);
        } else {
            ASSERT(src != addressTempRegister);

            x_li32(addressTempRegister, address.offset);
            // cracked on G5
            stwx(src, address.base, addressTempRegister);
        }
    }
    void store32(const Imm32 &src, const Address &address) {
        ispew("store32(imm, adr)");

        x_li32(tempRegister, src.value);
        store32(tempRegister, address);
    }
    void store32(const Register &src, const BaseIndex &dest) {
        ispew("store32(reg, bi)");

        // cracked on G5
        stwx(src, dest.base, baseIndexToRegister(dest));
    }
    void store32(const Imm32 &src, const BaseIndex &dest) {
        ispew("store32(imm, bi)");

        x_li32(tempRegister, src.value);
        store32(tempRegister, dest);
    }

    void loadDouble(const Address &address, FloatRegister dest) {
        ispew("loadDouble(adr, fpr)");

        if (PPC_OFFS_OK(address))
            lfd(dest, address.base, address.offset);
        else {
            x_li32(addressTempRegister, address.offset);
            lfdx(dest, address.base, addressTempRegister);
        }
    }
    void loadDouble(const BaseIndex &src, FloatRegister dest) {
        ispew("loadDouble(bi, fpr)");

        lfdx(dest, src.base, baseIndexToRegister(src));
    }

    void storeDouble(FloatRegister src, const Address &address) {
        ispew("storeDouble(fpr, adr)");

        if (PPC_OFFS_OK(address))
            stfd(src, address.base, address.offset);
        else {
            x_li32(addressTempRegister, address.offset);
            stfdx(src, addressTempRegister, address.base);
        }
    }
    void storeDouble(FloatRegister src, const BaseIndex &dest) {
        ispew("storeDouble(fpr, bi)");

        stfdx(src, dest.base, baseIndexToRegister(dest));
    }

    void zeroDouble(FloatRegister reg) {
        ispew("zeroDouble(fpr)");
        // Load 64 zeroes into an FPR, which is the value of "zero."
        // Do this on the stack again.

        xor_(tempRegister, tempRegister, tempRegister);
        // These are cracked on G5, so they take two slots.
        stwu(tempRegister, stackPointerRegister, -4);
        stwu(tempRegister, stackPointerRegister, -4);
#ifdef _PPC970_
        // G5 and POWER4+ do better if the lfd and the stws aren't in the
        // same dispatch group. It's better to insert the nops here because
        // then we know for sure the stwus will be split from the lfd. Two
        // ensures that even if one stwu leaks into the lfd dispatch stream,
        // they'll still be separate.
        x_nop();
        x_nop();
#endif
        lfd(reg, stackPointerRegister, 0);
        addi(stackPointerRegister, stackPointerRegister, 8);
    }

    void negateDouble(FloatRegister reg) {
        ispew("negateDouble(fpr)");
        fneg(reg, reg);
    }

    void addDouble(FloatRegister src, FloatRegister dest) {
        ispew("addDouble(fpr, fpr)");
        fadd(dest, src, dest);
    }
    void subDouble(FloatRegister src, FloatRegister dest) {
        // Intel uses subsd(src, dest), which internally maps into
        // subsd_rr(dest, src). So, this should be dest = dest - src.
        fsub(dest, dest, src);
    }
    void mulDouble(FloatRegister src, FloatRegister dest) {
        fmul(dest, dest, src);
    }
    void divDouble(FloatRegister src, FloatRegister dest) {
        // Intel uses divsd(src, dest), which internally maps into
        // divsd_rr(dest, src). So, this should be dest = dest / src.
        // This does not require the remainder to be computed.
        fdiv(dest, dest, src);
    }

    void convertDoubleToFloat(const FloatRegister &src, const FloatRegister &dest) {
        ispew("convertDoubleToFloat(fpr, fpr)");
        frsp(dest, src);
    }

    void convertDoubleToInt32(const FloatRegister &src, const Register &dest, Label *fail, bool negativeZeroCheck) {
        ispew("convertDoubleToInt32(fpr, reg, l, bool)");

        // Convert 'src' to an integer and move to 'dest'. If the result is
        // not representable as an integer (i.e., non-integral, or out of
        // range), branch. (Essentially this is the old
        // branchConvertDoubleToInt32.)
        JS_ASSERT(src != fpTempRegister);
        JS_ASSERT(dest != tempRegister);

        // Turn into a fixed-point integer (i.e., truncate). Set FX for any
        // exception (inexact or bad conversion), which becomes CR1+LT with
        // fctiwz_rc. FI is not sticky, so we need not whack it, but XX is.
        // On G5, all of these instructions are separate dispatch groups.
        // Worse still, mtfsb* requires FPSCR to be serialized, so clear as
        // few bits as possible.
        mtfsb0(6);  // whack XX
        mtfsb0(23); // whack VXCVI
        mtfsb0(0);  // then whack summary FX

        // fctiwz. (not fctiwz) is also a dispatch group unto itself.
        fctiwz_rc(fpTempRegister, src);

        // New dispatch group.
        // Move CR1 to CR0 since our branches don't use it yet. mcrf must
        // lead a dispatch group.
        mcrf(cr0, cr1);
        // Push on the stack so we can pull it off in pieces.
        // cracked on G5
        stfdu(fpTempRegister, stackPointerRegister, -8);
#ifdef _PPC970_
        // G5 and POWER4+ do better if the stfd and the lwz aren't in the
        // same dispatch group.
        x_nop();
#endif

        // Next dispatch group.
        // Pull out the lower 32 bits. This is the result.
        lwz(dest, stackPointerRegister, 4);
        // Remove the temporary frame.
        addi(stackPointerRegister, stackPointerRegister, 8);

        // Test and branch if inexact (i.e., if "less than").
        bc(LessThan, fail);

        // If negativeZeroCheck is true, we need to also branch to the
        // failure label if the result is -0 (if false, we don't care).
        if (negativeZeroCheck) {
            Label nonZero;
            ispew("<< checking for negativeZero >>");

            // If it's not zero, don't bother.
            and_rc(tempRegister, dest, dest);
            bc(NonZero, &nonZero);

            // FP negative zero is 0x8000 0000 0000 0000 in the IEEE 754
            // standard, so test the upper 32 bits by extracting it on the
            // stack (similar to our various breakDouble iteractions). We have
            // no constant to compare against, so this is the best option.
            // TODO: Might be able to do this in one step above.
            stfdu(src, stackPointerRegister, -8);
#ifdef _PPC970_
            x_nop();
            x_nop(); // cracked
#endif
            lwz(tempRegister, stackPointerRegister, 0); // upper 32 bits
            and_rc(tempRegister, dest, dest);
            addi(stackPointerRegister, stackPointerRegister, 8);
            bc(NonZero, fail);

            bind(&nonZero);
        }
    }

    // ARM doesn't have a loadFloatAsDouble for gpr->fpr, so we don't either
    void loadFloatAsDouble(const Address &address, FloatRegister dest) {
        ispew("loadFloatAsDouble(adr, fpr)");
        /* Intel does this as
        movss(Operand(src), dest);
        cvtss2sd(dest, dest);
        So, load a single precision float into the FPR, and go double. */

        if (PPC_OFFS_OK(address))
            lfs(dest, address.base, address.offset);
        else {
            x_li32(addressTempRegister, address.offset);
            lfsx(dest, address.base, addressTempRegister);
        }
        // The FPU automatically converts to double precision for us.
    }

    void loadFloatAsDouble(const BaseIndex &src, FloatRegister dest) {
        ispew("loadFloatAsDouble(bi, fpr)");

        lfsx(dest, src.base, baseIndexToRegister(src));
        // The FPU automatically converts to double precision for us.
    }
    void storeFloat(FloatRegister src, const Address &address) {
        ispew("storeFloat(fpr, adr)");

        if (PPC_OFFS_OK(address))
            stfs(src, address.base, address.offset);
        else {
            x_li32(addressTempRegister, address.offset);
            stfsx(src, addressTempRegister, address.base);
        }
    }
    void storeFloat(FloatRegister src, const BaseIndex &dest) {
        ispew("storeFloat(fpr, bi)");

        stfsx(src, dest.base, baseIndexToRegister(dest));
    }

    void clampIntToUint8(Register src, Register dest) {
        ispew("[[ clampIntToUint8(reg, reg)");
        // If < 0, return 0.
        // If > 255, return 255.

        Register ssrc = src;
        if (src == dest) { // sheesh
            x_mr(tempRegister, src);
            ssrc = tempRegister;
        }

        // Exploit this property for a branchless version:
        // max(a,0) = (a + abs(a)) / 2 (do this first)
        // min(a,255) = (a + 255 - abs(a-255)) / 2
        // Use dest as temporary work area.
        // 
        // First, compute max(src,0), starting with abs(src).
        // Absolute value routine from PowerPC Compiler Writer's Guide, p.50.
        srawi(dest, ssrc, 31);
        xor_(addressTempRegister, dest, ssrc);
        subf(dest, dest, addressTempRegister);
        // Finally, add src and divide by 2. Leave in dest.
        add(dest, dest, ssrc);
        srawi(dest, dest, 1);

        // Now for min(dest, 255). First, compute abs(a-255) into adrTemp.
        // We can clobber tempRegister safely now, since we don't need the
        // original value anymore, but we need to push dest since we need
        // three working registers for the integer absolute value.
        //
        x_li32(addressTempRegister, 255);
        stwu(dest, stackPointerRegister, -4);
        subf(addressTempRegister, addressTempRegister, dest); // T=B-A
        // Okay to clobber dest now ...
        srawi(dest, addressTempRegister, 31);
        xor_(r0, dest, addressTempRegister);
        subf(addressTempRegister, dest, r0);
        // Now 255 - addressTempRegister
        x_li32(r0, 255);
        subf(addressTempRegister, addressTempRegister, r0); // T=B-A
        // Get dest back, add it to addressTempRegister, and divide by 2.
        // There were adequate instructions between this lwz and the stwu to
        // keep them in separate G5 branch groups, so no nops are needed.
        lwz(dest, stackPointerRegister, 0);
        addi(stackPointerRegister, stackPointerRegister, 4);
        add(dest, dest, addressTempRegister);
        srawi(dest, dest, 1);
        // Ta-daaa!

        ispew("   clampIntToUint8(reg, reg) ]]");
    }

    // Mozilla says this is "Emit a JMP that can be toggled to a CMP," but
    // the actual use is a bit more prosaic. ToggleToJmp enables the jump to
    // the destination, and ToggleToCmp disables it (by using Cmp, which on
    // lesser architectures like x86 only sets cond codes and tanks no regs).
    // We do this a little more directly and sensibly, since we're superior.
    // See ToggleToJmp() and ToggleToCmp() in the Assembler.
    CodeOffsetLabel toggledJump(Label *label) {
        ispew("toggledJump(l)");
        // Make sure we have enough space for everything or our skip
        // stanza instruction could skip us right into the constant pool!
        ensureSpace(PPC_BRANCH_STANZA_LENGTH + 16);

        // It's now safe to get the label.
        // Don't use size(); it may flush the pool.
        CodeOffsetLabel offset(currentOffset());

        // Emit a nop first. This will get toggled to a "skip this jump"
        // if ToggleToCmp is called. The default state is to jump.
        x_nop();
        b(label);

        return offset;
    }
    // This uses a similar system to toggledJump, but for IonCode.
    CodeOffsetLabel toggledCall(IonCode *target, bool enabled) {
        ispew("toggledCall(icode, bool)");
        // Emit, depending on "enabled," nop or "skip this jump" so that
        // it can be toggled in the same way as a toggledJump (see
        // ToggleCall in the Assembler).

        // Make sure we have enough space for everything or our skip
        // stanza instruction could skip us right into the constant pool!
        ensureSpace(PPC_BRANCH_STANZA_LENGTH + 16);

        // It's now safe to get the label.
        // Don't use size(); it may flush the pool.
        CodeOffsetLabel offset(currentOffset());

        if (enabled) {
            x_nop();
        } else {
            x_skip_stanza(PPC_CALL_STANZA_LENGTH);
        }
        JmpSrc src = masm.m_call(); // to link later
        addPendingCall(src, target->raw(), Relocation::IONCODE);

        return offset;
    }
    static size_t ToggledCallSize() { return PPC_CALL_STANZA_LENGTH   + 4; }
    static size_t ToggledJumpSize() { return PPC_BRANCH_STANZA_LENGTH + 4; }

    // LEAve off this effective address crap!
    void computeEffectiveAddress(const Address &address, Register dest) {
        ispew("computeEffectiveAddress(adr, reg)");

        if (PPC_IMM_OK_S(address.offset)) {
            addi(dest, address.base, address.offset);
        } else {
            x_li32(tempRegister, address.offset);
            add(dest, address.base, tempRegister);
        }
    }
    void computeEffectiveAddress(const BaseIndex &address, Register dest) {
        ispew("computeEffectiveAddress(bi, reg)");

        add(dest, address.base, baseIndexToRegister(address));
    }

    // Convenience routine for local consumers needing an EA for an Address,
    // which saves an addi if the address has no offset (common situation).
    Register computeEAToRegister(const Address &address) {
        ispew("computeEAToRegister(adr)");
        if (!address.offset)
            return address.base;
        computeEffectiveAddress(address, addressTempRegister);
        return addressTempRegister;
    }

    // Incidentally, I write CPU-racist comments like this in here where no
    // one will ever notice.

    bool buildFakeExitFrame(const Register &scratch, uint32_t *offset);
    bool buildOOLFakeExitFrame(void *fakeReturnAddr);
    void callWithExitFrame(IonCode *target);
    void callIon(const Register &callee);
    void linkParallelExitFrame(const Register &reg);
    void handleFailureWithHandler(void *handler);

    // Ion really does mean this to check that the stack is aligned, but
    // we forcibly align the stack on every ABI call and we only need to
    // verify it there, so this is a no-op or it traps all the damn time.
    void checkStackAlignment() {
        // no-op
    }
    void checkStackAlignmentPriorToABICall() {
#ifdef DEBUG
        ispew("checkStackAlignmentPriorToABICall()");

        Label ok;

        // Ensure that the stack is quadword aligned, and trap if not.
        andi_rc(tempRegister, stackPointerRegister, 15);
        bc(Zero, &ok);
        x_trap();
        bind(&ok);
#endif
        // no-op in opt builds
    }

    CodeOffsetLabel labelForPatch() {
        return CodeOffsetLabel(currentOffset());
    }

  protected:
    MoveResolver moveResolver_;

  private:
    // We're big endian!!
    Operand payloadOf(const Address &address) {
        return Operand(address.base, address.offset + 4);
    }
    Operand tagOf(const Address &address) {
        return Operand(address.base, address.offset);
    }

  public:
    void setupABICall(uint32_t args);

    enum Result {
        GENERAL,
        DOUBLE
    };

    typedef MoveResolver::MoveOperand MoveOperand;
    typedef MoveResolver::Move Move;

    bool oom() const {
        return Assembler::oom() || !enoughMemory_;
    }

    // Again, we're big-endian, so this is the reverse of x86 and ARM7L.
    // These are mostly for reference. We don't actually call them (much).
    // Remember, payload is displaced in memory, not the tag.
    Operand ToType(Operand base) {
        return base;
    }
    Operand ToPayload(Operand base) {
        switch (base.kind()) {
          case Operand::REG_DISP:
            return Operand(Register::FromCode(base.base()),
                base.disp() + sizeof(void *));

          case Operand::REG:
            // Payloads are stored in memory by definition.
            JS_NOT_REACHED("GPReg not supported for payloads");

          case Operand::FPREG:
            JS_NOT_REACHED("FPReg not supported for payloads");
            return base;

          default:
            JS_NOT_REACHED("unexpected operand kind");
            return base; // Silence GCC warning.
        }
    }

    // moveValue: load a value into registers

    void moveValue(const Value &val, Register type, Register data) {
        ispew("moveValue(jsval, reg, reg)");

        // Split a jsval into type and data registers.
        jsval_layout jv = JSVAL_TO_IMPL(val);
        x_li32(type, (uint32_t)jv.s.tag);
        if (val.isMarkable()) {
            ImmGCPtr p(reinterpret_cast<gc::Cell *>(val.toGCThing()));
            writeDataRelocation(p);
            x_li32(data, p.value);
        } else
            x_li32(data, (uint32_t)jv.s.payload.i32);
    }
    void moveValue(const Value &val, const ValueOperand &dest) {
        ispew("moveValue(jsval, vo)");

        moveValue(val, dest.typeReg(), dest.payloadReg());
    }
    void moveValue(const ValueOperand &src, const ValueOperand &dest) {
        ispew("moveValue(vo, vo)");
        JS_ASSERT(src.typeReg() != dest.payloadReg());
        JS_ASSERT(src.payloadReg() != dest.typeReg());

        if (dest.payloadReg() != src.payloadReg())
            x_mr(dest.payloadReg(), src.payloadReg());
        if (dest.typeReg() != src.typeReg())
            x_mr(dest.typeReg(), src.typeReg());
    }

    // storeValue: store a value into memory (specified by an Operand)
    // split into storePayload and storeTypeTag

    void storePayload(Register src, Operand dest) {
        ispew("storePayload(reg, o)");
        
        switch(dest.kind()) {
            case Operand::REG:
                if (dest.reg() != src.code())
                    x_mr(Register::FromCode(dest.reg()), src);
                break;
            // Reg + displacement, so the offset may overflow.
            case Operand::REG_DISP:
                if (PPC_IMMOFFS_OK(dest.disp()+4)) {
                    stw(src, dest.base(), dest.disp()+4);
                } else {
                    JS_ASSERT(src != addressTempRegister);
                    JS_ASSERT(dest.base() != addressTempRegister.code());

                    x_li32(addressTempRegister, dest.disp()+4);
                    // cracked on G5
                    stwx(src, dest.base(), addressTempRegister);
                }
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported as payload operand");
                return;
            default:
                JS_NOT_REACHED("unexpected payload operand type");
                return;
        }
    }
    void storePayload(const Value &val, Operand dest) {
        ispew("storePayload(jsval, o)");

        jsval_layout jv = JSVAL_TO_IMPL(val);
        if (val.isMarkable()) {
            ImmGCPtr p((gc::Cell *)jv.s.payload.ptr);
            writeDataRelocation(p);
            x_li32(tempRegister, p.value);
            storePayload(tempRegister, dest);
        } else {
            x_li32(tempRegister, (uint32_t)jv.s.payload.i32);
            storePayload(tempRegister, dest);
        }
    }

    // For convenience.
    void storeTypeTag(Register src, Operand dest) {
        ispew("storeTypeTag(reg, o)");
        
        switch(dest.kind()) {
            case Operand::REG:
                if (dest.reg() != src.code())
                    x_mr(Register::FromCode(dest.reg()), src);
                break;
            // Reg + displacement, so the offset may overflow.
            case Operand::REG_DISP:
                if (PPC_IMMOFFS_OK(dest.disp())) {
                    stw(src, dest.base(), dest.disp());
                } else {
                    JS_ASSERT(src != addressTempRegister);
                    JS_ASSERT(dest.base() != addressTempRegister.code());

                    x_li32(addressTempRegister, dest.disp());
                    // cracked on G5
                    stwx(src, dest.base(), addressTempRegister);
                }
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported as type operand");
                return;
            default:
                JS_NOT_REACHED("unexpected type operand type");
                return;
        }
    }
    void storeTypeTag(ImmTag tag, Operand dest) {
        ispew("storeTypeTag(imm, o)");

        x_li32(tempRegister, tag.value);
        storeTypeTag(tempRegister, dest);
    }

    void storeValue(ValueOperand val, Operand dest) {
        ispew("[[ storeValue(vo, o)");

        storePayload(val.payloadReg(), dest);
        // Figure out the type. ("ToType")
        // We can't really use storeTypeTag here (see REG below).
        switch(dest.kind()) {
            case Operand::REG:
                // Can't fit two registers into one.
                JS_NOT_REACHED("Store of ValueOperand to Operand REG");
                break;
            case Operand::REG_DISP:
                // Displacement only.
                if (PPC_IMMOFFS_OK(dest.disp())) {
                    stw(val.typeReg(), dest.base(), dest.disp());
                } else {
                    JS_ASSERT(val.typeReg() != addressTempRegister);
                    JS_ASSERT(dest.base() != addressTempRegister.code());

                    x_li32(addressTempRegister, dest.disp());
                    // cracked on G5
                    stwx(val.typeReg(), dest.base(), addressTempRegister);
                }
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported as type operand");
                return;
            default:
                JS_NOT_REACHED("unexpected type operand type");
                return;
        }
        ispew("   storeValue(vo, o) ]]");
    }
    void storeValue(ValueOperand val, const Address &dest) {
        ispew("storeValue(vo, adr)");
        storeValue(val, Operand(dest));
    }

    // Emit code to load the equivalent displacement for a generic base index
    // into addressTempRegister, modified by an offset (similar to baseIndex
    // toRegister). Offset must fit within an immediate.
    // If we have another register already ready, then just use that.
    // TODO: This should be smart enough to optimize out the second shift
    // in this circumstance:
/*
[Codegen] bfffe1fc --- slwi r12,r14,3
[Codegen] bfffe200 --- lwzx r6,r12,r5
                                      << since offset is zero, we can
[Codegen] bfffe204 --- slwi r12,r14,3 << eliminate this instruction
[Codegen] bfffe208 --- addi r12,r12,4 (0x4)
[Codegen] bfffe20c --- lwzx r5,r12,r5
*/

    Register baseIndexToRegisterWithOffset(const BaseIndex &address,
            int32_t xoff) {
        if (address.scale == TimesOne) {
            if ((address.offset + xoff) == 0) 
                return address.index;
            add32(Imm32(address.offset + xoff),
                address.index, addressTempRegister);
        } else {
            JS_ASSERT(PPC_IMM_OK_S(xoff));

            if (PPC_OFFS_OK(address)) {
                x_slwi(addressTempRegister, address.index,
                                    address.scale);
                if ((address.offset + xoff) != 0) {
                    if (PPC_IMM_OK_S(xoff + address.offset)) {
                        // Try to save an add here.
                        addi(addressTempRegister, addressTempRegister,
                                (address.offset + xoff));
                    } else {
                        addi(addressTempRegister, addressTempRegister,
                                 address.offset);
                        addi(addressTempRegister, addressTempRegister, xoff);
                    }
                }
            } else {
                // Can't use r0 as a displacement, so we have to swap things up.
                ispew("!!baseIndexToRegister needed emergencyTempRegister");
                x_slwi(emergencyTempRegister,
                    address.index, address.scale);
                x_li32(addressTempRegister, (address.offset + xoff));
                add(addressTempRegister, addressTempRegister,
                                    emergencyTempRegister);
            }
        }
        return addressTempRegister;
    }

    // possible destinations are Address or BaseIndex
    void storeValue(JSValueType type, Register reg, const Address &dest) {
        ispew("storeValue(jsvaltype, reg, adr)");

        // The type tag is in type, and the actual payload is in the register.
        storeTypeTag(ImmTag(JSVAL_TYPE_TO_TAG(type)), Operand(dest));
        storePayload(reg, Operand(dest));
    }
    void storeValue(JSValueType type, Register reg, const BaseIndex &dest) {
        ispew("storeValue(jsvaltype, reg, bi)");
        JS_ASSERT(reg != tempRegister);
        JS_ASSERT(reg != addressTempRegister);
        JS_ASSERT(reg != emergencyTempRegister);

        // We don't implement an Operand type for BaseIndex (yet?).
        // Store the payload first for debugging purposes.
        // cracked on G5
        stwx(reg, dest.base, baseIndexToRegisterWithOffset(dest, 4));
        store32(Imm32(JSVAL_TYPE_TO_TAG(type)), dest);
    }
    void storeValue(const Value &val, const Address &dest) {
        ispew("storeValue(jsval, adr)");

        jsval_layout jv = JSVAL_TO_IMPL(val);
        storeTypeTag(ImmTag(jv.s.tag), Operand(dest));
        storePayload(val, Operand(dest));
    }
    void storeValue(const Value &val, const BaseIndex &dest) {
        ispew("storeValue(jsval, bi)");
        jsval_layout jv = JSVAL_TO_IMPL(val);

        // Store the payload.
        if (val.isMarkable()) {
            ImmGCPtr p((gc::Cell *)jv.s.payload.ptr);
            writeDataRelocation(p);
            x_li32(tempRegister, p.value);
        } else {
            x_li32(tempRegister, (uint32_t)jv.s.payload.i32);
        }
        // cracked on G5
        stwx(tempRegister, dest.base, baseIndexToRegisterWithOffset(dest, 4));

        // Store the type tag.
        store32(Imm32(jv.s.tag), dest);
    }
    void storeValue(ValueOperand val, BaseIndex dest) {
        ispew("storeValue(vo, bi)");

        // cracked on G5
        stwx(val.payloadReg(), dest.base,
            baseIndexToRegisterWithOffset(dest, 4));
        // cracked on G5
        stwx(val.typeReg(), dest.base, baseIndexToRegister(dest));
    }

    // For convenience.
    void loadPayload(const Operand &src, const Register &r) {
        ispew("loadPayload(o, reg)");

        switch(src.kind()) {
            case Operand::REG:
                if (r.code() != src.reg())
                    x_mr(r, Register::FromCode(src.reg()));
                break;
            // Reg + displacement, so the offset may overflow.
            case Operand::REG_DISP:
                if (PPC_IMMOFFS_OK(src.disp()+4)) {
                    lwz(r, src.base(), src.disp()+4);
                } else {
                    JS_ASSERT(src.base() != addressTempRegister.code());

                    x_li32(addressTempRegister, src.disp()+4);
                    lwzx(r, src.base(), addressTempRegister);
                }
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported as payload operand");
                return;
            default:
                JS_NOT_REACHED("unexpected payload operand type");
                return;
        }
    }
    void loadType(const Operand &src, const Register &r) {
        ispew("loadType(o, reg)");

        switch(src.kind()) {
            case Operand::REG:
                if (r.code() != src.reg())
                    x_mr(r, Register::FromCode(src.reg()));
                break;
            case Operand::REG_DISP:
                if (PPC_IMMOFFS_OK(src.disp())) {
                    lwz(r, src.base(), src.disp());
                } else {
                    JS_ASSERT(src.base() != addressTempRegister.code());

                    x_li32(addressTempRegister, src.disp());
                    lwzx(r, src.base(), addressTempRegister);
                }
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported as type operand");
                return;
            default:
                JS_NOT_REACHED("unexpected type operand type");
                return;
        }
    }

    // loadValue: load from memory to a value operand reg dyad
    // The situation may exist where val.*Reg() and the base register are
    // the same register. In fact, this happens a lot in Baseline where
    // the register set is constrained, so we might clobber the address
    // for the second load! We must account for this hazard below.

    void loadValue(Operand src, ValueOperand val) {
        ispew("[[ loadValue(reg, o)");

        // Account for the hazard above by inspecting the operand.
        switch(src.kind()) {
            case Operand::REG:
                if (val.payloadReg().code() == src.reg()) {
                    // Hazard. Load type first.
                    loadType(src, val.typeReg());
                    loadPayload(src, val.payloadReg());
                    ispew("   loadValue(reg, o) ]]");
                    return;
                }
                break;
            case Operand::REG_DISP:
                if (val.payloadReg().code() == src.base()) {
                    // Hazard. Load type first.
                    loadType(src, val.typeReg());
                    loadPayload(src, val.payloadReg());
                    ispew("   loadValue(reg, o) ]]");
                    return;
                }
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported as type operand");
                return;
            default:
                JS_NOT_REACHED("unexpected type operand type");
                return;
        }

        // No hazard, or src.reg()/base() is the typeReg.
        loadPayload(src, val.payloadReg());
        loadType(src, val.typeReg());
        ispew("   loadValue(reg, o) ]]");
    }
    void loadValue(Address src, ValueOperand val) {
        ispew("loadValue(adr, vo)");

        // Account for the hazard by converting to operand, and letting
        // the Operand-based version sweat it.
        loadValue(Operand(src), val);
    }
    void loadValue(const BaseIndex &src, ValueOperand val) {
        ispew("[[ loadValue(bi, vo)");

        // BaseIndex is complicated. There are two different hazards: the
        // one above, and if the baseindex's index register is one of the
        // payload or type registers. However, we *can* assert that:
        JS_ASSERT(src.index != src.base);

        if (src.index == val.payloadReg() || src.base == val.payloadReg()) {
            // Load the typeReg() first.
            lwzx(val.typeReg(), baseIndexToRegister(src), src.base);
            lwzx(val.payloadReg(), baseIndexToRegisterWithOffset(src, 4),
                src.base);
        } else {
            // No such hazard, or typeReg is either src.index or src.base.
            lwzx(val.payloadReg(), baseIndexToRegisterWithOffset(src, 4),
                src.base);
            lwzx(val.typeReg(), baseIndexToRegister(src), src.base);
        }
        ispew("   loadValue(bi, vo) ]]");
    }

    // tagValue: for a given payload, move it into a ValueOperand with
    // supplied tag. I still say this should be renoberateValue.

    void tagValue(JSValueType type, Register payload, ValueOperand dest) {
        ispew("tagValue(jsvaltype, reg, vo)");
        JS_ASSERT(payload != dest.typeReg());

        ImmType t(type);
        x_li32(dest.typeReg(), t.value);
        if (payload != dest.payloadReg())
            x_mr(dest.payloadReg(), payload);
    }

    void popValue(ValueOperand val) {
        ispew("popValue(vo)");

        // Pop type first -- we're big endian!
        pop2(val.typeReg(), val.payloadReg());
    }
    void pushValue(ValueOperand val) {
        ispew("pushValue(vo)");

        // Push payload first -- we're big endian! This forces payload to +4.
        // pushValue does not track framePushed_.
        push2(val.payloadReg(), val.typeReg());
    }

    // For these pushValues:
    // TODO: Interleaved load x_li32 to multiple registers, reordering
    // the instructions. Less useful on OOE implementations, but may make
    // the ordering more explicit for in-order or simpler CPUs.

    void pushValue(const Value &val) {
        ispew("pushValue(jsval)");

        // pushValue does not track framePushed_.
        addi(stackPointerRegister, stackPointerRegister, -8);
        jsval_layout jv = JSVAL_TO_IMPL(val);
        if (val.isMarkable()) {
            ImmGCPtr p(reinterpret_cast<gc::Cell *>(val.toGCThing()));
            writeDataRelocation(p);
            x_li32(tempRegister, p.value);
        } else
            x_li32(tempRegister, jv.s.payload.i32);

        x_li32(addressTempRegister, jv.s.tag);

        // Push payload first -- we're big endian!
        stw(tempRegister, stackPointerRegister, 4);
        stw(addressTempRegister, stackPointerRegister, 0);
    }
    void pushValue(const Address &addr) {
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
    }
    void pushValue(JSValueType type, Register reg) {
        ispew("pushValue(jsvaltype, reg)");

        addi(stackPointerRegister, stackPointerRegister, -8);
        x_li32(tempRegister, ImmTag(JSVAL_TYPE_TO_TAG(type)).value);
        stw(tempRegister, stackPointerRegister, 4);
        stw(reg, stackPointerRegister, 0);
    }

    void movePtr(const Register &src, const Register &dest) {
        ispew("movePtr(reg, reg)");
        if (dest != src)
            x_mr(dest, src);
    }

    // Returns the register containing the type tag.
    Register splitTagForTest(const ValueOperand &value) {
        return value.typeReg();
    }

    // The W0RD is TRUTHINESS! And the T in Colber is silen!
    // For truthiness testing. This is an unsigned compare. This is
    // also abused by branchTestValue.
    void cmp32(const Register &lhs, const ImmTag &imm) {
        ispew("cmp32(UNSIGNED reg,immtag)");

        if (PPC_IMM_OK_U(imm.value)) {
            cmplwi(lhs, uint16_t(imm.value & 0xffff));
        } else {
            Register temp = (lhs == tempRegister) ? addressTempRegister :
                tempRegister;
            x_li32(temp, imm.value);
            cmplw(lhs, temp);
        }
    }

    void emitSet(Assembler::Condition cond, const Register &dest) {
        // If the result of the condition is true, set dest to 1; else
        // set it to zero.
        //
        // Btw: WTF, Mozilla, WTFF?! Can you document these things?! How the
        // #$%& would anyone figure out that's what this does without poring
        // over the Ion code generator? kthxbai
        //
        // TODO: it appears that the condition set is Equal, NotEqual and
        // LessThan. We could eliminate a branch if we just extracted condreg
        // fields and dumped them in dest. Assert here to see if that's so.
        Label end;
        Label ifFalse;

        x_li(dest, 1);
        bc(cond, &end);
        bind(&ifFalse);
        xor_(dest, dest, dest);

        bind(&end);
    }
    void emitSet(Assembler::DoubleCondition cond, const Register &dest) {
        // Almost exactly the same.
        Label end;
        Label ifFalse;

        x_li(dest, 1);
        bc(cond, &end);
        bind(&ifFalse);
        xor_(dest, dest, dest);

        bind(&end);
    }

    // TODO: make these templates
    Condition testUndefined(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_UNDEFINED));
        return cond;
    }
    Condition testUndefined(Condition cond, const BaseIndex &bi) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(bi, ImmTag(JSVAL_TAG_UNDEFINED));
        return cond;
    }
    Condition testBoolean(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_BOOLEAN));
        return cond;
    }
    Condition testBoolean(Condition cond, const BaseIndex &bi) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(bi, ImmTag(JSVAL_TAG_BOOLEAN));
        return cond;
    }
    Condition testInt32(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testInt32(Condition cond, const BaseIndex &bi) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(bi, ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testDouble(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
        Condition actual = (cond == Equal) ? Below : AboveOrEqual;
        cmp32(tag, ImmTag(JSVAL_TAG_CLEAR));
        return actual;
    }
    Condition testDouble(Condition cond, const Address &addr) {
        JS_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
        Condition actual = (cond == Equal) ? Below : AboveOrEqual;
        cmp32(Operand(addr), ImmTag(JSVAL_TAG_CLEAR));
        return actual;
    }
    Condition testDouble(Condition cond, const BaseIndex &bi) {
        JS_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
        Condition actual = (cond == Equal) ? Below : AboveOrEqual;
        cmp32(bi, ImmTag(JSVAL_TAG_CLEAR));
        return actual;
    }
    Condition testNull(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_NULL));
        return cond;
    }
    Condition testNull(Condition cond, const BaseIndex &bi) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(bi, ImmTag(JSVAL_TAG_NULL));
        return cond;
    }
    Condition testString(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_STRING));
        return cond;
    }
    Condition testString(Condition cond, const BaseIndex &bi) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(bi, ImmTag(JSVAL_TAG_STRING));
        return cond;
    }
    Condition testObject(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_OBJECT));
        return cond;
    }
    Condition testObject(Condition cond, const BaseIndex &bi) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(bi, ImmTag(JSVAL_TAG_OBJECT));
        return cond;
    }
    Condition testNumber(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_UPPER_INCL_TAG_OF_NUMBER_SET));
        return cond == Equal ? BelowOrEqual : Above;
    }

    Condition testGCThing(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }
    Condition testGCThing(Condition cond, const Address &address) {
        ispew("testGCThing(cond, adr)");
        JS_ASSERT(cond == Equal || cond == NotEqual);
        
        // tag is at the base address; payload is +4
        load32(address, tempRegister);
        cmp32(tempRegister, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }
    Condition testGCThing(Condition cond, const BaseIndex &address) {
        ispew("testGCThing(cond, bi)");
        JS_ASSERT(cond == Equal || cond == NotEqual);

        load32(address, tempRegister);
        cmp32(tempRegister, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }

    Condition testMagic(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testMagic(Condition cond, const Address &address) {
        ispew("testMagic(cond, adr)");
        JS_ASSERT(cond == Equal || cond == NotEqual);

        load32(address, tempRegister);
        cmp32(tempRegister, ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testMagic(Condition cond, const BaseIndex &address) {
        ispew("[[ testMagic(cond, bi)");
        JS_ASSERT(cond == Equal || cond == NotEqual);

        load32(address, tempRegister);
        cmp32(tempRegister, ImmTag(JSVAL_TAG_MAGIC));
        ispew("   testMagic(cond, bi) ]]");
        return cond;
    }
    Condition testMagic(Condition cond, const Operand &src) {
        ispew("[[ testMagic(cond, o)");
        JS_ASSERT(cond == Equal || cond == NotEqual);

        // Type is always same location as operand
        loadType(src, tempRegister);
        cmp32(tempRegister, ImmTag(JSVAL_TAG_MAGIC));
        ispew("   testMagic(cond, o) ]]");
        return cond;
    }

    Condition testPrimitive(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_UPPER_EXCL_TAG_OF_PRIMITIVE_SET));
        return cond == Equal ? Below : AboveOrEqual;
    }
    Condition testError(Condition cond, const Register &tag) {
        return testMagic(cond, tag);
    }

    Condition testInt32(Condition cond, const Operand &src) {
        ispew("[[ testInt32(cond, o)");
        JS_ASSERT(cond == Equal || cond == NotEqual);

        loadType(src, tempRegister);
        cmp32(tempRegister, ImmTag(JSVAL_TAG_INT32));
        ispew("   testInt32(cond, o) ]]");
        return cond;
    }
    Condition testInt32(Condition cond, const Address &addr) {
        ispew("testInt32(cond, adr)");
        return testInt32(cond, Operand(addr));
    }
    // For the templates below.
    Condition testInt32(Condition cond, const ValueOperand &value) {
        return testInt32(cond, value.typeReg());
    }
    Condition testUndefined(Condition cond, const ValueOperand &value) {
        return testUndefined(cond, value.typeReg());
    }
    Condition testBoolean(Condition cond, const ValueOperand &value) {
        return testBoolean(cond, value.typeReg());
    }
    Condition testDouble(Condition cond, const ValueOperand &value) {
        return testDouble(cond, value.typeReg());
    }
    Condition testNull(Condition cond, const ValueOperand &value) {
        return testNull(cond, value.typeReg());
    }
    Condition testString(Condition cond, const ValueOperand &value) {
        return testString(cond, value.typeReg());
    }
    Condition testObject(Condition cond, const ValueOperand &value) {
        return testObject(cond, value.typeReg());
    }
    Condition testMagic(Condition cond, const ValueOperand &value) {
        return testMagic(cond, value.typeReg());
    }
    Condition testError(Condition cond, const ValueOperand &value) {
        return testMagic(cond, value);
    }
    Condition testNumber(Condition cond, const ValueOperand &value) {
        return testNumber(cond, value.typeReg());
    }
    Condition testGCThing(Condition cond, const ValueOperand &value) {
        return testGCThing(cond, value.typeReg());
    }
    Condition testPrimitive(Condition cond, const ValueOperand &value) {
        return testPrimitive(cond, value.typeReg());
    }

    void branchTestValue(Condition cond, const ValueOperand &value,
        const Value &v, Label *label) {
        ispew("[[ branchTestValue(cond, vo, v, l)");
        JS_ASSERT(cond == Equal || cond == NotEqual);

        jsval_layout jv = JSVAL_TO_IMPL(v);
        if (v.isMarkable()) {
            ImmGCPtr ptr(reinterpret_cast<gc::Cell *>(v.toGCThing()));
            writeDataRelocation(ptr);

            // Use unsigned compare. TODO: Again, we need to force a load here
            // so that TraceDataRelocations has something to trace.
            JS_ASSERT(value.payloadReg() != tempRegister);
            x_li32(tempRegister, ptr.value);
            cmplw(value.payloadReg(), tempRegister);            
        } else
            // Use unsigned compare, so convert it to a Tag.
            cmp32(value.payloadReg(), ImmTag(jv.s.payload.i32));

        if (cond == Equal) {
            Label done;
            bc(NotEqual, &done);
            {
                cmp32(value.typeReg(), ImmTag(jv.s.tag));
                bc(Equal, label);
            }
            bind(&done);
        } else {
            bc(NotEqual, label);

            cmp32(value.typeReg(), ImmTag(jv.s.tag));
            bc(NotEqual, label);
        }
        ispew("   branchTestValue(cond, vo, v, l) ]]");
    }
    void branchTestValue(Condition cond, const Address &addr,
        const ValueOperand &vo, Label *label) {
        ispew("[[ branchTestValue(cond, adr, vo, l)");
        JS_ASSERT(cond == Equal || cond == NotEqual);
        JS_ASSERT(vo.typeReg() != tempRegister);
        JS_ASSERT(vo.payloadReg() != tempRegister);
        JS_ASSERT(vo.typeReg() != addressTempRegister);
        JS_ASSERT(vo.payloadReg() != addressTempRegister);

        // Get the effective address into a register.
        Register basebase = computeEAToRegister(addr);

        // Start with tag (+0) and then payload (+4).
        lwz(tempRegister, basebase, 0);
        cmplw(vo.typeReg(), tempRegister);
        // Cheap as free cmplw! Do the load while the cmplw finishes.
        lwz(tempRegister, basebase, 4);
        if (cond == Equal) {
            Label done;
            bc(NotEqual, &done);
            {
                cmplw(vo.payloadReg(), tempRegister);
                bc(Equal, label);
            }
            bind(&done);
        } else {
            bc(NotEqual, label);
            cmplw(vo.payloadReg(), tempRegister);
            bc(NotEqual, label);
        }
                
        ispew("   branchTestValue(cond, adr, vo, l) ]]");
    }

    // For convenience
    void cmpPtr(Register lhs, uint32_t imm) {
        if (PPC_IMM_OK_U(imm)) {
            cmplwi(lhs, int16_t(imm & 0xffff));
        } else {
            JS_ASSERT(lhs != tempRegister);
            x_li32(tempRegister, imm);
            cmplw(lhs, tempRegister);
        }
    }
    void cmpPtr(Register lhs, const ImmWord rhs) {
        ispew("cmpPtr(reg, immw)");
        cmpPtr(lhs, rhs.value);
    }
    void cmpPtr(Register lhs, const ImmGCPtr rhs) {
        ispew("cmpPtr(reg, immgcptr)");

        // TODO: Because the GCPtr can be garbage collected, we have to emit
        // a lis/ori or equivalent or TraceDataRelocations will fail. Fix that.
        // We also need equivalent code for cmpPtrOperand (below).
        JS_ASSERT(lhs != tempRegister);
        writeDataRelocation(rhs);
        x_li32(tempRegister, rhs.value);
        cmplw(lhs, tempRegister);
    }

    // Generic. Required to account for the GC situation (see above).
    void cmpPtrOperand(const Operand &src, uint32_t rhs, bool isgc) {
        ispew("cmpPtr(o, immw)");

        // Get operand value into tempRegister.
        switch(src.kind()) {
            case Operand::REG:
                if (isgc) {
                    JS_ASSERT(src.reg() != tempRegister.code());
                    writeDataRelocation(ImmGCPtr(rhs));
                    x_li32(tempRegister, rhs);
                    cmplw(Register::FromCode(src.reg()), tempRegister);
                } else {
                    cmpPtr(Register::FromCode(src.reg()), rhs);
                }
                return; // break;
            case Operand::REG_DISP:
                JS_ASSERT(src.base() != addressTempRegister.code());
                if (PPC_IMMOFFS_OK(src.disp())) {
                    lwz(tempRegister, src.base(), src.disp());
                } else {
                    x_li32(addressTempRegister, src.disp());
                    lwzx(tempRegister, src.base(), addressTempRegister);
                }
                if (isgc)
                    writeDataRelocation(ImmGCPtr(rhs));
                x_li32(addressTempRegister, rhs);
                cmplw(tempRegister, addressTempRegister);
                return;
            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported as type operand");
                return;
            default:
                JS_NOT_REACHED("unexpected type operand type");
                return;
        }
    }
    void cmpPtr(const Operand &src, const ImmWord rhs) {
        cmpPtrOperand(src, rhs.value, false);
    }
    void cmpPtr(const Operand &lhs, const ImmGCPtr rhs) {
        ispew("cmpPtr(o, immgcptr)");
        cmpPtrOperand(lhs, rhs.value, true);
    }
    void cmpPtr(const Address &address, const ImmGCPtr rhs) {
        ispew("cmpPtr(a, immgcptr)");
        cmpPtrOperand(Operand(address), rhs.value, true);
    }
    void cmpPtr(Register lhs, Register rhs) {
        cmplw(lhs, rhs);
    }
    void cmpPtr(const Address &lhs, Register rhs) {
        ispew("cmpPtr(adr, reg)");
        JS_ASSERT(rhs != tempRegister);

        load32(lhs, tempRegister);
        cmplw(tempRegister, rhs);
    }
    void cmpPtr(const Address &lhs, const ImmWord rhs) {
        ispew("cmpPtr(adr, immw)");
        
        load32(lhs, tempRegister); // this may clobber addressTempRegister
        x_li32(addressTempRegister, rhs.value); // this WILL clobber it :)
        cmplw(tempRegister, addressTempRegister);
    }
    void cmpPtr(const AbsoluteAddress &address, Register rhs) {
        ispew("cmpPtr(aadr, reg)");
        JS_ASSERT(rhs != tempRegister);

        x_li32(addressTempRegister, (uint32_t)address.addr);
        lwz(tempRegister, addressTempRegister, 0);
        cmplw(tempRegister, rhs);
    }

    void testPtr(Register lhs, Register rhs) {
        ispew("testPtr(reg, reg)");

        and_rc(tempRegister, lhs, rhs);
    }

    Condition testNegativeZero(const FloatRegister &reg, const Register
        &scratch) {
            // FP negative zero is 0x8000 0000 0000 0000 in the IEEE 754
            // standard, so test the upper 32 bits by extracting it on the
            // stack (similar to our various breakDouble iteractions). We have
            // no constant to compare against, so this is the best option.
            stfdu(reg, stackPointerRegister, -8);
#ifdef _PPC970_
            x_nop();
            x_nop(); // cracked
#endif
            lwz(tempRegister, stackPointerRegister, 0); // upper 32 bits
            and_rc(tempRegister, tempRegister, tempRegister);
            addi(stackPointerRegister, stackPointerRegister, 8);
            return NonZero;
    }

    void reserveStack(uint32_t amount) {
        ispew("reserveStack(u32)");
        JS_ASSERT(PPC_IMM_OK_S(amount)); // signed immediate instruction
        if (amount)
            x_subi(stackPointerRegister, stackPointerRegister, amount);
        framePushed_ += amount;
    }
    void freeStack(uint32_t amount) {
        ispew("freeStack(u32)");
        JS_ASSERT(amount <= framePushed_);
        JS_ASSERT(PPC_IMM_OK_S(amount)); // signed immediate instruction
        if (amount)
            addi(stackPointerRegister, stackPointerRegister, amount);
        framePushed_ -= amount;
    }
    void freeStack(Register amount) {
        ispew("freeStack(reg)");
        add(stackPointerRegister, stackPointerRegister, amount);
    }

    void addPtr(const Register &src, const Register &dest) {
        ispew("addPtr(reg, reg)");
        add(dest, src, dest);
    }
    void addPtr(Imm32 imm, const Register &dest) {
        ispew("addPtr(imm, reg)");
        add32(imm, dest, dest);
    }
    void addPtr(ImmWord imm, const Register &dest) {
        ispew("addPtr(immw, reg)");
        add32(Imm32(imm.value), dest, dest);
    }
    void addPtr(Imm32 imm, const Address &dest) {
        ispew("addPtr(imm, adr)");
        add32(imm, dest);
    }
    void addPtr(Address adr, const Register &dest) {
        ispew("[[ addPtr(adr, reg)");

        JS_ASSERT(dest != tempRegister);
        JS_ASSERT(adr.base != tempRegister);

        load32(adr, tempRegister);
        add(dest, dest, tempRegister);

        ispew("   addPtr(adr, reg) ]]");
    }
    void subPtr(Imm32 imm, const Register &dest) {
        ispew("subPtr(imm, reg)");
        add32(Imm32(-(imm.value)), dest, dest);
    }

    // Cond, (Address | Register), (Register | ImmGCPtr | ImmWord), Label
    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, Label *label) {
        ispew("branchPtr(cond, T, S, l)");
        // XXX Might need a couple more variants for this
        cmpPtr(lhs, ptr);
        bc(cond, label);
    }

    // (Address | Register)
    template <typename T>
    void branchPrivatePtr(Condition cond, T lhs, ImmWord ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }
    void branchPrivatePtr(Condition cond, const Address &lhs, Register r, Label *label) {
        branchPtr(cond, lhs, r, label);
    }

    // Cond, (Address | Register), (Register | ImmGCPtr | ImmWord), Label
    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, RepatchLabel *label) {
        ispew("branchPtr(cond, T, S, rl)");
        // XXX Might need a couple more variants for this
        cmpPtr(lhs, ptr);
        bc(cond, label);
    }

    CodeOffsetJump jumpWithPatch(RepatchLabel *label) {
        ispew("jumpWithPatch(rl)");
        CodeOffsetJump j = currentOffset();
        b(label);
        return j;
    }
    // Cond, (Address | Register), (Register | ImmGCPtr | ImmWord), Label
    template <typename S, typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, S lhs, T ptr, RepatchLabel *label) {
// XXX?
        ispew("branchPtrWithPatch(cond, S, T, rl)");
        branchPtr(cond, lhs, ptr, label);
        return CodeOffsetJump(currentOffset());
    }
    void branchPtr(Condition cond, Register lhs, Register rhs, RepatchLabel *label) {
// XXX?
        ispew("branchPtr(cond, reg, reg, rl)");
        cmplw(lhs, rhs);
        bc(cond, label);
    }
    void branchPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        ispew("branchPtr(cond, reg, reg, l)");
        cmplw(lhs, rhs);
        bc(cond, label);
    }
    void branchTestPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        ispew("branchTestPtr(cond, reg, reg, l)");
        and_rc(tempRegister, lhs, rhs);
        bc(cond, label);
    }
    void branchTestPtr(Condition cond, Register lhs, Imm32 rhs, Label *label) {
        ispew("branchTestPtr(cond, reg, imm32, l)");
        JS_ASSERT(lhs != tempRegister);

        x_li32(tempRegister, rhs.value);
        and_rc(tempRegister, lhs, tempRegister);
        bc(cond, label);
    }
    void branchTestPtr(Condition cond, Register lhs, Address rhs, Label *label) {
        ispew("[[ branchTestPtr(cond, reg, adr, l)");
        JS_ASSERT(lhs != tempRegister);

        load32(rhs, tempRegister);
        and_rc(tempRegister, lhs, tempRegister);
        bc(cond, label);
        ispew("   branchTestPtr(cond, reg, adr, l) ]]");
    }
    // ARGH, HOW MANY FREAKING VARIANTS DO YOU HAVE?!
    void branchTestPtr(Condition cond, Address lhs, Imm32 rhs, Label *label) {
        ispew("[[ branchTestPtr(cond, adr, imm32, l)");

        load32(lhs, tempRegister);
        x_li32(addressTempRegister, rhs.value);
        and_rc(tempRegister, addressTempRegister, tempRegister);
        bc(cond, label);
        ispew("   branchTestPtr(cond, adr, imm32, l) ]]");
    }

    void decBranchPtr(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        ispew("decBranchPtr(cond, reg, imm, l)");

        // This is something like bdnz, but to another register, and by more
        // than one. For this, we'll need subf_rc. (There is no subfic_rc.)
        // TODO: See if, in the degenerate lhs == 1 case, mtctr/bdnz can be
        // faster than this, or maybe addme, but we'd need to have carry clear.
        JS_ASSERT(lhs != tempRegister);
        x_li32(tempRegister, imm.value);
        subf_rc(lhs, tempRegister, lhs);
        bc(cond, label);
    }

    void movePtr(ImmWord imm, Register dest) {
        ispew("movePtr(immw, reg)");
        x_li32(dest, imm.value);
    }
    void movePtr(ImmGCPtr imm, Register dest) {
        ispew("movePtr(immgcptr, dest)");
        writeDataRelocation(imm);
        x_li32(dest, imm.value);
    }
    void loadPtr(const Address &address, Register dest) {
        ispew("loadPtr(adr, reg)");
        load32(address, dest);
    }
    void loadPtr(const BaseIndex &src, Register dest) {
        ispew("loadPtr(bi, reg)");
        load32(src, dest);
    }
    void loadPtr(const AbsoluteAddress &address, Register dest) {
        ispew("loadPtr(aadr, reg)");

        x_li32(addressTempRegister, (uint32_t)address.addr);
        lwz(dest, addressTempRegister, 0);
    }
    void loadPrivate(const Address &src, Register dest) {
        ispew("loadPrivate(adr, reg)");

        // The private portion of the stored value at address is the
        // payload (therefore +4 for us). So just load that.
        if (PPC_IMMOFFS_OK(src.offset+4)) {
            lwz(dest, src.base, src.offset+4);
        } else {
            JS_ASSERT(src.base != addressTempRegister);

            x_li32(addressTempRegister, (src.offset + 4));
            lwzx(dest, src.base, addressTempRegister);
        }
    }
    void storePtr(ImmWord imm, const Address &address) {
        store32(Imm32(imm.value), address);
    }
    void storePtr(ImmGCPtr imm, const Address &address) {
        writeDataRelocation(imm);
        x_li32(tempRegister, imm.value);
        store32(tempRegister, address);
    }
    void storePtr(Register src, const Address &address) {
        store32(src, address);
    }
    void storePtr(Register src, const AbsoluteAddress &address) {
        ispew("storePtr(reg, aadr)");

        JS_ASSERT(src != addressTempRegister);
        x_li32(addressTempRegister, (uint32_t)address.addr);
        stw(src, addressTempRegister, 0);
    }

    // Get argth slot from the stack into the register.
    void setStackArg(const Register &reg, uint32_t arg) {
        ispew("setStackArg(reg, u32)");

        JS_ASSERT(PPC_IMMOFFS_OK(arg * STACK_SLOT_SIZE));
        lwz(reg, stackPointerRegister, (arg * STACK_SLOT_SIZE));
    }

    // Type testing instructions can take a tag in a register or a
    // ValueOperand.
    template <typename T>
    void branchTestUndefined(Condition cond, const T &t, Label *label) {
        cond = testUndefined(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestInt32(Condition cond, const T &t, Label *label) {
        cond = testInt32(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestBoolean(Condition cond, const T &t, Label *label) {
        cond = testBoolean(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestDouble(Condition cond, const T &t, Label *label) {
        cond = testDouble(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestNull(Condition cond, const T &t, Label *label) {
        cond = testNull(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestString(Condition cond, const T &t, Label *label) {
        cond = testString(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestObject(Condition cond, const T &t, Label *label) {
        cond = testObject(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestNumber(Condition cond, const T &t, Label *label) {
        cond = testNumber(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestGCThing(Condition cond, const T &t, Label *label) {
        cond = testGCThing(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestPrimitive(Condition cond, const T &t, Label *label) {
        cond = testPrimitive(cond, t);
        bc(cond, label);
    }
    template <typename T>
    void branchTestMagic(Condition cond, const T &t, Label *label) {
        cond = testMagic(cond, t);
        bc(cond, label);
    }

    void branchTestMagicValue(Condition cond, const ValueOperand &val,
            JSWhyMagic why, Label *label)
    {
        // Oh, oh, oh, it's MAGIC! Or NO! Then we will branch if it's SO!
        JS_ASSERT(cond == Equal || cond == NotEqual);

        if (cond == Equal) {
            // Test for magic, maybe echo if I'm listening to a Rush album.
            Label notmagic;

            Condition testCond = testMagic(Equal, val);
            bc(InvertCondition(testCond), &notmagic);
            // Test magic value
            branch32(Equal, val.payloadReg(),
                Imm32(static_cast<int32_t>(why)), label);
            bind(&notmagic);
        } else {
            // Test for boring.
            Condition testCond = testMagic(NotEqual, val);
            bc(testCond, label);
            branch32(NotEqual, val.payloadReg(),
                Imm32(static_cast<int32_t>(why)), label);
        }
    }

    void boxDouble(const FloatRegister &src, const ValueOperand &dest) {
        ispew("boxDouble(fpr, vo)");
        // Intel handles this by moving the low 32 bits into payloadReg()
        // and the high into typeReg(). We don't have anything like psrldq
        // for the FPU, so just spill to the stack and pull from there.
        // cracked on G5
        stfdu(src, stackPointerRegister, -8);
#ifdef _PPC970_
        // G5 and POWER4+ do better if the stfd and the lwz aren't in the
        // same dispatch group. Since stfdu is cracked, only two nops needed.
        x_nop();
        x_nop();
#endif
        // Remember, payload is displaced, type tag is not!
        lwz(dest.typeReg(), stackPointerRegister, 0);
        lwz(dest.payloadReg(), stackPointerRegister, 4);
        addi(stackPointerRegister, stackPointerRegister, 8);
    }

    void boxNonDouble(JSValueType type, const Register &src, const ValueOperand &dest) {
        ispew("boxNonDouble(jsvaltype, reg, vo)");
        ImmType t(type);

        if (src != dest.payloadReg())
            x_mr(dest.payloadReg(), src);
        x_li32(dest.typeReg(), t.value);
    }
    void unboxInt32(const ValueOperand &src, const Register &dest) {
        ispew("unboxInt32(vo, reg)");
        if (dest != src.payloadReg())
            x_mr(dest, src.payloadReg());
    }
    void unboxInt32(const Address &src, const Register &dest) {
        ispew("unboxInt32(adr, reg) >>");
        loadPayload(Operand(src), dest); // not ToPayload; that makes +8
    }
    void unboxBoolean(const ValueOperand &src, const Register &dest) {
        ispew("unboxBoolean(vo, dest)");
        // Booleans are word-sized on PowerPC, so this is just a 32-bit load.
        if (dest != src.payloadReg())
            x_mr(dest, src.payloadReg());
    }
    void unboxBoolean(const Address &src, const Register &dest) {
        ispew("unboxBoolean(adr, reg) >>");
        loadPayload(Operand(src), dest); // not ToPayload; that makes +8
    }
    void unboxDouble(const ValueOperand &src, const FloatRegister &dest) {
        ispew("unboxDouble(vo, fpr)");

        // Like boxDouble, we have no FPR shift to SSE, so stack it!
        // We can be a little tricky here with stwu. Remember, payloadReg
        // is displaced +4, so it goes on the stack first.
        // cracked on G5. Since they're both cracked, they're in one dispatch
        // group.
        stwu(src.payloadReg(), stackPointerRegister, -4);
        stwu(src.typeReg(), stackPointerRegister, -4);
#ifdef _PPC970_
        // G5 and POWER4+ do better if the stw and the lfd aren't in the
        // same dispatch group. This makes sure the lfd will never be with
        // them together.
        x_nop();
        x_nop();
#endif
        lfd(dest, stackPointerRegister, 0);
        addi(stackPointerRegister, stackPointerRegister, 8);
    }
    void unboxDouble(const Address &addr, const FloatRegister &dest) {
        ispew("unboxDouble(adr, fpr)");

        // If we ever get a boxDouble to an address, we might be able to
        // keep the byte order reversed in memory too. Otherwise, do this
        // the "hard" way so that we keep everything as JS expects.

        // Get the effective address of the boxed double in memory.
        Register basebase = computeEAToRegister(addr);

        // Build the boxed double on the stack, like the above, starting
        // with the payload. There's not much to be gained by going
        // double-barrelled with addressTempRegister, and computeEAToRegister
        // may need it, so just use tempReg.
        lwz(tempRegister, basebase, 4);
        stwu(tempRegister, stackPointerRegister, -4);
        // If basebase is stackPointerRegister, the offset needs to be 4
        // because stwu backed it up by 4 already or we lose the bottom word.
        if (basebase == stackPointerRegister)
            lwz(tempRegister, basebase, 4);
        else
            lwz(tempRegister, basebase, 0);
        stwu(tempRegister, stackPointerRegister, -4);
#ifdef _PPC970_
        // G5 and POWER4+ do better if the stw and the lfd aren't in the
        // same dispatch group. This makes sure the lfd will never be with
        // them together.
        x_nop();
        x_nop();
#endif
        lfd(dest, stackPointerRegister, 0);
        addi(stackPointerRegister, stackPointerRegister, 8);
    }        

    void loadInt32OrDouble(const Operand &src, const FloatRegister &dest) {
        ispew("[[ loadInt32OrDouble(o, fpr)");
        Label notInt32, end;

        // If the tag is not an int32, just load it. Otherwise, convert it.
        branchTestInt32(Assembler::NotEqual, src, &notInt32);

        // It's an int32. Load the payload.
        loadPayload(src, tempRegister); // not ToPayload
        // Convert to FPR.
        convertInt32ToDouble(tempRegister, dest);
        jump(&end);

        bind(&notInt32);
        // The tag is (we assume?) a Double. That means we have to do an
        // operand load for an FPR here. Assume it is already in proper
        // order so we just have to lfd/lfdx it.

        switch(src.kind()) {
            case Operand::REG:
                JS_NOT_REACHED("GPR to FPR move doesn't make sense");
                return;
            case Operand::REG_DISP:
                if (PPC_IMMOFFS_OK(src.disp())) {
                    lfd(dest, src.base(), src.disp());
                } else {
                    x_li32(addressTempRegister, src.disp()+4);
                    lfdx(dest, src.base(), addressTempRegister);
                }
                break;
            case Operand::FPREG:
                JS_NOT_REACHED("FPReg not supported as double operand hurhur");
                return;
            default:
                JS_NOT_REACHED("unexpected payload operand type");
                return;
        }
        bind(&end);
        ispew("   loadInt32OrDouble(o, fpr) ]]");
    }

    // WTF? Why have the MacroAssembler handle a relatively high level
    // operation? But, whatev.
    void unboxValue(const ValueOperand &src, AnyRegister dest) {
        ispew("[[ unboxValue(vo, anyreg)");

        if (dest.isFloat()) {
            Label notInt32, end;
            // If the tag is not an int32, unbox directly to float.
            // If it is, convert it.
            branchTestInt32(Assembler::NotEqual, src, &notInt32);
            convertInt32ToDouble(src.payloadReg(), dest.fpu());
            b(&end);
            bind(&notInt32);
            unboxDouble(src, dest.fpu());
            bind(&end);
        } else {
            if (src.payloadReg() != dest.gpr())
                x_mr(dest.gpr(), src.payloadReg());
        }
        ispew("   unboxValue(vo, anyreg) ]]");
    }
    void unboxPrivate(const ValueOperand &src, Register dest) {
        if (src.payloadReg() != dest)
            x_mr(dest, src.payloadReg());
    }

    // Extended unboxing API. If the payload is already in a register, returns
    // that register. Otherwise, provides a move to the given scratch register,
    // and returns that.
    Register extractObject(const Address &address, Register scratch) {
        // Being a bit evil here.
        loadPayload(Operand(address), scratch);
        return scratch;
    }
    Register extractObject(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    Register extractInt32(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    Register extractBoolean(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    Register extractTag(const Address &address, Register scratch) {
        // More evil!
        loadType(Operand(address), scratch);
        return scratch;
    }
    Register extractTag(const ValueOperand &value, Register scratch) {
        return value.typeReg();
    }

    void boolValueToDouble(const ValueOperand &operand, const FloatRegister &dest) {
        // Just convert it as if it were an int.
        convertInt32ToDouble(operand.payloadReg(), dest);
    }
    void int32ValueToDouble(const ValueOperand &operand, const FloatRegister &dest) {
        // Huh?
        convertInt32ToDouble(operand.payloadReg(), dest);
    }

    void loadStaticDouble(const double *dp, const FloatRegister &dest) {
        ispew("loadStaticDouble(double*, fpr)");
        // Assuming that dp is a pointer to a double, get the address and
        // lfd that directly.
        x_li32(addressTempRegister, (uint32_t)dp);
        lfd(dest, addressTempRegister, 0);
    }

    void loadConstantDouble(double d, const FloatRegister &dest) {
        ispew("loadConstantDouble(double, fpr)");

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
        x_li32(addressTempRegister, (uint32_t)dbl.uses.prev());
        lfd(dest, addressTempRegister, 0);
        dbl.uses.setPrev(masm.size());
    }

    Condition testInt32Truthy(bool truthy, const ValueOperand &operand) {
        ispew("testInt32Truthy(bool, vo)");
        JS_ASSERT(operand.typeReg() != tempRegister);
        JS_ASSERT(operand.payloadReg() != tempRegister);

        and_rc(tempRegister, operand.payloadReg(), operand.payloadReg());
        return truthy ? NonZero : Zero;
    }
    void branchTestBooleanTruthy(bool truthy, const ValueOperand &operand, Label *label) {
        ispew("branchTestBooleanTruthy(bool, vo, l)");
        JS_ASSERT(operand.typeReg() != tempRegister);
        JS_ASSERT(operand.payloadReg() != tempRegister);

        and_rc(tempRegister, operand.payloadReg(), operand.payloadReg());
        bc(truthy ? NonZero : Zero, label);
    }
    Condition testStringTruthy(bool truthy, const ValueOperand &value) {
        ispew("testStringTruthy(bool, vo)");
        JS_ASSERT(value.typeReg() != tempRegister);
        JS_ASSERT(value.payloadReg() != tempRegister);
        JS_ASSERT(value.typeReg() != addressTempRegister);
        JS_ASSERT(value.payloadReg() != addressTempRegister);

        // Get the length word and see if it is non-zero.
        Register string = value.payloadReg();
        if (PPC_IMMOFFS_OK(JSString::offsetOfLengthAndFlags())) {
            lwz(tempRegister, value.payloadReg(),
                JSString::offsetOfLengthAndFlags());
        } else {
            JS_ASSERT(value.payloadReg() != tempRegister); // won't work!
            x_li32(addressTempRegister, JSString::offsetOfLengthAndFlags());
            lwzx(tempRegister, value.payloadReg(), addressTempRegister);
        }
        size_t mask = (0xFFFFFFFF << JSString::LENGTH_SHIFT);
        x_li32(addressTempRegister, mask);
        and_rc(tempRegister, tempRegister, addressTempRegister);
        return truthy ? Assembler::NonZero : Assembler::Zero;
    }

    void loadUnboxedValue(const Address &src, MIRType type, AnyRegister dest) {
        ispew("[[ loadUnboxedValue(adr, mirt, anyreg)");
        if (dest.isFloat())
            loadInt32OrDouble(Operand(src), dest.fpu());
        else
            load32(src, dest.gpr());
        ispew("   loadUnboxedValue(adr, mirt, anyreg) ]]");
    }
    void loadUnboxedValue(const BaseIndex &src, MIRType type, AnyRegister dest) {
        ispew("[[ loadUnboxedValue(bi, mirt, anyreg)");
        if (dest.isFloat()) {
            // Get and compute the effective address in one register.
            add(addressTempRegister, src.base, baseIndexToRegister(src));

            // Now make an operand out of it, and call loadInt32OrDouble.
            loadInt32OrDouble(Operand(addressTempRegister), dest.fpu());
        } else
            load32(src, dest.gpr());
        ispew("   loadUnboxedValue(bi, mirt, anyreg) ]]");
    }

    void rshiftPtr(Imm32 imm, Register dest) {
        JS_ASSERT(imm.value <= 31);
        srawi(dest, dest, imm.value);
    }
    void lshiftPtr(Imm32 imm, Register dest) {
        JS_ASSERT(imm.value <= 31);
        x_slwi(dest, dest, imm.value);
    }
    void orPtr(Imm32 imm, Register dest) {
        ispew("orPtr(imm, reg)");
        JS_ASSERT(dest != tempRegister);

        x_li32(tempRegister, imm.value);
        or_(dest, tempRegister, dest);
    }
    void orPtr(Register src, Register dest) {
        ispew("orPtr(reg, reg)");
        or_(dest, src, dest);
    }
    void xorPtr(Register src, Register dest) {
        ispew("xorPtr(reg, reg)");
        xor_(dest, src, dest);
    }
    void andPtr(Register src, Register dest) {
        ispew("andPtr(reg, reg)");
        and_(dest, src, dest);
    }
    void andPtr(Imm32 imm, const Register &srcdest) {
        ispew("andPtr(imm, reg)");
        and32(imm, srcdest);
    }

    void convertUInt32ToDouble(const Register &src, const FloatRegister &dest) {
        // This is almost the same as convertInt, except the zero constant
        // is 0x4330 0000 0000 0000 (not 8000), and we don't xoris the sign.
        // See also OPPCC chapter 8 p.157.
        ispew("convertUInt32ToDouble(reg, fpr)");

        // It's possible to call this for temp registers, so use spares.
        FloatRegister fpTemp = (dest == fpTempRegister) ? fpConversionRegister
            : fpTempRegister;
        Register temp = (src == tempRegister) ? addressTempRegister 
            : tempRegister;

        // Build temporary frame with zero constant.
        x_lis(temp, 0x4330); 
        stwu(temp, stackPointerRegister, -8); // cracked on G5
        x_lis(temp, 0x0000); // not 8000

        // (2nd G5 dispatch group)
        stw(src, stackPointerRegister, 4);
        // Don't flip integer value's sign; use as integer component directly.
        // stack now has 0x4330 0000 <src>
#ifdef _PPC970_
        // G5 and POWER4+ do better if the lfd and the stws aren't in the
        // same dispatch group.
        x_nop();
        x_nop();
        x_nop();
#endif

        // (3rd G5 dispatch group)
        // Load intermediate float.
        lfd(dest, stackPointerRegister, 0);
#ifdef _PPC970_
        // G5 and POWER4+ do better if the lfd and the stws aren't in the
        // same dispatch group.
        x_nop();
        x_nop();
        x_nop();
#endif

        // (4th G5 dispatch group)
        stw(temp, stackPointerRegister, 4);
        // stack now has 0x4330 0000 0000 0000
#ifdef _PPC970_
        x_nop();
        x_nop();
        x_nop();
#endif

        // (Final G5 dispatch group)
        // Load zero float and normalize with a subtraction operation.
        lfd(fpTemp, stackPointerRegister, 0);
        fsub(dest, dest, fpTemp);
        // Destroy temporary frame.
        addi(stackPointerRegister, stackPointerRegister, 8);
    }

    void inc64(AbsoluteAddress dest) {
        // Get the absolute address, increment its *second* word, and then
        // next word UP if needed because we're big-endian, and therefore
        // awesome.
        // TODO: G5 should kick ass here. Consider ld/addi/nop*/std later.
        // Note that unaligned doubleword loads/stores might take up a full
        // dispatch group if indexed, which could make timing really tricky.

        // (First G5 dispatch group.) Load effective address.
        x_li32(addressTempRegister, (uint32_t)dest.addr); // 2 inst

        // Get the LSB and increment it, setting or clearing carry.
        lwz(tempRegister, addressTempRegister, 4);
        addic(tempRegister, tempRegister, 1);

        // (Second G5 dispatch group.) Store it.
        stw(tempRegister, addressTempRegister, 4);
#ifdef _PPC970_
        // Keep this stw and the following lwz in separate dispatch groups.
        x_nop();
        x_nop();
        x_nop();
#endif

        // (Third G5 dispatch group.) Load MSB and increment it with carry
        // by adding zero. This way we don't need to branch!
        lwz(tempRegister, addressTempRegister, 0);
        addze(tempRegister, tempRegister);
#ifdef _PPC970_
        // Force the next stw into another dispatch group.
        x_nop();
        x_nop();
#endif

        // (Final G5 dispatch group.) Store it; done.
        stw(tempRegister, addressTempRegister, 0);
    }

    // If source is a double, load it into dest. If source is int32,
    // convert it to double. Else, branch to failure. This is an
    // expensive function to call.
    void ensureDouble(const ValueOperand &source, FloatRegister dest, Label *failure) {
        Label isDouble, done;
        branchTestDouble(Assembler::Equal, source.typeReg(), &isDouble);
        branchTestInt32(Assembler::NotEqual, source.typeReg(), failure);
    
        convertInt32ToDouble(source.payloadReg(), dest);
        jump(&done);
    
        bind(&isDouble);
        unboxDouble(source, dest);
    
        bind(&done);
    }

    void x_fsqrt(const FloatRegister &dest, const FloatRegister &src) {
#if defined(_PPC970_)
        fsqrt(dest, src);
#else
        JS_NOT_REACHED("x_fsqrt not yet implemented for non-G5");
#endif
    }

    // Setup a call to C/C++ code, given the number of general arguments it
    // takes.
    void setupAlignedABICall(uint32_t args);
    void setupUnalignedABICall(uint32_t args, const Register &scratch);
    void passABIArg(const MoveOperand &from);
    void passABIArg(const Register &reg);
    void passABIArg(const FloatRegister &reg);

    // Define custom handlers for this, since we need endian support for it
    // or we fail bug 813784.
    void branchIfFunctionHasNoScript(Register fun, Label *label);
    void branchIfInterpreted(Register fun, Label *label);

  private:
    void callWithABIPre(uint32_t *stackAdjust);
    void callWithABIPost(uint32_t stackAdjust, Result result);

  public:
    // Emits a call to a C/C++ function, resolving all argument moves.
    void callWithABI(void *fun, Result result = GENERAL);
    void callWithABI(const Address &fun, Result result = GENERAL);

    void makeFrameDescriptor(Register frameSizeReg, FrameType type);

    // Save an exit frame (which must be aligned to the stack pointer) to
    // ThreadData::ionTop.
    void linkExitFrame();
    void callWithExitFrame(IonCode *target, Register dynStack);

    void enterOsr(Register calleeToken, Register code);
};

typedef MacroAssemblerPPC MacroAssemblerSpecific;

} // namespace jit
} // namespace js

#endif // jsion_macro_assembler_ppc_h__

