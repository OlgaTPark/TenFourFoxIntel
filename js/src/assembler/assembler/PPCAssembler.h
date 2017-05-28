#ifdef _PPC970_
#define USE_970_BRANCHING 1
#endif

/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * ***** BEGIN LICENSE BLOCK *****
 * Version: MPL 1.1/GPL 2.0/LGPL 2.1
 *
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 * for the specific language governing rights and limitations under the
 * License.
 *
 * The Original Code is Mozilla SpiderMonkey JavaScript 1.9 code.
 *
 * The Initial Developer of the Original Code is
 *    Cameron Kaiser <classilla@floodgap.com> and the TenFourFox team
 *    Benjamin Stuhl <bks24@cornell.edu>
 * 
 * Portions created by the Initial Developer are Copyright (C) 2010-2011
 * the Initial Developer. All Rights Reserved.
 *
 * Alternatively, the contents of this file may be used under the terms of
 * either the GNU General Public License Version 2 or later (the "GPL"), or
 * the GNU Lesser General Public License Version 2.1 or later (the "LGPL"),
 * in which case the provisions of the GPL or the LGPL are applicable instead
 * of those above. If you wish to allow use of your version of this file only
 * under the terms of either the GPL or the LGPL, and not to allow others to
 * use your version of this file under the terms of the MPL, indicate your
 * decision by deleting the provisions above and replace them with the notice
 * and other provisions required by the GPL or the LGPL. If you do not delete
 * the provisions above, a recipient may use your version of this file under
 * the terms of any one of the MPL, the GPL or the LGPL.
 *
 * ***** END LICENSE BLOCK ***** */

#ifndef PPCAssembler_h
#define PPCAssembler_h

#include <assembler/wtf/Platform.h>

// Some debug code uses s(n)printf for instruction logging.
#include <stdio.h>

#if ENABLE_ASSEMBLER && WTF_CPU_PPC

#include "AssemblerBufferWithConstantPool.h"
#include <assembler/wtf/Assertions.h>

/* 32-bit PowerPC execution model, compatible to the 603 and up. */

namespace JSC {
    namespace PPCRegisters {
        typedef enum {
            r0 = 0,
            r1, // sp
            sp = r1,
            r2, // toc
            r3, // args and volatile GPRs
            r4,
            r5,
            r6,
            r7,
            r8,
            r9,
            r10,
            r11, // environment
            r12, // linkage
            r13, // non-volatile GPRs
            r14, 
            r15,
            r16,
            r17,
            r18,
            r19,
            r20,
            r21,
            r22,
            r23,
            r24,
            r25,
            r26,
            r27,
            r28,
/*
 * Currently Ion can only handle 29 registers.
 
            r29,
            r30,
            r31,
 */
            invalid_reg = -1
        } RegisterID;

        typedef enum {
            f0 = 0,
            f1,
            f2,
            f3,
            f4,
            f5,
            f6,
            f7,
            f8,
            f9,
            f10,
            f11,
            f12,
            f13,
            f14,
            f15,
            f16,
            f17,
            f18,
            f19,
            f20,
            f21,
            f22,
            f23,
            f24,
            f25,
            f26,
            f27,
            f28,
/*
 * Currently Ion can only handle 29 registers.

            f29,
            f30,
            f31,
 */
            invalid_freg = -1
        } FPRegisterID;

        // PPC special purpose registers (just the ones we need).
        typedef enum {
            xer = 1,
            lr = 8,
            ctr = 9,
            vrsave = 256 // for future SIMD JS?
        } SPRegisterID;

        // PPC conditional registers. We only use this for branches.
        typedef enum {
            cr0 = 0,
            cr1,
            cr2,
            cr3,
            cr4,
            cr5,
            cr6,
            cr7
        } CRegisterID;

    } // namespace PPCRegisters

    class PPCAssembler : public GenericAssembler {
    public:
        typedef PPCRegisters::RegisterID RegisterID;
        typedef PPCRegisters::FPRegisterID FPRegisterID;
        typedef PPCRegisters::SPRegisterID SPRegisterID;
        typedef PPCRegisters::CRegisterID CRegisterID;
        typedef SegmentedVector<int, 64> Jumps;

#if USE_970_BRANCHING
        typedef AssemblerBuffer BufferType;
#else
        // We use the constant pool machinery to reserve space for out-of-line
        // trampolines to enable far jumps and calls.
        typedef AssemblerBufferWithConstantPool<
            16 * 1024 /* pool size in bytes = 16K */,
            4 /* barrier instruction size */,
            4 /* max instruction size */,
            PPCAssembler> BufferType;
#endif
        BufferType m_buffer;

        bool oom() const { return m_buffer.oom(); }

#ifdef JS_METHODJIT_SPEW
        bool isOOLPath;
        PPCAssembler() : isOOLPath(false) { }
#else
        PPCAssembler() { }
#endif

        // PPC conditional constants (fed to bc and bcctr). The upper nybble
        // indicates the bit in the conditional register being tested (e.g.,
        // equal = bit 2). The lower nybble is the branch option for the test.
        // We then need to add desired CR * 4 for the upper nybble. See
        // Optimizing PowerPC Code, appendix A, p.369.
        //
        // We use XER OV, not CR SO, so it is not tracked here.
        
        // Definitions of the branch-option field inside [Double]Condition
        enum {
            BranchOnClear = 0x04,
            BranchOnSet = 0x0c,
            BranchOptionMask = 0x0f,
            BranchOptionInvert = 0x08 // XOR with this to invert the sense of a Condition
        };
        
        enum BranchAddressType {
            RelativeBranch = 0,
            AbsoluteBranch = 2
        };
        
        enum BranchLikelihood {
            NotLikelyBranch = 0,
            LikelyBranch = 1
        };
        
        enum BranchLink {
            DontLinkBranch = 0,
            LinkBranch = 1
        };

        // Integer condition codes
        enum Condition {
            // These are off pp370-1 in OPPCC. The top nybble is the offset
            // to the CR field (the x in BIF*4+x), and the bottom is the BO.
            ConditionEQ  = 0x2c, // Zero
            ConditionNE  = 0x24, // Non-zero
            ConditionGT  = 0x1c,
            ConditionGE  = 0x04,
            ConditionLT  = 0x0c,
            ConditionLE  = 0x14,
            ConditionSO  = 0x3c, // Summary overflow only (not XER)
            ConditionNS  = 0x34,

            // Bit flag for unsigned comparisons (remember that you have to
            // choose the type of comparison at the compare step, not the
            // branch). We just mask this bit off, but the macro assembler
            // uses it as a flag. This is a synthetic code.
            ConditionUnsigned   = 0x100,        // Computation only

            // XER-only codes. We need to have XER in the CR using mcrxr or
            // an equivalent first, but we don't need to check CR itself.
            // This is a synthetic code.
            ConditionOnlyXER    = 0x200,        // Not used, for comparisons.
            ConditionXERCA      = 0x22c,        // same as EQ bit
            ConditionXERNCA     = 0x224,
            ConditionXEROV      = 0x21c,        // same as GT bit
        };

        // Floating-point condition codes: like the integer ones, but
        // also specify handling of unordered results. The normal (not _U)
        // versions are false if the result is unordered, while the _U versions
        // are true if the result is unordered.
        enum DoubleCondition {
            DoubleConditionEQ  = 0x2c, // Zero
            DoubleConditionNE  = 0x24, // Non-zero
            DoubleConditionGT  = 0x1c,
            DoubleConditionGE  = 0x04,
            DoubleConditionLT  = 0x0c,
            DoubleConditionLE  = 0x14,

            // Used by order comparisons (see m_fbranch).
            // FU (Floating-Unordered) is the same bit as SO
            DoubleConditionFU  = 0x3c,
            DoubleConditionFO  = 0x34,

            // Unordered comparisons.
            // If either operand is NaN (i.e., unordered is true), the entire
            // comparison is *true* (regardless of the condition).
            DoubleConditionUnordered    = 0x100,    // Not used, for comparison.
            DoubleConditionEQ_U         = 0x12c,
            DoubleConditionNE_U         = 0x124,
            DoubleConditionGT_U         = 0x11c,
            DoubleConditionGE_U         = 0x104,
            DoubleConditionLT_U         = 0x10c,
            DoubleConditionLE_U         = 0x114
        };

        enum PPCOpcodes {
        // Copied and expanded from nanojit/NativePPC.h
        // Some we don't use yet (but we will).
        PPC_add     = 0x7C000214, // add
        PPC_adde    = 0x7C000114, // add extended
        PPC_addo    = 0x7C000614, // add & OE=1 (can set OV)
        PPC_addi    = 0x38000000, // add immediate
        PPC_addic   = 0x30000000, // add immediate carrying
        PPC_addis   = 0x3C000000, // add immediate shifted
        PPC_addme   = 0x7C0001D4, // add -1 extended
        PPC_addze   = 0x7C000194, // add zero extended
        PPC_and     = 0x7C000038, // and
        PPC_andc    = 0x7C000078, // and with compliment
        PPC_andi    = 0x70000000, // and immediate
        PPC_andis   = 0x74000000, // and immediate shifted
        PPC_b       = 0x48000000, // branch
        PPC_bc      = 0x40000000, // branch conditional
        PPC_bctr    = 0x4E800420, // branch to CTR (+/- LR)
        PPC_bcctr   = 0x4C000420, // branch conditional to count register
        PPC_blr     = 0x4E800020, // branch to link register
        PPC_cmpw    = 0x7C000000, // compare
        PPC_cmpwi   = 0x2C000000, // compare immediate
        PPC_cmplw   = 0x7C000040, // compare logical
        PPC_cmplwi  = 0x28000000, // compare logical immediate
        PPC_cntlzw  = 0x7C000034, // count leading zeroes
        PPC_crand   = 0x4C000202, // condition register and
        PPC_crandc  = 0x4C000102, // condition register and-with-complement
        PPC_cror    = 0x4C000382, // condition register or
        PPC_crorc   = 0x4C000342, // condition register or-with-complement
        PPC_crxor   = 0x4C000182, // condition register xor
        PPC_divw    = 0x7C0003D6, // integer divide
        PPC_divwo   = 0x7C0007D6, // integer divide & OE=1 (can set OV)
        PPC_eqv     = 0x7C000238, // equivalence operator
        PPC_extsb   = 0x7C000774, // extend sign byte
        PPC_extsh   = 0x7C000734, // extend sign halfword
        PPC_extsw   = 0x7C0007B4, // extend sign word
        PPC_fabs    = 0xFC000210, // floating absolute value (double precision)
        PPC_fadd    = 0xFC00002A, // floating add (double precision)
        PPC_fcfid   = 0xFC00069C, // floating convert from integer doubleword
        PPC_fctiw   = 0xFC00001C, // floating convert to integer (to -Inf)
        PPC_fctiwz  = 0xFC00001E, // floating convert to integer (to zero)
        PPC_fcmpu   = 0xFC000000, // floating compare unordered
        PPC_fdiv    = 0xFC000024, // floating divide (double precision)
        PPC_fmr     = 0xFC000090, // floating move register (double precision)
        PPC_fmul    = 0xFC000032, // floating multiply (double precision)
        PPC_fneg    = 0xFC000050, // floating negate
        PPC_frsp    = 0xFC000018, // convert to single precision
        PPC_fsel    = 0xFC00002E, // floating point select
        PPC_fsub    = 0xFC000028, // floating subtract (double precision)
        PPC_fsqrt   = 0xFC00002C, // floating square root (G5 only) (double)
        PPC_frsqrte = 0xFC000034, // floating reciprocal square root estimate
        PPC_fnmsub  = 0xFC00003C, // floating fused negative multiply-subtract
        PPC_fmadd   = 0xFC00003A, // floating fused multiply-add
        PPC_lbz     = 0x88000000, // load byte and zero
        PPC_lbzx    = 0x7C0000AE, // load byte and zero indexed
        PPC_ld      = 0xE8000000, // load doubleword
        PPC_ldx     = 0x7C00002A, // load doubleword indexed
        PPC_lfd     = 0xC8000000, // load floating point double
        PPC_lfdx    = 0x7C0004AE, // load floating-point double indexed
        PPC_lfs     = 0xC0000000, // load single precision float
        PPC_lfsx    = 0x7C00042E, // load single precision float indexed
        PPC_lha     = 0xA8000000, // load halfword algebraic
        PPC_lhax    = 0x7C0002AE, // load halfword algebraic indexed
        PPC_lhz     = 0xA0000000, // load halfword and zero
        PPC_lhzx    = 0x7C00022E, // load halfword and zero indexed
        PPC_lwz     = 0x80000000, // load word and zero
        PPC_lwzx    = 0x7C00002E, // load word and zero indexed
        PPC_mcrxr   = 0x7C000400, // move XER[0-3] to CR[0-3] (G4 and earlier)
        PPC_mcrf    = 0x4C000000, // move CR[0-3] to CR[0-3]
        PPC_mcrfs   = 0xFC000080, // move FPSCR fields to CR
        PPC_mfcr    = 0x7C000026, // move from condition register
        PPC_mffs    = 0xFC00048E, // move from fpscr to fpr
        PPC_mfspr   = 0x7C0002A6, // move from spr (special purpose register)
        PPC_mtcrf   = 0x7C000120, // move to condition register field
        PPC_mtfsb0  = 0xFC00008C, // move zero bit into FPSCR
        PPC_mtfsb1  = 0xFC00004C, // move one bit into FPSCR
        PPC_mtspr   = 0x7C0003A6, // move to spr
        PPC_mulli   = 0x1C000000, // multiply low immediate
        PPC_mullw   = 0x7C0001D6, // multiply low word
        PPC_mullwo  = 0x7C0005D6, // multiply low word with overflow
        PPC_nand    = 0x7C0003B8, // nand
        PPC_neg     = 0x7C0000D0, // negate
        PPC_nor     = 0x7C0000F8, // nor
        PPC_or      = 0x7C000378, // or
        PPC_ori     = 0x60000000, // or immediate
        PPC_oris    = 0x64000000, // or immediate shifted
        PPC_rlwimi  = 0x50000000, // rotate left word imm then mask insert
        PPC_rlwinm  = 0x54000000, // rotate left word then and with mask
        PPC_rldicl  = 0x78000000, // rotate left doubleword immediate then clear left
        PPC_rldicr  = 0x78000004, // rotate left doubleword immediate then clear right
        PPC_rldimi  = 0x7800000C, // rotate left doubleword immediate then mask insert
        PPC_sld     = 0x7C000036, // shift left doubleword
        PPC_slw     = 0x7C000030, // shift left word
        PPC_srad    = 0x7C000634, // shift right algebraic doubleword (sign ext)
        PPC_sradi   = 0x7C000674, // shift right algebraic doubleword immediate
        PPC_sraw    = 0x7C000630, // shift right algebraic word (sign ext)
        PPC_srawi   = 0x7C000670, // shift right algebraic word immediate
        PPC_srd     = 0x7C000436, // shift right doubleword (zero ext)
        PPC_srw     = 0x7C000430, // shift right word (zero ext)
        PPC_stb     = 0x98000000, // store byte
        PPC_stbx    = 0x7C0001AE, // store byte indexed
        PPC_std     = 0xF8000000, // store doubleword
        PPC_stdu    = 0xF8000001, // store doubleword with update
        PPC_stdux   = 0x7C00016A, // store doubleword with update indexed
        PPC_stdx    = 0x7C00012A, // store doubleword indexed
        PPC_stfd    = 0xD8000000, // store floating-point double
        PPC_stfdu   = 0xDC000000, // store floating-point double with update
        PPC_stfdx   = 0x7C0005AE, // store floating-point double indexed
        PPC_stfs    = 0xD0000000, // store floating-point single
        PPC_stfsx   = 0x7C00052E, // store floating-point single indexed
        PPC_sth     = 0xB0000000, // store halfword
        PPC_sthx    = 0x7C00032E, // store halfword indexed
        PPC_stw     = 0x90000000, // store word
        PPC_stwu    = 0x94000000, // store word with update
        PPC_stwux   = 0x7C00016E, // store word with update indexed
        PPC_stwx    = 0x7C00012E, // store word indexed
        PPC_subf    = 0x7C000050, // subtract from
        PPC_subfc   = 0x7C000010, // subtract from with carry
        PPC_subfe   = 0x7C000110, // subtract from extended
        PPC_subfo   = 0x7C000450, // subtract from with overflow
#ifdef __APPLE__
        PPC_trap    = 0x7FE00008, // trap word (extended from tw 31,r0,r0)
#else
#error Specify the trap word for your PPC operating system
#endif
        PPC_xor     = 0x7C000278, // xor
        PPC_xori    = 0x68000000, // xor immediate
        PPC_xoris   = 0x6C000000, // xor immediate shifted

        // simplified mnemonics
        PPC_mr = PPC_or,
        PPC_not = PPC_nor,
        PPC_nop = PPC_ori,
        
        PPC_MAJOR_OPCODE_MASK = 0xFC000000 // AND with this to get some idea of the opcode
        };

        class JmpSrc {
            // This is a bit off because we built TenFourFox with gcc 4.0.1
            // previously and it was wacky with friend classes.
        public:
            JmpSrc()
                : m_offset(-1)
            {
            }

            JmpSrc(int offset)
                : m_offset(offset)
            {
            }

            int m_offset;

            int offset() {return m_offset;}

            bool isSet() const {
                return m_offset != -1;
            }
        };

        class JmpDst {
        public:
            JmpDst()
                : m_offset(-1)
                , m_used(false)
            {
            }

            bool isUsed() const { return m_used; }
            void used() { m_used = true; }
            bool isValid() const { return m_offset != -1; }
            int offset() { return m_offset; }
            JmpDst(int offset)
                : m_offset(offset)
                , m_used(false)
            {
                ASSERT(m_offset == offset);
            }

            signed int m_offset : 31;
            int m_used : 1;
        };

        // Laziness macros. Some are used by MacroAssemblerPPC.h, others here.

// whether a (Trusted)Imm32 fits in an unsigned immediate value
#define PPC_IMM_OK_U(x) (MOZ_LIKELY(((x).m_value & 0xffff0000) == 0))

// whether a (Trusted)Imm32 fits in a signed immediate value
#define PPC_IMM_OK_S(x) (MOZ_LIKELY(((x).m_value & 0xffff8000) == 0 || \
    ((x).m_value & 0xffff8000) == 0xffff8000))

// whether the offset part of an Address fits in a (signed) immediate value
#define PPC_OFFS_OK(x) (MOZ_LIKELY(((x).offset & 0xffff8000) == 0 || \
    ((x).offset & 0xffff8000) == 0xffff8000))

// same test, but checking a bit ahead (for multiple loads)
#define PPC_OFFS_INCR_OK(x, incr) (MOZ_LIKELY((((x).offset + incr) & 0xffff8000) == 0 || \
    (((x).offset + incr) & 0xffff8000) == 0xffff8000))

// convert SPRid to 10-bit split encoding (OPPCC appendix A, p.514)
#define PPC_SPR(x) (((int)x>>5) | ((int)x & 31)<<5)

// convenience macros for defining instructions
#if defined(DEBUG)
#define JSPW(x, y, ...)  do { \
    spew_with_address(y, ##__VA_ARGS__); \
    m_buffer.putInt(x); \
    } while(0)

#define JSPWS(x, y) do { \
    spew_with_address(y); \
    m_buffer.putInt(x); \
    } while(0)
#else
#define JSPW(x, y, ...) m_buffer.putInt(x);
#define JSPWS(x, y) m_buffer.putInt(x);
#endif

#if defined(DEBUG)
        // Specialized spewer to show addresses so we can monitor codegen.
        // Ion doesn't do this for us, apparently?
        void spew_with_address(const char *fmt, ...) {
            char buf[200];
            intptr_t code = intptr_t(m_buffer.data());
            // We emit this *before* we emit the instruction so it matches up.
            code += (intptr_t)(m_buffer.size());

            va_list va;
            va_start(va, fmt);
            int i = vsnprintf(buf, sizeof(buf), fmt, va);
            va_end(va);

            GenericAssembler::staticSpew("%08x --- %s", code, buf);
        }
#endif

        /* Low-level instruction emission, cribbed off the PPC nanojit. */

        // Basic relative branching (bctr[l] is separate).

        // Branch "always" instruction, used mostly by the MacroAssembler.
        void b(int32_t offset, BranchAddressType addrType, BranchLink lk) {
            // Maximum LI is a signed 24-bit word-aligned quantity.
            ASSERT(!(offset & 0xFC00003) || (offset & 0xFC00003) == 0xFC000000);

            // Mask to the "24 bit" LI field (22, but word-aligned) so we
            // don't get sign extension when computing the instruction.
            uint32_t offset_bm = offset & 0xFFFFFC;
            JSPW(PPC_b | offset_bm | addrType | lk,
                "b%s%s .%+d", (addrType == AbsoluteBranch ? "a" : ""),
                (lk ? "l" : ""), offset);
         }

        // Branch computation for conditional branches. These use the basic
        // condition code to generate the final instruction. The expanded
        // mnemonic versions are used by Ion.
        void bc(CRegisterID cr, Condition op, int32_t offset,
                BranchLikelihood likely, BranchLink is_link) {
            ASSERT(op < 256);

            // Maximum displacement is a signed 16-bit word-aligned quantity.
            ASSERT(!(offset & 0x03));
            ASSERT(offset > -32767 && offset < 32768);

            // Bitmask the offset so that we don't get sign extension when
            // computing the instruction. The offset field is "16 bit" (14,
            // but limited to word).
            uint32_t offset_bm = offset & 0xfffc;
            // Decode the op into branch option and bit(s). BO is the lower
            // nybble; BI is the top (add cr * 4).
            int bi = (cr << 2) + (op >> 4);
            int bo = (op & 15);
            JSPW((PPC_bc | (bo | likely)<<21 | bi<<16 | offset_bm | is_link),
                "bc%s%s %d, %d, %d", (is_link)?"l":"",
                    (likely)?"+":"", bo, bi, offset);
        }

        void bcctr(CRegisterID cr, int op,
                   BranchLikelihood likely, BranchLink is_link) {
            ASSERT(op < 256);
            int bi = (cr << 2) + (op >> 4);
            int bo = (op & 15);
            JSPW((PPC_bcctr | (bo | likely)<<21 | bi<<16 | is_link),
                "bcctr%s %d, %d",  (likely)?"+":"", bo, bi);
        }

        /* General instructions and class macros.
           These are elemental instructions and standard parts of PowerPC. */
        
        void blr(void) {
            JSPW(PPC_blr, "blr");
        }

#define DEF_CRCR(op) \
    void op(uint8_t t, uint8_t a, uint8_t b) {                          \
        JSPW(PPC_##op | t << 21 | a << 16 | b << 11,                    \
            "%s %i,%i,%i", #op, t, a, b);                               \
    }
        DEF_CRCR(crand)
        DEF_CRCR(crandc)
        DEF_CRCR(cror)
        DEF_CRCR(crorc)
        DEF_CRCR(crxor)
#undef DEF_CRCR

#define DEF_ALU2(op) \
    void op(RegisterID rd, RegisterID ra, RegisterID rb) {              \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rb << 11,  \
            "%s %s,%s,%s", #op, nGPR(rd), nGPR(ra), nGPR(rb));          \
    }                                                                   \
    void op##_rc(RegisterID rd, RegisterID ra, RegisterID rb) {         \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rb << 11 | 1, \
            "%s. %s,%s,%s", #op, nGPR(rd), nGPR(ra), nGPR(rb));         \
    }

        DEF_ALU2(add)
        DEF_ALU2(adde)
        DEF_ALU2(addo)
        DEF_ALU2(subf)
        DEF_ALU2(subfc)
        DEF_ALU2(subfe)
        DEF_ALU2(subfo)
        DEF_ALU2(divw)
        DEF_ALU2(divwo)
        DEF_ALU2(mullw)
        DEF_ALU2(mullwo)
#undef DEF_ALU2

    // eqv is a little weird.
    void eqv(RegisterID ra, RegisterID rs, RegisterID rb) {
        JSPW(PPC_eqv | (int)rs << 21 | (int)ra << 16 | (int)rb << 11,
            "eqv %s,%s,%s", nGPR(ra), nGPR(rs), nGPR(rb));
    }
    void eqv_rc(RegisterID ra, RegisterID rs, RegisterID rb) {
        JSPW(PPC_eqv | (int)rs << 21 | (int)ra << 16 | (int)rb << 11 | 1,
            "eqv. %s,%s,%s", nGPR(ra), nGPR(rs), nGPR(rb));
    }

#define DEF_ALUI(op) \
    void op(RegisterID rd, RegisterID ra, int16_t im) {                 \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | uint16_t(im),   \
            "%s %s,%s,%u (0x%x)", #op, nGPR(rd), nGPR(ra), im, im);     \
    }

        DEF_ALUI(addi)
        DEF_ALUI(addic)
        DEF_ALUI(addis)
        // mulli is usually strength-reduced, since it can take up to five
        // cycles in the worst case. See x_sr_mulli.
        DEF_ALUI(mulli)
#undef DEF_ALUI

#define DEF_ALUE(op) \
    void op(RegisterID rd, RegisterID ra) {                             \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16,                  \
            "%s %s,%s", #op, nGPR(rd), nGPR(ra));                       \
    } \
    void op##_rc(RegisterID rd, RegisterID ra) {                        \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | 1,              \
            "%s. %s,%s", #op, nGPR(rd), nGPR(ra));                      \
    }

        DEF_ALUE(addme)
        DEF_ALUE(addze)
#undef DEF_ALUE

        // cntlzw is "special" -- its encoded operand order is reversed,
        // so the regular DEF_ALUE macro doesn't work.
        void cntlzw(RegisterID ra, RegisterID rs) {
            JSPW(PPC_cntlzw | (int)ra << 16 | (int)rs << 21,
                "cntlzw %s,%s", nGPR(ra), nGPR(rs));
        }
        void cntlzw_rc(RegisterID ra, RegisterID rs) {
            JSPW(PPC_cntlzw | (int)ra << 16 | (int)rs << 21 | 1,
                "cntlzw. %s,%s", nGPR(ra), nGPR(rs));
        }

#define DEF_BITALU2(op) \
    void op(RegisterID rd, RegisterID rs, RegisterID rb) {              \
        JSPW(PPC_##op | (int)rd << 16 | (int)rs << 21 | (int)rb << 11,  \
            "%s %s,%s,%s", #op, nGPR(rd), nGPR(rs), nGPR(rb));          \
    }                                                                   \
    void op##_rc(RegisterID rd, RegisterID rs, RegisterID rb) {         \
        JSPW(PPC_##op | (int)rd << 16 | (int)rs << 21 | (int)rb << 11 | 1, \
            "%s. %s,%s,%s", #op, nGPR(rd), nGPR(rs), nGPR(rb));         \
    }                                                                   \

        DEF_BITALU2(andc)
        DEF_BITALU2(nand)
        DEF_BITALU2(nor)
        DEF_BITALU2(slw)
        DEF_BITALU2(srw)
        DEF_BITALU2(sraw)
        DEF_BITALU2(sld)
        DEF_BITALU2(srd)
        DEF_BITALU2(srad)
#undef DEF_BITALU2

// |and|, |or|, |xor|, and |not| are operators in C++, so we can't use those
// tokens as arguments to macros(!) and therefore write out these functions.
        void and_(RegisterID rd, RegisterID rs, RegisterID rb) {
            JSPW(PPC_and | (int)rd << 16 | (int)rs << 21 | (int)rb << 11,
                "and %s,%s,%s", nGPR(rd), nGPR(rs), nGPR(rb));
        }
        void and_rc(RegisterID rd, RegisterID rs, RegisterID rb) {
            JSPW(PPC_and | (int)rd << 16 | (int)rs << 21 | (int)rb << 11 | 1,
                "and. %s,%s,%s", nGPR(rd), nGPR(rs), nGPR(rb));
        }
        void or_(RegisterID rd, RegisterID rs, RegisterID rb) {
            JSPW(PPC_or | (int)rd << 16 | (int)rs << 21 | (int)rb << 11,
                "or %s,%s,%s", nGPR(rd), nGPR(rs), nGPR(rb));
        }
        void or_rc(RegisterID rd, RegisterID rs, RegisterID rb) {
            JSPW(PPC_or | (int)rd << 16 | (int)rs << 21 | (int)rb << 11 | 1,
                "or. %s,%s,%s", nGPR(rd), nGPR(rs), nGPR(rb));
        }
        void xor_(RegisterID rd, RegisterID rs, RegisterID rb) {
            JSPW(PPC_xor | (int)rd << 16 | (int)rs << 21 | (int)rb << 11,
                "xor %s,%s,%s", nGPR(rd), nGPR(rs), nGPR(rb));
        }
        void xor_rc(RegisterID rd, RegisterID rs, RegisterID rb) {
            JSPW(PPC_xor | (int)rd << 16 | (int)rs << 21 | (int)rb << 11 | 1,
                "and. %s,%s,%s", nGPR(rd), nGPR(rs), nGPR(rb));
        }

#define DEF_BITALUI(op) \
    void op(RegisterID rd, RegisterID ra, uint16_t im) {                \
        JSPW(PPC_##op | (int)ra << 21 | (int)rd << 16 | im,             \
            "%s %s,%s,%u (0x%x)", #op, nGPR(rd), nGPR(ra), im, im);     \
    }
        DEF_BITALUI(ori)
        DEF_BITALUI(oris)
        DEF_BITALUI(xori)
        DEF_BITALUI(xoris)

        // andi. is special in that it sets condition codes: call it andi_rc
        void andi_rc(RegisterID rd, RegisterID ra, uint16_t im) {
            JSPW(PPC_andi | (int)ra << 21 | (int)rd << 16 | im,
                "andi. %s,%s,%u (0x%x)", nGPR(rd), nGPR(ra), im, im);
        }
        void andis_rc(RegisterID rd, RegisterID ra, uint16_t im) {
            JSPW(PPC_andis | (int)ra << 21 | (int)rd << 16 | im,
                "andis. %s,%s,%u (0x%x)", nGPR(rd), nGPR(ra), im, im);
        }
#undef DEF_BITALUI

#define DEF_ALUEXT(op) \
    void op(RegisterID rd, RegisterID rs) {             \
        JSPW(PPC_##op | (int)rd << 16 | (int)rs << 21,  \
            "%s %s,%s", #op, nGPR(rd), nGPR(rs));       \
    }
        DEF_ALUEXT(extsb)
        DEF_ALUEXT(extsh)
        DEF_ALUEXT(extsw)
#undef DEF_ALUEXT

// NB! This is written from the perspective of T,A,C,B being T,A,B,C (i.e.,
// we have C and B transposed below). Our code accounts for that, I hope.
#define DEF_FPUABC(op) \
    void op(FPRegisterID rd,FPRegisterID ra,FPRegisterID rb,FPRegisterID rc) {\
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rc << 11   \
                | (int)rb << 6,                                         \
            "%s %s,%s,%s,%s",#op, nFPR(rd), nFPR(ra), nFPR(rb), nFPR(rc)); \
    } \
    void op##_rc(FPRegisterID rd,FPRegisterID ra,FPRegisterID rb,FPRegisterID rc) {\
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rc << 11   \
                | (int)rb << 6 | 1,                                     \
            "%s. %s,%s,%s,%s",#op, nFPR(rd), nFPR(ra), nFPR(rb), nFPR(rc)); \
    }

        DEF_FPUABC(fmadd)
        DEF_FPUABC(fnmsub)
        DEF_FPUABC(fsel)
#undef DEF_FPUABC

#define DEF_FPUAB(op) \
    void op(FPRegisterID rd, FPRegisterID ra, FPRegisterID rb) {        \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rb << 11,  \
            "%s %s,%s,%s", #op, nFPR(rd), nFPR(ra), nFPR(rb));          \
    }                                                                   \
    void op##_rc(FPRegisterID rd, FPRegisterID ra, FPRegisterID rb) {   \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rb << 11 | 1, \
            "%s. %s,%s,%s", #op, nFPR(rd), nFPR(ra), nFPR(rb));         \
    }

        DEF_FPUAB(fadd)
        DEF_FPUAB(fdiv)
        DEF_FPUAB(fsub)
#undef DEF_FPUAB

#define DEF_FPUAC(op) \
    void op(FPRegisterID rd, FPRegisterID ra, FPRegisterID rc) {        \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rc << 6,   \
            "%s %s,%s,%s", #op, nFPR(rd), nFPR(ra), nFPR(rc));          \
    }                                                                   \
    void op##_rc(FPRegisterID rd, FPRegisterID ra, FPRegisterID rc) {   \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rc << 6 | 1, \
            "%s. %s,%s,%s", #op, nFPR(rd), nFPR(ra), nFPR(rc));         \
    }

        DEF_FPUAC(fmul)
#undef DEF_FPUAC

#define DEF_FPUDS(op) \
    void op(FPRegisterID rd, FPRegisterID rs) {         \
        JSPW(PPC_##op | (int)rd << 21 | (int)rs << 11,  \
            "%s %s,%s", #op, nFPR(rd), nFPR(rs));       \
    } \
    void op##_rc(FPRegisterID rd, FPRegisterID rs) {         \
        JSPW(PPC_##op | (int)rd << 21 | (int)rs << 11 | 1,  \
            "%s. %s,%s", #op, nFPR(rd), nFPR(rs));       \
    }

        DEF_FPUDS(fabs)
        DEF_FPUDS(fneg)
        DEF_FPUDS(fmr)
        DEF_FPUDS(fcfid)
        DEF_FPUDS(fctiw)
        DEF_FPUDS(fctiwz)
        DEF_FPUDS(frsp)
        DEF_FPUDS(frsqrte)

        // G5 only
        DEF_FPUDS(fsqrt)
#undef DEF_FPUDS

#define DEF_MEMd(op) \
    void op(RegisterID rd, RegisterID rbase, int16_t off) {                 \
        JSPW(PPC_##op | (int)rd << 21 | (int)rbase << 16 | uint16_t(off),   \
            "%s %s,%d(%s)", #op, nGPR(rd), off, nGPR(rbase));               \
    }

        DEF_MEMd(lbz)
        DEF_MEMd(lha)
        DEF_MEMd(lhz)
        DEF_MEMd(lwz)
        DEF_MEMd(ld)

        DEF_MEMd(stb)
        DEF_MEMd(stw)
        DEF_MEMd(stwu)
        DEF_MEMd(sth)
        DEF_MEMd(std)
        DEF_MEMd(stdu)
#undef DEF_MEMd

#define DEF_MEMx(op) \
    void op(RegisterID rd, RegisterID ra, RegisterID rb) {            \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rb << 11,         \
            "%s %s,%s,%s", #op, nGPR(rd), nGPR(ra), nGPR(rb));  \
    }

        DEF_MEMx(lbzx)
        DEF_MEMx(lhax)
        DEF_MEMx(lhzx)
        DEF_MEMx(lwzx)
        DEF_MEMx(ldx)

        DEF_MEMx(stbx)
        DEF_MEMx(stwx)
        DEF_MEMx(stwux)
        DEF_MEMx(sthx)
        DEF_MEMx(stdx)
        DEF_MEMx(stdux)
#undef DEF_MEMx

#define DEF_FMEMd(op) \
    void op(FPRegisterID rd, RegisterID rbase, int16_t off) {               \
        JSPW(PPC_##op | (int)rd << 21 | (int)rbase << 16 | uint16_t(off),   \
            "%s %s,%d(%s)", #op, nFPR(rd), off, nGPR(rbase));               \
    }

        DEF_FMEMd(lfd)
        DEF_FMEMd(lfs)
        DEF_FMEMd(stfd)
        DEF_FMEMd(stfs)
        DEF_FMEMd(stfdu)
#undef DEF_FMEMd

#define DEF_FMEMx(op) \
    void op(FPRegisterID rd, RegisterID ra, RegisterID rb) {            \
        JSPW(PPC_##op | (int)rd << 21 | (int)ra << 16 | (int)rb << 11,  \
            "%s %s,%s,%s", #op, nFPR(rd), nGPR(ra), nGPR(rb));          \
    }

        DEF_FMEMx(lfdx)
        DEF_FMEMx(lfsx)
        DEF_FMEMx(stfdx)
        DEF_FMEMx(stfsx)
#undef DEF_FMEMx

        // not enough generality to be worth a macro
        void neg(RegisterID rd, RegisterID rs) {
            JSPW(PPC_neg | (int)rd << 21 | (int)rs << 16,
                "neg %s,%s", nGPR(rd), nGPR(rs));
        }

        void cmpw(RegisterID ra, RegisterID rb) {
            JSPW(PPC_cmpw | (int)ra << 16 | (int)rb << 11,
                "cmpw cr0,%s,%s", nGPR(ra), nGPR(rb));
        }
        
        void cmpwi(RegisterID ra, int16_t im) {
            JSPW(PPC_cmpwi | (int)ra << 16 | uint16_t(im),
                "cmpwi %s,%d (0x%x)", nGPR(ra), im, im);
        }

        void cmplw(RegisterID ra, RegisterID rb) {
            JSPW(PPC_cmplw | (int)ra << 16 | (int)rb << 11,
                "cmplw cr0,%s,%s", nGPR(ra), nGPR(rb));
        }
        
        void cmplwi(RegisterID ra, int16_t im) {
            JSPW(PPC_cmplwi | (int)ra << 16 | uint16_t(im),
                "cmplwi %s,%d (0x%x)", nGPR(ra), im, im);
        }

        void fcmpu(FPRegisterID ra, FPRegisterID rb) {
            JSPW(PPC_fcmpu | (int)ra << 16 | (int)rb << 11,
                "fcmpu cr0,%s,%s", nFPR(ra), nFPR(rb));
        }

        void srawi(RegisterID rd, RegisterID rs, int8_t n) {
            JSPW(PPC_srawi | (int)rs << 21 | (int)rd << 16 | n << 11,
                "srawi %s,%s,%d", nGPR(rd), nGPR(rs), n);
        }

        void rlwinm(RegisterID rd, RegisterID rs, int8_t sh, int8_t mb,
                int8_t me) {
            JSPW(PPC_rlwinm | (int)rs << 21 | (int)rd << 16 | (sh) << 11 |
                    (mb) << 6 | (me) << 1,
                "rlwinm %s,%s,%d,%d,%d", nGPR(rd), nGPR(rs), sh, mb, me);
        }

        // this is a cracked instruction on G5
        void rlwimi(RegisterID rd, RegisterID rs, int8_t sh, int8_t mb,
            int8_t me) {
            JSPW(PPC_rlwimi | (int)rs << 21 | (int)rd << 16 | (sh) << 11 |
                (mb) << 6 | (me) << 1,
                "rlwimi %s,%s,%d,%d,%d", nGPR(rd), nGPR(rs), sh, mb, me);
        }

        void bctr(BranchLink is_link) {
            JSPW(PPC_bctr | (is_link ? 1 : 0), "bctr%s", (is_link) ? "l" : "");
        }
        
        // Extended mnemonics for m[tf]%spr% are below.
        void mfspr(SPRegisterID sr, RegisterID r) {
            JSPW(PPC_mfspr | (int)r << 21 | PPC_SPR(sr) << 11,
                "mfspr %s,%s", nGPR(r), nSPR(sr));
        }

        void mtspr(SPRegisterID sr, RegisterID r) {
            JSPW(PPC_mtspr | (int)r<<21 | PPC_SPR(sr) << 11,
                 "mtspr %s, %s", nSPR(sr), nGPR(r));
        }
        
        void mfcr(RegisterID rd) {
            JSPW(PPC_mfcr | (int)rd << 21, "mfcr %s", nGPR(rd));
        }

        // G5 only
        void mfocrf(RegisterID rd, CRegisterID crfs) {
            JSPW(PPC_mfcr | (int)rd << 21 | 1 << 20 | 1 << (19-(int)crfs),
                "mfocrf %s,%s", nGPR(rd), nCR(crfs));
        }
        
        void mtcrf(uint16_t mask, RegisterID rs) {
            JSPW(PPC_mtcrf | (int)rs << 21 | mask << 12,
                "mtcrf %d,%s", mask, nGPR(rs));
        }

        void mtfsb0(uint8_t bt) {
            JSPW(PPC_mtfsb0 | (bt << 21), "mtfsb0 %d", bt);
        }
        void mtfsb1(uint8_t bt) {
            JSPW(PPC_mtfsb1 | (bt << 21), "mtfsb1 %d", bt);
        }

        void mcrf(CRegisterID crft, CRegisterID crfs) {
            JSPW(PPC_mcrf | (int)crft<<23 | (int)crfs<<18,
                "mcrf %s,%s", nCR(crft), nCR(crfs));
        }

        void mcrfs(CRegisterID bf, uint8_t bfa) {
            JSPW(PPC_mcrfs | (int)bf<<23 | (bfa << 18),
                "mcrfs %s,%d", nCR(bf), bfa);
        }
        
        /* Synthetic "extended" instructions ("x_*").
           These aren't real PPC instructions, but we simulate them as
           useful fictions for various cases. Some map to multiple actual
           instructions, and some are limited versions of real instructions
           that are tailored for our usage. Many are accepted mnemonics. */

        void x_nop() {
            // Emit PPC_nop, which is ori 0,0,0
            JSPWS(PPC_nop, "nop");
        }

        void x_trap() {
            // Emit PPC_trap, which is tw 31, r0, r0
            JSPWS(PPC_trap, "trap");
        }

        void x_bctrl() {
            bctr(LinkBranch);
        }

        void x_baa(RegisterID target) {
            x_mtctr(target);
#if defined(_PPC970_)
// G5 does really badly if the mtctr and bctr are in the same dispatch group.
// Insert nops to desperately avoid this. We need four to ensure the bctr
// doesn't wind up in the branch slot.
            x_nop();
            x_nop();
            x_nop();
            x_nop();
#endif
            bctr(DontLinkBranch);
        }

        void x_slwi(RegisterID rd, RegisterID rs, int n) {
            JSPW(PPC_rlwinm | (int)rs << 21 | (int)rd << 16 | n << 11 | 0 << 6 | (31-n) << 1,
                "slwi %s,%s,%d", nGPR(rd), nGPR(rs), n);
        }
        
        void x_srwi(RegisterID rd, RegisterID rs, int n) {
            JSPW(PPC_rlwinm | (int)rs << 21 | (int)rd << 16 | (32-n) << 11 | n << 6 | 31<<1,
                "srwi %s,%s,%d", nGPR(rd), nGPR(rs), n);
        }

        void x_mtcr(RegisterID rs) {
            JSPW(PPC_mtcrf | (int)rs << 21 | 0xff << 12,
                "mtcr %s", nGPR(rs));
        }

        // Strength-reduced immediate multiply. mulli, in the worst case,
        // can take up to five cycles (most implementations range from three
        // to four). Try to prevent this by turning into series of additions
        // without exceeding the expected cycle count.
        void x_sr_mulli(RegisterID rd, RegisterID ra, int16_t im) {
            // TODO: This is limited to adds. Explore if rlwimi + extsw can
            // still be faster in the general case or in cases of 2^x & 2^x+1.
#if DEBUG
                           GenericAssembler::staticSpew("#x_sr_mulli %s,%s,%d",
                           nGPR(rd), nGPR(ra), im);
#endif
            switch(im) {
                case 0:
                    // Shouldn't this be optimized out?
                    // Just load 0.
                    x_li32(rd, 0);
                    break;

                case 1:
                    // Shouldn't this be optimized out?
                    // Just load ra, unless rd == ra.
                    if (rd != ra)
                        or_(rd, ra, ra);
                    break;

                case 2:
                    // rd = ra + ra
                    add(rd, ra, ra);
                    break;

                case 3:
                    // This is on SunSpider, btw.
                    // The trick here is that rd may equal ra. If so,
                    // we can't do this optimization because storing and
                    // restoring the register won't save us any time (and
                    // we can't steal r0 like the nanojit did because the
                    // macroassembler may be using it without us knowing).
                    // rd = ra + ra
                    // rd = rd + ra
                    if (rd != ra) {
                        add(rd, ra, ra);
                        add(rd, rd, ra);
                    } else {
                        // Oh well.
                        mulli(rd, ra, im);
                    }
                    break;

                case 4:
                    // rd = ra + ra, then
                    // rd = rd + rd
                    add(rd, ra, ra);
                    add(rd, rd, rd);
                    break;

                case 5:
                    // Same limitation as 3.
                    // rd = ra + ra
                    // rd = rd + rd
                    // rd = rd + ra
                    if (rd != ra) {
                        add(rd, ra, ra);
                        add(rd, rd, rd);
                        add(rd, rd, ra);
                    } else {
                        // Too bad.
                        mulli(rd, ra, im);
                    }
                    break;

                case 8:
                    // *4 then *2
                    add(rd, ra, ra);
                    add(rd, rd, rd);
                    add(rd, rd, rd);
                    break;

                case 10:
                    // *5 then *2
                    if (rd != ra) {
                        add(rd, ra, ra);
                        add(rd, rd, rd);
                        add(rd, rd, ra);
                        add(rd, rd, rd);
                    } else {
                        // So sad.
                        mulli(rd, ra, im);
                    }
                    break;

                case 16:
                    // *8 then *2
                    add(rd, ra, ra);
                    add(rd, rd, rd);
                    add(rd, rd, rd);
                    add(rd, rd, rd);
                    break;

                default:
                    // Oh well, we tried.
                    mulli(rd, ra, im);
                    break;
            }
        }

        // set rd to 1 if the given bit in rs is set, 0 otherwise
        void x_bit_value(RegisterID rd, RegisterID rs, unsigned bit) {
            ASSERT(bit < 32);
            // rotate the given bit to bit 31 and then mask with 0x00000001
            if(bit < 31) {
                JSPW(PPC_rlwinm |
                    (int)rs << 21 | (int) rd << 16 | (bit+1) << 11 |
                        31 << 6 | 31 << 1,
                    "x_bit_value %s,%s,%d", nGPR(rd), nGPR(rs), bit);
            } else {
                andi_rc(rd, rs, 0x00000001);
            }
        }

        // fill bits 0-15 of rd with bits 16-31 of rs
        void x_insertbits0_15(RegisterID rd, RegisterID rs) {
            rlwimi(rd, rs, 16, 0, 15);
        }

        void x_li(RegisterID rd, int16_t im) {
            JSPW(PPC_addi | (int)rd << 21 | uint16_t(im),
                "li %s,%d (0x%x)", nGPR(rd), im, im);
        }
        
        void x_lis(RegisterID rd, int16_t im) {
            JSPW(PPC_addis | (int)rd << 21 | uint16_t(im),
                "lis %s,%d (0x%x)", nGPR(rd), im, im << 16);
        }

        void x_subi(RegisterID rd, RegisterID ra, int16_t im) {
            JSPW(PPC_addi | (int)rd << 21 | (int)ra << 16 | uint16_t(-im),
                "subi %s,%s,%d", nGPR(rd), nGPR(ra), im);
        }

        void x_li32(RegisterID rd, int32_t im) {
            if ((im & 0xffff8000) == 0 || (im & 0xffff8000) == 0xffff8000) {
                // fits in an int16_t: li
                x_li(rd, uint16_t(im & 0xffff));
            } else if (!(im & 0xffff)) {
                // doesn't have a lower 16 bits: only lis
                x_lis(rd, uint16_t(im >> 16));
            } else {
                // 32 bits: lis for the upper 16 bits, then or_ in the lower
                x_lis(rd, uint16_t(im >> 16));
                ori(rd, rd, uint16_t(im & 0xffff));
            }
        }
        
        // patchable form: always lis + ori
        void x_p_li32(RegisterID rd, int32_t im) {
            x_lis(rd, uint16_t(im >> 16));
            ori(rd, rd, uint16_t(im & 0xffff));
        }

        // Emit mcrxr (G3, G4) or equivalent (G5, POWER4+). On OS X mcrxr
        // is emulated in software. EEEEEEEEK.
        void x_mcrxr(CRegisterID r) {
            // XXX: Right now, this is only supported for CR7.
            // Change the two rlwinm instructions below for other CRs.
            ASSERT(r == PPCRegisters::cr7);

// Use alternate code in DEBUG mode since I usually debug on a G5 and I'd
// like my test runs to finish sometime before the heat death of the universe.
#if defined(_PPC970_) || defined(DEBUG)
            // Use r12 for this, because r0 already has our ctr.
            RegisterID cfc = PPCRegisters::r12; // get it? o-chem humour

            // Rotate the XER bits into the right slot in the temp register.
            x_mfxer(cfc);
            rlwinm(cfc, cfc, 4, 0, 31);
            // Copy to that CR field.
            // Our mtcrf mask is based on the condreg we're after. 128 = cr0
            mtcrf((1 << (7-(int)r)), cfc);
            // Now rotate back and clear out the XER fields we need to erase.
            rlwinm(cfc, cfc, 28, 3, 31);
            // And do the write back to the XER
            x_mtxer(cfc);
#else
            JSPW(PPC_mcrxr | (int)r << 23, "mcrxr %s", nCR(r));
#endif
        }

        // SPR convenience methods.
#define DEF_MXSPR(spr) \
        void x_mt##spr(RegisterID rs) { mtspr(PPCRegisters::spr, rs); } \
        void x_mf##spr(RegisterID rs) { mfspr(PPCRegisters::spr, rs); }

        DEF_MXSPR(ctr)
        DEF_MXSPR(xer)
        DEF_MXSPR(lr)
#undef DEF_MXSPR

        /* Modular operations ("m_*").
           These are multi-instruction (mostly) operations that we treat
           as atomic units but have more logic than synthetic instructions. */

        // Convert a (possibly synthetic) condition code (implicitly in CR0)
        // into a zero or one in the specified register.
        void m_set_cond(RegisterID dest, int cond)
        {
#if DEBUG
            GenericAssembler::staticSpew("#m_set_cond  ((%02x))", cond);
#endif

            if (cond & ConditionOnlyXER) {
                x_mfxer(dest);
                x_bit_value(dest, dest, (cond >> 4) & 0x03);
                if ((cond & BranchOptionMask) == BranchOnClear) {
                    xori(dest, dest, 1);
                }
            } else {
#if _PPC970_
                // Because we only care about CR0 ...
                mfocrf(dest, JSC::PPCRegisters::cr0);
#else
                mfcr(dest);
#endif
                x_bit_value(dest, dest, (cond >> 4) & 0x03);

                if ((cond & BranchOptionMask) == BranchOnClear) {
                    // need to invert the sense of the CR0 bit
                    xori(dest, dest, 1);
                }
            }
        }
        
        // On G3/G4, we reserve two words for each branch. The first is a branch
        // instruction (skipping over the second instruction) which records
        // what type of branch it is: conditional vs. unconditional, linking,
        // and what the condition is. The second word is an addi which stores
        // the offset to three more words reserved in the constant pool area.
        // In the event of a branch which is too large to fit in a direct branch
        // instruction, the first word is patched to be the lis of a
        // lis/ori/mtctr/bctr
        // far branch stanza. The second word then becomes the active branch,
        // except that it targets the trampoline, which in turn completes the
        // far branch stanza.
        // This strategy should also favour embedded and "little" PPC cores
        // with smaller caches or less aggressive speculative execution such
        // as e500, QorIQ and PowerPC 4xx.
        //
        // On G5, however, this interferes with its tendencies to be (in Apple's
        // words) "very hungry, very fast and very sequential." It gains 5-10%
        // on most benchmarks to keep everything inline in a four-word branch,
        // normally the bare branch, but a full lis/ori/mtctr/bctr if it is a
        // far call. We don't use a trampoline in the constant pool at all.
        // This strategy also favours "big POWER" like POWER5/6/7 and Cell
        // PPE/Xenon, since the G5 is really just a POWER4 with a deeper
        // pipeline and AltiVec.

        // The length, in bytes, of a branch stanze. Since it's a multiple of 4
        // you can just OR it with the branch opcode for a valid branch. (Bits
        // 30 and 31 of the branch instructions are the AA and LK bits, but the
        // target address field is divided by 4 and then left-shifted by two --
        // exactly equivalent to simply ORing it in in-place.)
#if USE_970_BRANCHING
#define PPC_BRANCH_STANZA_LENGTH 16
#define PPC_CALL_STANZA_LENGTH 16
#else
#define PPC_BRANCH_STANZA_LENGTH 8
#define PPC_CALL_STANZA_LENGTH 8
#endif

    private:
        void ensureSpaceForBranch(void)
        {
#if USE_970_BRANCHING
            ASSERT(0);
#else
            // make sure that the branch is not broken up by the constant buffer
            m_buffer.ensureSpace(8, 12);
#endif
        }

    public:
        void x_skip_this_jump() {
            // a pseudo-instruction which jumps over the entire branch stanza
#define PPC_SKIP_THIS_JUMP (PPC_b | PPC_BRANCH_STANZA_LENGTH)
            JSPWS(PPC_SKIP_THIS_JUMP, "x_skip_this_jump");
        }

        JmpSrc m_call()
        {
#if USE_970_BRANCHING
            x_skip_this_jump();
            x_nop();
            x_nop();
            x_bctrl(); // unconditional branch, with LK

            return JmpSrc(m_buffer.size());
#else
            ensureSpaceForBranch();
            b(8, RelativeBranch, LinkBranch);
            m_buffer.putIntWithMultipleConstantInts(PPC_addi, 3, PPC_trap);

            return JmpSrc(m_buffer.uncheckedSize());
#endif
        }
        
        // This doesn't even need to be patchable, so it's only two words
        // (except G5).
        JmpSrc m_call_reg(RegisterID target)
        {
            x_mtctr(target);
#if defined(_PPC970_)
            x_nop();
            x_nop();
            x_nop();
            x_nop();
#endif
            x_bctrl();

            return JmpSrc(m_buffer.size());
        }

        JmpSrc m_jump()
        {
#if USE_970_BRANCHING
            x_skip_this_jump(); // this shortcuts the jump in the default case.
            x_nop();
            x_nop();
            bctr(DontLinkBranch); // unconditional branch, no LK

            return JmpSrc(m_buffer.size());
#else
            ensureSpaceForBranch();
            b(8, RelativeBranch, DontLinkBranch);
            m_buffer.putIntWithMultipleConstantInts(PPC_addi, 3, PPC_trap);

            return JmpSrc(m_buffer.uncheckedSize());
#endif
        }

        // Generate a branch based on an integer condition. This can get
        // synthetic condition codes. The base code is assumed in CR0, with
        // mcrxr in CR7 (if needed).
        JmpSrc m_branch(int cond)
        {
            // First emit the mcrxr equivalent, before the actual branch stanza.
            // This will burn r12 on G5, but we assume the macroassembler
            // doesn't need it anymore.
            // We store XER to CR7 in case we need to also test against CR0
            // later (which is possible).
            if (cond & ConditionOnlyXER)
                x_mcrxr(PPCRegisters::cr7);

#if USE_970_BRANCHING
            // Now for the branch stanza
            x_skip_this_jump();
            x_nop();
            x_nop(); // 3 words

            if (cond & ConditionOnlyXER) {
                // If we're testing against XER codes, we put those into CR7
                bcctr(PPCRegisters::cr7, (uint32_t)cond & 0xff, NotLikelyBranch, DontLinkBranch);
            } else {
                // Otherwise we default to testing against CR0
                bcctr(PPCRegisters::cr0, (uint32_t)cond & 0xff, NotLikelyBranch, DontLinkBranch);
            }

            return JmpSrc(m_buffer.size());
#else
            ensureSpaceForBranch();

            if (cond & ConditionOnlyXER) {
                // If we're testing against XER codes, we put those into CR7
                bc(PPCRegisters::cr7, Condition(cond & 0xff), 8, NotLikelyBranch, DontLinkBranch);
            } else {
                // Otherwise we default to testing against CR0
                bc(PPCRegisters::cr0, Condition(cond & 0xff), 8, NotLikelyBranch, DontLinkBranch);
            }
            m_buffer.putIntWithMultipleConstantInts(PPC_addi, 3, PPC_trap);

            return JmpSrc(m_buffer.uncheckedSize());
#endif
        }

        // Generate a branch based on a floating point condition. This can
        // get synthetic condition codes. This is similar to m_branch.
        JmpSrc m_fbranch(int cond)
        {
            const uint8_t condBit = crBit(PPCRegisters::cr0, DoubleCondition(cond));
            const uint8_t fuBit = crBit(PPCRegisters::cr0, DoubleConditionFU);

            // Use condition register logic to combine the FU (Unordered) bit
            // with the requested condition bit. This is not part of the actual
            // branch stanza.
            if (cond & DoubleConditionUnordered) {
                // branch if cond OR Unordered
                if ((cond & BranchOptionMask) == BranchOnClear) {
                    // invert the condBit, or it with fuBit, and then branch on Set
                    crorc(condBit, fuBit, condBit);
                    cond |= BranchOnSet;
                } else {
                    // or the condBit with fuBit, and then branch on Set
                    cror(condBit, fuBit, condBit);
                }
            } else {
                // branch if cond AND !Unordered
                if ((cond & BranchOptionMask) == BranchOnClear) {
                    // or the condBit with fuBit, and branch on Clear
                    cror(condBit, fuBit, condBit);
                } else {
                    // and the condBit with !fuBit, and branch on Set
                    crandc(condBit, condBit, fuBit);
                }
            }
            
            // The branch stanza starts here
#if USE_970_BRANCHING
            x_skip_this_jump();
            x_nop();
            x_nop();
            bcctr(PPCRegisters::cr0, (uint32_t)cond & 0xff, NotLikelyBranch, DontLinkBranch); // conditional branch, no LK

            return JmpSrc(m_buffer.size());
#else
            ensureSpaceForBranch();
            bc(PPCRegisters::cr0, Condition(cond & 0xff), 8, NotLikelyBranch, DontLinkBranch); // conditional branch, no LK
            m_buffer.putIntWithMultipleConstantInts(PPC_addi, 3, PPC_trap);

            return JmpSrc(m_buffer.uncheckedSize());
#endif
        }

        /* Assembler admin methods and general helpers. */

        JmpDst label()
        {
            // Attempting to align this to 32-byte blocks made runtime worse
            // on G5.
            JmpDst r = JmpDst(m_buffer.size());
#if DEBUG
            GenericAssembler::staticSpew("#label     ((%d))", r.m_offset);
#endif
            return r;
        }

        size_t allocSize() const
        {
            return m_buffer.allocSize();
        }

        void ensureSpace(int space) 
        {
#if USE_970_BRANCHING
            m_buffer.ensureSpace(space);
#else
            // make sure that we have enough room to emit a trampoline
            // for each two-word block of space requested
            m_buffer.ensureSpace(space, space/2 * 3);
#endif
        }

        int flushCount()
        {
#if USE_970_BRANCHING
            ASSERT(0);
            return 0;
#else
            return m_buffer.flushCount();
#endif
        }

        void forceConstantPoolFlushWithBarrier()
        {
#if !defined(USE_970_BRANCHING)
            return m_buffer.forceConstantPoolFlush();
#endif
        }

        static int getDifferenceBetweenLabels(JmpDst src, JmpDst dst)
        {
            return dst.m_offset - src.m_offset;
        }
    
        static int getDifferenceBetweenLabels(JmpDst src, JmpSrc dst)
        {
            return dst.m_offset - src.m_offset;
        }
    
        static int getDifferenceBetweenLabels(JmpSrc src, JmpDst dst)
        {
            return dst.m_offset - src.m_offset;
        }

        static void* getRelocatedAddress(void* code, JmpSrc jump)
        {
            ASSERT(jump.m_offset != -1);

            return reinterpret_cast<void*>(reinterpret_cast<ptrdiff_t>(code) + jump.m_offset);
        }
    
        static void* getRelocatedAddress(void* code, JmpDst destination)
        {
            ASSERT(destination.m_offset != -1);

            return reinterpret_cast<void*>(reinterpret_cast<ptrdiff_t>(code) + destination.m_offset);
        }

        int size() { return m_buffer.size(); }
        unsigned char *buffer() const { return m_buffer.buffer(); }

        // Absolute internal jumps can only patched once we know the final
        // destination of the code
        void* executableAllocAndCopy(ExecutableAllocator* allocator, ExecutablePool **poolp, CodeKind kind)
        {
            void *ret = m_buffer.executableAllocAndCopy(allocator, poolp, kind);
            if (!ret)
                return 0;
            finalizeAbsoluteJumps(ret);
            return ret;
        }

        // Again, we have to fix up the absolute jumps once we know the new location
        void executableCopy(void* buffer)
        {
            memcpy(buffer, m_buffer.buffer(), size());
            finalizeAbsoluteJumps(buffer);
        }

        void forceFlushConstantPool()
        {
#if !defined(USE_970_BRANCHING)
            m_buffer.flushWithoutBarrier(true);
#endif
        }

        // Ion does not call executableAllocAndCopy, but it still needs to
        // flush the buffer or we can't finalize the jumps.
        void executableCopyAndFlush(void *buffer)
        {
#if !defined(USE_970_BRANCHING)
            forceFlushConstantPool();
#endif
            executableCopy(buffer);
        }

        static unsigned getCallReturnOffset(JmpSrc call)
        {
            ASSERT(call.m_offset >= 0);
            return call.m_offset;
        }

        // Patching: we need to be able to patch in a lis/ori 32-bit immediate load,
        // and also to change a load into an address calculation or vice-versa.

        // in spite of the name, this is used for patching _data_ loads, not flow control
        static void linkPointer(void* code, JmpDst where, void* value)
        {
#if DEBUG
            GenericAssembler::staticSpew("##linkPointer     ((%p + %#x)) points to ((%p))",
                           code, where.m_offset, value);
#endif

            uint32_t *from = (uint32_t *)(intptr_t(code) + where.m_offset);
            patchImmediate(from, uint32_t(value));
        }

        static void repatchInt32(void* where, int value)
        {
#if DEBUG
            GenericAssembler::staticSpew("##repatchInt32 ((where=%p)) holds ((value=%d))",
                           where, value);
#endif
            patchImmediate((uint32_t *)where, uint32_t(value));
        }

        static void repatchPointer(void* where, void* value)
        { 
#if DEBUG
            GenericAssembler::staticSpew("##repatchPointer ((where = %p)) points to ((%p))",
                           where, value);
#endif
            patchImmediate((uint32_t *)where, uint32_t(value));
        }


    public: // IonMonkey uses this.
        static void patchImmediate(uint32_t *code, uint32_t imm);

    public:
        static void repatchLoadPtrToLEA(void* where)
        {
            // This is a grotesque x86-centric thing, where we need to turn
            // a standard lis/ori/lwzx into lis/ori/add, emulating what LEA
            // does on x86. I need to go spit on my C2D Mac mini. Back soon.
            // ARM and SPARC have to do essentially the same thing as they also
            // lack LEA or a similar instruction; SPARC is most like us.

            // Skip the load, which is guaranteed to be 2 words.
            uint32_t *ptr = (uint32_t *)(intptr_t(where) + 8);
            // This damn well better be a PPC_lwzx or we are hosed.
#if DEBUG
            GenericAssembler::staticSpew("##repatchLoadPtrToLEA ((where = %p))", ptr);
#endif
            ASSERT((ptr[0] & PPC_lwzx) == PPC_lwzx);

            // Flip the bits to turn lwzx into add.
            // 07c00002e ^ 0x0000023a <=> 07c000214 ^ 0x0000023a
            ptr[0] ^= 0x0000023a;
            ExecutableAllocator::cacheFlush(ptr, 4);
        }

        static void repatchLEAToLoadPtr(void* where)
        {
            // Reverse the above operation.
            // However, sometimes this gets called when the instruction is
            // already an lwzx, so we should politely accept that.
            uint32_t *ptr = (uint32_t *)(intptr_t(where) + 8);
            if ((ptr[0] & PPC_lwzx) == PPC_lwzx) {
#if DEBUG
                GenericAssembler::staticSpew("##repatchLEAToLoadPtr [ignored] ((where = %p))", ptr);
#endif
                return;
            }
            // But, if it's not lwzx, it better be an add.
#if DEBUG
            GenericAssembler::staticSpew("##repatchLEAToLoadPtr ((where = %p))", ptr);
#endif
            ASSERT((ptr[0] & PPC_add) == PPC_add);
            ptr[0] ^= 0x0000023a;
            ExecutableAllocator::cacheFlush(ptr, 4);
        }

        // We reserve 4 words for every jump, so we _can_ link any jump to any address
        static bool canRelinkJump(void* from, void* to)
        {
            return true;
        }

        // Routines to patch jumps and calls. These routines dynamically try to
        // optimize the branch instructions used, using relative jumps if possible
        // or falling back to a full-blown lis,ori,mtctr,b[c]ctr[l] sequence if
        // needed for full 32-bit addressability.

        // 'link' and 'patch' methods are for use on unprotected code, such as
        // the code within the AssemblerBuffer and code being patched by the
        // patch buffer. Once code has been finalized it is (platform support
        // permitting) within a non-writable region of memory; to modify the
        // code in an execute-only executable pool the 'repatch' and 'relink'
        // methods should be used.

    public: // IonMonkey also uses this method directly.
        static bool patchBranch(void *stanza_start, void *target, BufferType *buffer = 0);

    public:
        static void linkCall(void* code, JmpSrc where, void* target)
        {
            void *stanza_start = (void *)(intptr_t(code) + where.m_offset - PPC_CALL_STANZA_LENGTH);
            relinkCall(stanza_start, target);
        }

        void linkJump(JmpSrc from, JmpDst to)
        {
            ASSERT(from.m_offset != -1);
            ASSERT(to.m_offset != -1);
            intptr_t code = intptr_t(m_buffer.data());
            void *stanza_start = (void *)(code + from.m_offset - PPC_BRANCH_STANZA_LENGTH);
            void *target = (void *)(code + to.m_offset);
#if DEBUG
            GenericAssembler::staticSpew("##link2    ((%p)) jumps to ((%p))", stanza_start, target);
#endif
            bool needRelocation = patchBranch(stanza_start, target, &m_buffer);

            if (needRelocation)
                m_largeJumps.append(uint32_t(stanza_start) - uint32_t(m_buffer.data()));
        }

        // Ion PatchJump() calls this.
        void directPatchJump(void *stanza_start, void *target) {
#if DEBUG
            GenericAssembler::staticSpew(
        "##IonPatchJump    ((%p)) jumps to ((%p))", stanza_start, target);
#endif
            bool needRelocation = patchBranch(stanza_start, target, &m_buffer);
            if (needRelocation)
                m_largeJumps.append(uint32_t(stanza_start) - uint32_t(m_buffer.data()));
        }

        static void linkJump(void* code, JmpSrc from, void* target)
        {
            ASSERT(from.m_offset != -1);
            void *stanza_start = (void *)(intptr_t(code) + from.m_offset - PPC_BRANCH_STANZA_LENGTH);
#if DEBUG
            GenericAssembler::staticSpew("##link3    ((%p)) jumps to ((%p))", stanza_start, target);
#endif
            bool needRelocation = patchBranch(stanza_start, target);
#if !defined(USE_970_BRANCHING)
            ASSERT(!needRelocation);
#endif
            (void)needRelocation; // suppress unused variable warnings
        }

        static void relinkCall(void* stanza_start, void* target)
        {
#if DEBUG
            GenericAssembler::staticSpew("##linkCall ((from=%p)) ((to=%p))", stanza_start, target);
#endif
            bool needRelocation = patchBranch(stanza_start, target);
#if !defined(USE_970_BRANCHING)
            ASSERT(!needRelocation);
#endif
            (void)needRelocation; // suppress unused variable warnings
        }

        static void relinkJump(void* from, void* target)
        {
            void *stanza_start = (void *)(intptr_t(from) - PPC_BRANCH_STANZA_LENGTH);
#if DEBUG
            GenericAssembler::staticSpew("##relink   ((%p)) jumps to ((%p))", stanza_start, target);
#endif
            bool needRelocation = patchBranch(stanza_start, target);
#if !defined(USE_970_BRANCHING)
            ASSERT(!needRelocation);
#endif
            (void)needRelocation; // suppress unused variable warnings
        }

        // Support routines for AssemblerBufferWithConstantPool
        static uint32_t patchConstantPoolLoad(uint32_t insn, int constantIndex)
        {
            // We store constant indicies as negative numbers, so that we
            // can tell whether the pool has been allocated or not -- since
            // references to the pool are always positive offsets. We also
            // left shift the index by two to convert it into a pseudo-offset,
            // so that the bits 30 and 31 are free for AA and LK.
            return insn | ((-constantIndex << 2) & 0xFFFC);
        }
        
        static void patchConstantPoolLoad(void *loadAddr, void *constantPoolBase)
        {
            uint32_t *insn = static_cast<uint32_t *>(loadAddr);
            // bits 30 and 31 are AA and LK, _not_ part of the index
            int16_t ix = -(int16_t(*insn & 0xFFFC) >> 2);
            ASSERT(ix >= 0);

            int32_t offset = int32_t(constantPoolBase) - int32_t(loadAddr) + 4*ix;
            ASSERT(offset > 0 && offset <= 0x7FFF);
            *insn = (*insn & 0xFFFF0000) | offset;
        }
        
        static uint32_t placeConstantPoolBarrier(int size)
        {
            ASSERT(!(size & 3));
#if DEBUG
            GenericAssembler::staticSpew("## place constant pool buffer (size=%d)", size);
#endif
            return (PPC_b | ((size + 4) & 0x03FFFFFC));
        }
        
        enum ConstantBufferPaddingValues {
            padForAlign8 = 0,
            padForAlign16 = 0,
            padForAlign32 = PPC_xoris // use a valid but rarely-used instruction
        };

    private:
        static char const * nGPR(RegisterID reg)
        {
            ASSERT(reg <= 31);
            ASSERT(reg >= 0);
            static char const *names[] = {
                "r0",  "sp",  "r2",  "r3",  "r4",  "r5",  "r6",  "r7",
                "r8",  "r9",  "r10", "r11", "r12", "r13", "r14", "r15",
                "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
                "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31"
            };
            return names[reg];
        }

        static char const * nFPR(FPRegisterID reg)
        {
            ASSERT(reg <= 31);
            ASSERT(reg >= 0);
            static char const *names[] = {
                "f0",  "f1",  "f2",  "f3",  "f4",  "f5",  "f6",  "f7",
                "f8",  "f9",  "f10", "f11", "f12", "f13", "f14", "f15",
                "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
                "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31"
            };
            return names[reg];
        }
        
        static char const * nCR(CRegisterID reg)
        {
            ASSERT(reg <= 7);
            ASSERT(reg >= 0);
            static char const *names[] = {
                "cr0", "cr1", "cr2", "cr3", "cr4", "cr5", "cr6", "cr7"
            };
            return names[reg];
        }

        static char const * nSPR(SPRegisterID reg)
        {
            // XXX: we don't handle VRSAVE with this, but we don't use it yet.
            ASSERT(reg >= 1);
            ASSERT(reg <= 9);
            static char const *names[] = {
                "", "xer", "", "", "", "", "", "", "lr", "ctr"
            };
            return names[reg];
        }

        // Which absolute bit number does a condition register + Condition pair
        // refer to?
        static uint8_t crBit(CRegisterID cr, Condition cond)
        {
            return (cr << 2) + ((cond & 0xf0) >> 4);
        }
        
        static uint8_t crBit(CRegisterID cr, DoubleCondition cond)
        {
            return (cr << 2) + ((cond & 0xf0) >> 4);
        }

        // If we are told to link a jump and can't fit it into a relative
        // branch, we record the offset of the start of that branch stanza
        // here. On G3/G4, the stanza itself is temporarily patched to
        //   trap
        //   b <trampoline>
        // which is then finally patched at link time to
        //  lis r0, HI(target)
        //  b <trampoline>
        // [...]
        // trampoline:
        //  ori r0, LO(target)
        //  mtctr r0
        //  bctr
        // On G5, it is simpler:
        //   trap
        //   0 (yes, 32 bits of zeros)
        //   the branch _code_ offset (NOT the relative offset) as a int32_t
        //   b[c]ctr[l]
        // and then at executableCopy() time we will repatch the stanza to
        // the proper lis,ori,mtctr,b[c]ctr[l] form.

        Jumps m_largeJumps;
        void finalizeAbsoluteJumps(void *finalCodeAddr);

        /* The remainder are Ion methods only (YARR does not use these). */

    public:

        JmpDst align(int alignment)
        {
            // We can't fix alignment that isn't word-sized this way.
            ASSERT(!(alignment & 3));

            while (!m_buffer.isAligned(alignment)) { x_nop(); }
            return label();
        }


        void setInt32(void *where, int32_t value)
        {
            int32_t *w = (int32_t *)where;
            *w = value;
        }

        static int32_t getInt32(void* where)
        {
            return reinterpret_cast<int32_t*>(where)[0];
        }

        // Directly emit a byte into the instruction stream. Used where
        // we need to insert instructions or branches we already know and
        // are invariant.
        void emitInt32(int32_t value)
        {
            m_buffer.putInt(value);
        }

        // Ion branch management.
        bool nextJump(const JmpSrc& from, JmpSrc* next, uint32_t bopcode);
        uint32_t setNextJump(const JmpSrc& from, const JmpSrc &to);
        void linkPendedJump(JmpSrc from, JmpDst to);

        // Additional Ion branch types. We use the extended mnemonics for
        // these instructions so that they don't conflict with existing
        // branch operations. These branches are implemented in terms of b()
        // and bc(), meaning they require a precalculated offset, NOT a label!
        // TODO: Allow these to accept labels, computing them against size()
        // and appropriately assert if the displacement is too large. However,
        // we have to figure out how to do this for forward branches.

        // This is used by the trampoline prologue during the args copy.
        void x_bdnz(int32_t offset,
                BranchLikelihood likely, BranchLink is_link) {
            // Maximum displacement is a signed 16-bit word-aligned quantity.
            ASSERT(!(offset & 0x03));
            ASSERT(offset > -32767 && offset < 32768);

            // Bitmask the offset so that we don't get sign extension when
            // computing the instruction. The offset field is "16 bit" (14,
            // but limited to word).
            uint32_t offset_bm = offset & 0xfffc;
            // Fixed values from the table in OPPCC appendix A, page 370.
            // BO = 16, BI = 0
            JSPW((PPC_bc | (16 | likely)<<21 | 0<<16 | offset_bm | is_link),
                "bdnz%s%s %d", (is_link)?"l":"", (likely)?"+":"", offset);
        }

        inline void x_beq(CRegisterID cr, int32_t offset,
                BranchLikelihood likely, BranchLink is_link) {
           bc(cr, ConditionEQ, offset, likely, is_link);
        }
        inline void x_bne(CRegisterID cr, int32_t offset,
                BranchLikelihood likely, BranchLink is_link) {
           bc(cr, ConditionNE, offset, likely, is_link);
        }
        // Special form for Ion.
        inline void x_b(int32_t offset, BranchLink is_link) {
            b(offset, RelativeBranch, is_link);
        }
    }; // class PPCAssembler

} // namespace JSC

#endif // ENABLE(ASSEMBLER) && CPU(PPC)

#endif // PPCAssembler_h
