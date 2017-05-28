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

#include "assembler/wtf/Platform.h"

#if WTF_CPU_PPC || WTF_CPU_PPC64

#include "PPCAssembler.h"

namespace JSC {

static inline bool branchIsUnconditional(int32_t instr)
{
    if ((instr & PPCAssembler::PPC_MAJOR_OPCODE_MASK) == PPCAssembler::PPC_b)
        return true;
    if (instr == PPCAssembler::PPC_bctr)
        return true;
    return false;
}

static inline bool offsetFitsInB(int32_t offset)
{
#if defined(_PPC970_)
    return !(offset & 0xFE000000) || ((offset & 0xFE000000) == 0xFE000000);
#else
    return ((offset & 0xFE000003) == 0) || ((offset & 0xFE000003) == 0xFE000000);
#endif
}

// Helper functions to put together (but _not_ emit) instructions
static inline int32_t make_lis(PPCAssembler::RegisterID rd, int16_t imm)
{
    return (PPCAssembler::PPC_addis | int(rd) << 21 | uint16_t(imm));
}

static inline int32_t make_li(PPCAssembler::RegisterID rd, int16_t imm)
{
    return (PPCAssembler::PPC_addi | int(rd) << 21 | uint16_t(imm));
}

static inline int32_t make_ori(PPCAssembler::RegisterID rd, uint16_t imm)
{
    return (PPCAssembler::PPC_ori | int(rd) << 21 | int(rd) << 16 | imm);
}

static inline int32_t make_nop()
{
    return PPCAssembler::PPC_nop;
}

static inline int32_t make_trap()
{
    return PPCAssembler::PPC_trap;
}

static inline int32_t make_mtctr(PPCAssembler::RegisterID rd)
{
    return (PPCAssembler::PPC_mtspr | int(rd) << 21 | PPC_SPR(PPCRegisters::ctr) << 11);
}

static inline int32_t make_b(int32_t target, PPCAssembler::BranchAddressType bt, PPCAssembler::BranchLink lk)
{
    return (PPCAssembler::PPC_b | (target & 0x03FFFFFC) | bt | lk);
}

static inline int32_t make_bctr(PPCAssembler::BranchLink lk)
{
    return (PPCAssembler::PPC_bctr | lk);
}

static inline int32_t make_bc_like(int32_t oldInstr, int32_t byteOffset)
{
    // preserve the BO, BI, and LK fields from oldInstr
    return (PPCAssembler::PPC_bc | (oldInstr & 0x03FF0001) | (byteOffset & 0xFFFC));
}

static inline int32_t make_bcctr_like(int32_t oldInstr)
{
    // preserve the BO, BI, and LK fields from oldInstr
    return (PPCAssembler::PPC_bcctr | (oldInstr & 0x03FF0001));
}

#undef PPC_DEBUG_BRANCHES
#ifdef PPC_DEBUG_BRANCHES
#define dbrPrintf(x, ...) fprintf(stderr, (x), ##__VA_ARGS__)
#else
#define dbrPrintf(x, ...)
#endif

#define PPC_FORCE_FAR_BRANCHES (0)

// The core patch-an-immediate-load routine.
void PPCAssembler::patchImmediate(uint32_t *code, uint32_t imm) 
{
    ASSERT((code[0] & PPC_MAJOR_OPCODE_MASK) == PPC_addis);
    ASSERT((code[1] & PPC_MAJOR_OPCODE_MASK) == PPC_ori);

    // Pick out the target register from the preexisting lis instruction
    uint32_t reg = (code[0] >> 21) & 31;

    // and do the patch
    code[0] = make_lis(RegisterID(reg), imm >> 16);
    code[1] = make_ori(RegisterID(reg), imm & 0xFFFF);
    ExecutableAllocator::cacheFlush(code, 8);
}

enum RelocationNeed {
    DontForceRelocation = 0,
    ForceRelocation = 1
};

// returns true if the branch needs a relocation entry
static inline bool writeRelativeTrampoline(int32_t *trampoline, uint32_t buffer,
    void *target, RelocationNeed forceRelocation = DontForceRelocation)
{
    int32_t offset = int32_t(target) - int32_t(trampoline);
    
    if (forceRelocation || !offsetFitsInB(offset)) {
        trampoline[0] = int32_t(target) - int32_t(buffer);
        trampoline[1] = make_trap();
        trampoline[2] = make_bctr(PPCAssembler::DontLinkBranch);
        return true;
    }

    trampoline[0] = make_b(offset, PPCAssembler::RelativeBranch, PPCAssembler::DontLinkBranch);
    return false;
}

static inline void writeAbsoluteTrampoline(int32_t *trampoline, void *target_)
{
    int32_t target = int32_t(target_);

    if (offsetFitsInB(target)) {
        trampoline[0] = make_b(target, PPCAssembler::AbsoluteBranch, PPCAssembler::DontLinkBranch);
    } else {
        trampoline[0] = make_ori(PPCRegisters::r0, uint32_t(target) & 0xFFFF);
        trampoline[1] = make_mtctr(PPCRegisters::r0);
        trampoline[2] = make_bctr(PPCAssembler::DontLinkBranch);
    }
}

// The optimizing branch-patcher.
// There are two completely different versions, one for G3/G4 and one for G5.
// For why, see PPCAssembler.h.
bool PPCAssembler::patchBranch(void *stanza_start, void *target, BufferType *codeBuffer)
{
    int32_t *stanza = reinterpret_cast<int32_t *>(stanza_start);
    
#ifdef _PPC970_

	// G5 version.
	
    ASSERT(stanza[0] == PPC_trap || stanza[0] == PPC_SKIP_THIS_JUMP
        || (stanza[0] & PPC_MAJOR_OPCODE_MASK) == PPC_b
        || (stanza[0] & PPC_bc) == PPC_bc
        || (stanza[0] & PPC_addis) == PPC_addis);
    ASSERT(stanza[1] == PPC_nop || stanza[1] == 0
        || (stanza[1] & PPC_ori) == PPC_ori);
    
    // Figure out whether the stanza and target are each in our accumulation buffer.
    // Since we can take labels without growing the buffer, the target can actually
    // be at (buffer + buflen) and still be valid.
    const uint32_t buffer = codeBuffer ? uint32_t(codeBuffer->data()) : 0;
    const uint32_t buflen = codeBuffer ? uint32_t(codeBuffer->size()) : 0;

    bool sourceInBuffer = buffer && uint32_t(stanza_start) >= uint32_t(buffer)
                        && uint32_t(stanza_start) < uint32_t(buffer) + buflen;
    bool targetInBuffer = buffer && uint32_t(target) >= uint32_t(buffer)
                        && uint32_t(target) <= uint32_t(buffer) + buflen;

    int32_t offset = int32_t(target) - int32_t(stanza_start);
    bool needRelocation = false;

    // figure out where the actual branch instruction is in the stanza:
    // - if the last instruction is not a NOP, it's the branch
    // - otherwise, the first instruction is the branch
    int32_t origBranch = stanza[3];
    if (origBranch == PPC_nop)
        origBranch = stanza[0];
    
    dbrPrintf(">>> patching %x (%x; branch=%x) to target %x (sIB=%d, tIB=%d)\n",
        uint32_t(stanza_start), stanza[0], origBranch, uint32_t(target), sourceInBuffer, targetInBuffer);

    // handle unconditional branches first, since we have a couple extra
    // options for dealing with them
    if (branchIsUnconditional(origBranch)) {
        if (offsetFitsInB(offset) && (!sourceInBuffer || targetInBuffer)) {
            // either the code won't move, so we can use a relative branch even
            // to outside the buffer, or it's a branch to inside the buffer and
            // can so be relative
            dbrPrintf("  > unconditional, relative b\n");
            stanza[0] = make_b(offset, RelativeBranch, (origBranch & 1) ? LinkBranch : DontLinkBranch);
            stanza[1] = make_nop();
            stanza[2] = make_nop();
            stanza[3] = make_nop();
        } else if (!targetInBuffer && offsetFitsInB(int32_t(target))) {
            // we're linking to a pre-existing function, so the _target_ won't move
            // when we relocate: we can use an absolute branch
            dbrPrintf("  > unconditional, absolute b\n");
            stanza[0] = make_b(int32_t(target), AbsoluteBranch, (origBranch & 1) ? LinkBranch : DontLinkBranch);
            stanza[1] = make_nop();
            stanza[2] = make_nop();
            stanza[3] = make_nop();
        } else if (!targetInBuffer) {
            // target won't move: no need to relocate it
            dbrPrintf("  > unconditional bctr\n");
            stanza[0] = make_lis(PPCRegisters::r0, uint32_t(target) >> 16);
            stanza[1] = make_ori(PPCRegisters::r0, uint32_t(target) & 0xFFFF);
            stanza[2] = make_mtctr(PPCRegisters::r0);
            stanza[3] = make_bctr((origBranch & 1) ? LinkBranch : DontLinkBranch);
        } else {
            // an oversized relative branch: record it for relocation
            dbrPrintf("  > unconditional relocated bctr\n");
            stanza[0] = make_trap();
            stanza[1] = 0;
            stanza[2] = int32_t(target) - int32_t(buffer);
            stanza[3] = make_bctr((origBranch & 1) ? LinkBranch : DontLinkBranch);

            needRelocation = true;
        }

        ExecutableAllocator::cacheFlush(stanza, 16);
        return needRelocation;
    }

    // conditional branches: either bc or bcctr
    if (offset >= -32768 && offset <= 32767 && (!sourceInBuffer || targetInBuffer)) {
        // we can fit this branch into a simple relative branch, and won't
        // need to relocate it later
        dbrPrintf("  > conditional, relative bc\n");
        stanza[0] = make_bc_like(origBranch, offset);
        stanza[1] = make_nop();
        stanza[2] = make_nop();
        stanza[3] = make_nop();
    } else if (targetInBuffer) {
        // an oversized relative branch: record it for relocation
        dbrPrintf("  > conditional relocated bcctr\n");
        stanza[0] = make_trap();
        stanza[1] = 0;
        stanza[2] = int32_t(target) - int32_t(buffer);
        stanza[3] = make_bcctr_like(origBranch);

        needRelocation = true;
    } else {
        // no relocation will be needed: fall back to the full stanza
        dbrPrintf("  > conditional bcctr\n");
        stanza[0] = make_lis(PPCRegisters::r0, uint32_t(target) >> 16);
        stanza[1] = make_ori(PPCRegisters::r0, uint32_t(target) & 0xFFFF);
        stanza[2] = make_mtctr(PPCRegisters::r0);
        stanza[3] = make_bcctr_like(origBranch);
    }

    ExecutableAllocator::cacheFlush(stanza, 16);
    return needRelocation;

#else

    // G3-G4 version.
    
    // bits 30-31 of trampoline offset are actually the AA and LK bits if
    // stanza[1] is a branch instruction, and so must be masked off
    int16_t trampolineOffset = int16_t(stanza[1] & 0xFFFC);
    int16_t trampolineIndex = -(int16_t(stanza[1] & 0xFFFC) >> 2);
    bool trampolineIsValid = (trampolineOffset > 0);

    int32_t *trampoline = reinterpret_cast<int32_t *>(uint32_t(stanza) 
                                + 4 + trampolineOffset);

    // 1st instruction is either the branch or the lis for the far branch
    // 2nd instruction is either an addi recording the position of the trampoline
    //     or a branch _to_ the trampoline
    //     Note that the trampoline branch is always just a jump: the branch with
    //     linkage or the conditional branch is always in the inline stanza.
    ASSERT((stanza[0] & PPC_MAJOR_OPCODE_MASK) == PPC_b
        || (stanza[0] & PPC_bc) == PPC_bc
        || stanza[0] == make_trap()
        || (stanza[0] & PPC_addis) == PPC_addis);
    ASSERT((stanza[1] & PPC_MAJOR_OPCODE_MASK) == PPC_b
        || (stanza[1] & PPC_bc) == PPC_bc
        || (stanza[1] & PPC_addi) == PPC_addi);
    
    // Figure out whether the stanza and target are each in our accumulation buffer.
    // Since we can take labels without growing the buffer, the target can actually
    // be at (buffer + buflen) and still be an internal branch.
    
    const uint32_t buffer = codeBuffer ? uint32_t(codeBuffer->data()) : 0;
    
    // we don't want to accidentally allocate the constant pool here!
    const uint32_t buflen = codeBuffer ? uint32_t(codeBuffer->uncheckedSize()) : 0;
    const bool sourceInBuffer = buffer && uint32_t(stanza_start) >= buffer
                        && uint32_t(stanza_start) < buffer + buflen;
    const bool targetInBuffer = buffer && uint32_t(target) >= buffer
                        && uint32_t(target) <= buffer + buflen;

    int32_t offset = int32_t(target) - int32_t(stanza_start);
    bool needRelocation = false;

    // Figure out where the actual branch instruction is:
    // if the first instruction is not an lis, it's the branch; otherwise
    // the branch is the second instruction
    int32_t origBranch = stanza[0];
    if ((origBranch & PPC_addis) == PPC_addis) {
        origBranch = stanza[1];
    }
    
    dbrPrintf(">>> patching %x (trampolineOffset=%d; branch=%x) to target %x (sIB=%d, tIB=%d)\n",
        uint32_t(stanza_start), trampolineOffset, origBranch,
        uint32_t(target), sourceInBuffer, targetInBuffer);

    // Handle unconditional branches first, since we have an extra
    // option for dealing with them.
    if (branchIsUnconditional(origBranch)) {
        const BranchLink linkage = (origBranch & 1) ? LinkBranch : DontLinkBranch;

#ifdef PPC_DEBUG_BRANCHES
        const char *linkSuffix = (origBranch & 1) ? "l" : "";
#endif

        if (offsetFitsInB(offset) && (!sourceInBuffer || targetInBuffer)
            && !PPC_FORCE_FAR_BRANCHES) {
            // either the code won't move, so we can use a relative branch even
            // to outside the buffer, or it's a branch to inside the buffer and
            // can so be relative
            dbrPrintf("  > unconditional, relative b%s\n", linkSuffix);
            stanza[0] = make_b(offset, RelativeBranch, linkage);
            stanza[1] = make_li(PPCRegisters::r0, stanza[1] & 0xFFFC);
        } else if (!targetInBuffer && offsetFitsInB(int32_t(target))
            && !PPC_FORCE_FAR_BRANCHES) {
            // we're linking to a pre-existing function, so the _target_ won't move
            // when we relocate: we can use an absolute branch
            dbrPrintf("  > unconditional, absolute b%s\n", linkSuffix);
            stanza[0] = make_b(int32_t(target), AbsoluteBranch, linkage);
            stanza[1] = make_li(PPCRegisters::r0, stanza[1] & 0xFFFC);
        } else if (!targetInBuffer) {
            // target won't move: no need to relocate it
            dbrPrintf("  > unconditional, absolute b%s via trampoline\n", linkSuffix);
            if (!trampolineIsValid) {
                dbrPrintf(" << no trampoline allocated, writing to pool\n");
                trampoline = reinterpret_cast<int32_t *>(&(codeBuffer->poolAddress()[trampolineIndex]));
            }

            stanza[0] = make_lis(PPCRegisters::r0, uint32_t(target) >> 16);
            stanza[1] = make_b(stanza[1] & 0xFFFC, RelativeBranch, linkage);
            writeAbsoluteTrampoline(trampoline, target);

            ExecutableAllocator::cacheFlush(trampoline, 12);
        } else {
            // an oversized relative branch: record it for relocation
            dbrPrintf("  > unconditional, relocated b%s via trampoline\n", linkSuffix);
            RelocationNeed rn = DontForceRelocation;

            if (!trampolineIsValid) {
                dbrPrintf(" << no trampoline allocated, writing to pool\n");
                trampoline = reinterpret_cast<int32_t *>(&(codeBuffer->poolAddress()[trampolineIndex]));
                rn = ForceRelocation;
            }

            needRelocation = writeRelativeTrampoline(trampoline, buffer, target, rn);

            if (needRelocation)
                stanza[0] = make_trap();
            else
                stanza[0] = make_nop();

            stanza[1] = make_b(stanza[1] & 0xFFFC, RelativeBranch, linkage);
        }

        ExecutableAllocator::cacheFlush(stanza, 8);
        return needRelocation;
    }

    // conditional branches: either bc or bcctr
    if (offset >= -32768 && offset <= 32767
        && (!sourceInBuffer || targetInBuffer) && !PPC_FORCE_FAR_BRANCHES) {
        // we can fit this branch into a simple relative branch, and won't
        // need to relocate it later
        dbrPrintf("  > conditional, relative bc\n");
        stanza[0] = make_bc_like(origBranch, offset);
        stanza[1] = make_li(PPCRegisters::r0, stanza[1] & 0xFFFC);
    } else if (targetInBuffer) {
        // an oversized relative branch: record it for relocation
        dbrPrintf("  > conditional, relocated bc via trampoline\n");
        RelocationNeed rn = DontForceRelocation;

        if (!trampolineIsValid) {
            dbrPrintf(" << no trampoline allocated, writing to pool\n");
            trampoline = reinterpret_cast<int32_t *>(&(codeBuffer->poolAddress()[trampolineIndex]));
            rn = ForceRelocation;
        }

        needRelocation = writeRelativeTrampoline(trampoline, buffer, target, rn);

        if (needRelocation)
            stanza[0] = make_trap();
        else
            stanza[0] = make_nop();

        stanza[1] = make_bc_like(origBranch, stanza[1] & 0xFFFC);
    } else {
        // no relocation will be needed: fall back to the full stanza
        dbrPrintf("  > conditional, absolute bc via trampoline\n");
        if (!trampolineIsValid) {
            dbrPrintf(" << no trampoline allocated, writing to pool\n");
            trampoline = reinterpret_cast<int32_t *>(&(codeBuffer->poolAddress()[trampolineIndex]));
        }

        stanza[0] = make_lis(PPCRegisters::r0, uint32_t(target) >> 16);
        stanza[1] = make_bc_like(origBranch, stanza[1] & 0xFFFC);
        writeAbsoluteTrampoline(trampoline, target);

        ExecutableAllocator::cacheFlush(trampoline, 12);
    }

    ExecutableAllocator::cacheFlush(stanza, 8);
    return needRelocation;
#endif
}

// Similarly, there is a separate PowerPC 970 path for this function.
void PPCAssembler::finalizeAbsoluteJumps(void *finalCodeAddr)
{
#if !defined(_PPC970_)
    ASSERT(m_buffer.sizeOfConstantPool() == 0);
#endif
    dbrPrintf("<<< Finalizing jumps from %x to %x\n", uint32_t(m_buffer.data()), uint32_t(finalCodeAddr));

    for (Jumps::Iterator iter = m_largeJumps.begin(), end = m_largeJumps.end();
                         iter != end;
                         ++iter) {
        uint32_t stanzaStartOffset = *iter;
        int32_t *stanza = reinterpret_cast<int32_t *>(uint32_t(finalCodeAddr) + stanzaStartOffset);

#ifdef _PPC970_

        dbrPrintf(">>> finalizing jump @ %x (%x,%x,%x,%x)", uint32_t(stanza), stanza[0], stanza[1], stanza[2], stanza[3]);
        if (stanza[0] != PPC_trap || stanza[1] != 0) {
            dbrPrintf(" [skipped]\n");
            continue;
        }

        int32_t target = int32_t(finalCodeAddr) + stanza[2];
        dbrPrintf(" to %u\n", target);

        stanza[0] = make_lis(PPCRegisters::r0, uint32_t(target) >> 16);
        stanza[1] = make_ori(PPCRegisters::r0, target & 0xFFFF);
        stanza[2] = make_mtctr(PPCRegisters::r0);
        // leave stanza[3] as the prexisting b[c]ctr[l]

#else

        int32_t *trampoline = reinterpret_cast<int32_t *>(uint32_t(stanza) 
                                    + 4 + (stanza[1] & 0x7FFC));

        dbrPrintf(">>> finalizing jump @ %x (%x,%x)",
            uint32_t(stanza), stanza[0], stanza[1]);
        if (stanza[0] != PPC_trap) {
            dbrPrintf(" [skipped]\n");
            continue;
        }

        int32_t target = int32_t(finalCodeAddr) + trampoline[0];
        dbrPrintf(" to %x\n", target);

        stanza[0] = make_lis(PPCRegisters::r0, uint32_t(target) >> 16);
        if (writeRelativeTrampoline(trampoline, uint32_t(finalCodeAddr),
                reinterpret_cast<void *>(target))) {
            // Writing it as a relative branch doesn't work, so write it as
            // absolute.
            // This is now safe for all branches, since the code has had its
            // final relocation.
            writeAbsoluteTrampoline(trampoline, reinterpret_cast<void *>(target));
        }

#endif
    }
}

// Ion methods

bool
PPCAssembler::nextJump(const JmpSrc& from, JmpSrc* next, uint32_t bopcode) {
        // Sanity check - if the assembler has OOM'd, it will overwrite
        // its internal buffer and thus our links could be garbage.
        if (oom())
            return false;

        char* code = reinterpret_cast<char*>(m_buffer.data());
        int32_t offset = getInt32(code + from.m_offset -
            PPC_BRANCH_STANZA_LENGTH);

        // Replace the offset word with the original opcode.
        setInt32(code + from.m_offset - PPC_BRANCH_STANZA_LENGTH, bopcode);

        // If the -1 flag was set, the shift should keep it negative.
        if (offset < 0) return false;

        // Otherwise, return the offset.
        uint32_t x = offset;
        *next = JmpSrc(x);
        return true;
}

uint32_t
PPCAssembler::setNextJump(const JmpSrc& from, const JmpSrc &to) {
        // Sanity check - if the assembler has OOM'd, it will overwrite
        // its internal buffer and thus our links could be garbage.
        if (oom())
            return;

        char* code = reinterpret_cast<char*>(m_buffer.data());
        // We want to overwrite either the x_skip_this_jump or the b .+8.
        // Depending on the trampoline, that could be a bc, but it's
        // always some sort of branch.
        uint32_t *ptr = (uint32_t *)(code +
            from.m_offset - PPC_BRANCH_STANZA_LENGTH);
        uint32_t bopcode = *ptr;
#if DEBUG
        GenericAssembler::staticSpew("##setNextJump (%08x -> %08x) (%08x)\n",
            ptr, to.m_offset, bopcode);
#endif
        ASSERT(((*ptr & PPC_MAJOR_OPCODE_MASK) == PPC_b) ||
               ((*ptr & PPC_MAJOR_OPCODE_MASK) == PPC_bc));
        setInt32(ptr, to.m_offset);

        // Return the original branch instruction so that nextJump can
        // restore it.
        return bopcode;
}

void
PPCAssembler::linkPendedJump(JmpSrc from, JmpDst to) {
        ASSERT(from.m_offset != -1);
        ASSERT(to.m_offset != -1);
#if DEBUG
        intptr_t code = intptr_t(m_buffer.data());
        uint32_t *stanza_start = (uint32_t *)(code + from.m_offset
            - PPC_BRANCH_STANZA_LENGTH);
            GenericAssembler::staticSpew(
                "##linkPendedJump @ %08x", stanza_start);
#endif
        linkJump(from, to);
}

} // namespace JSC

#endif
