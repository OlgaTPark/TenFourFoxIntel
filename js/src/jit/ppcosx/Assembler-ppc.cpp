#include "assembler/assembler/PPCAssembler.h"
#include "jit/IonMacroAssembler.h"
#include "Assembler-ppc.h"
#include "gc/Marking.h"
#include "jsutil.h"
#include "assembler/jit/ExecutableAllocator.h"
#include "jscompartment.h"
#include "jit/IonCompartment.h"

namespace js {
namespace jit {

ABIArgGenerator::ABIArgGenerator()
  : stackOffset_(0),
    usedGPRs_(0),
    usedFPRs_(0),
    current_()
{}

ABIArg
ABIArgGenerator::next(MIRType type)
{
    // TODO:
    // This does not yet handle the situation where we overflow the argregs.
    switch (type) {
      case MIRType_Int32:
      case MIRType_Pointer:
	if (usedGPRs_ == 8) // i.e., we already allocated r10
		JS_NOT_REACHED("ABIArgGenerator overflowed");
	current_ = ABIArg(Register::FromCode(usedGPRs_ + 3));
	usedGPRs_++;
	break;
      case MIRType_Double:
	if (usedFPRs_ == 12) // i.e., we already allocated f13
		JS_NOT_REACHED("ABIArgGenerator overflowed");
	current_ = ABIArg(FloatRegister::FromCode(usedFPRs_ + 1));
	usedGPRs_ += 2;
	usedFPRs_ ++;
	break;
      default:
	JS_NOT_REACHED("Unexpected argument type");
    }
    return current_;
}

// I'm not sure what to do with these yet.
const Register ABIArgGenerator::NonArgReturnVolatileReg1 = r9;
const Register ABIArgGenerator::NonArgReturnVolatileReg2 = r10;
const Register ABIArgGenerator::NonVolatileReg = r13;

void PatchJump(CodeLocationJump jump, CodeLocationLabel label) {
	JSC::PPCAssembler::patchBranch(jump.raw(), label.raw());
}

void
Assembler::finish() {
}

void
Assembler::writeCodePointer(AbsoluteLabel *label) {
// XXX
	JS_NOT_REACHED("writeCodePointer NYI");
}

void
Assembler::patchWrite_NearCall(CodeLocationLabel startLabel,
	CodeLocationLabel target)  {
	// Overwrite the given location with a direct branch. We assume that
	// the call is within the same function and therefore within the
	// maximum displacement of the b instruction (see ARM).
	uint32_t *where = (uint32_t *)startLabel.raw();
	int32_t offset = (int32_t)(target.raw() - startLabel.raw());
#if DEBUG
	IonSpew(IonSpew_Codegen,
		"patchWrite_NearCall: %p to %p (offset %i)\n",
			startLabel.raw(), target.raw(), offset);
#endif
	JS_ASSERT(offset < 65536 && offset >= -65536); // paranoid
	JS_ASSERT((offset & 3) == 0);
	*where = 0x48000000 | (offset & 0x03FFFFFC); // PPC_b | offset
	JSC::ExecutableAllocator::cacheFlush((void*)where, 4);
}

void
Assembler::patchWrite_Imm32(CodeLocationLabel dataLabel, Imm32 toWrite) {
	// Assume that dataLabel has a lis/ori we can patch from x_p_li32.
	// patchImmediate will assert if that's not the case, but let's make
	// this even more airtight.
#if DEBUG
	uint32_t *w = (uint32_t *)dataLabel.raw();
	// rA must be zero, but rD could be anything, so mask those bits.
	JS_ASSERT((*w & 0xFC1F0000) == 0x3C000000); // PPC_addis
#endif
	JSC::PPCAssembler::patchImmediate((uint32_t *)dataLabel.raw(),
		toWrite.value);
}

void
Assembler::patchDataWithValueCheck(CodeLocationLabel data, ImmWord newData,
	ImmWord expectedData) {
	// Assert that the patchable 32-bit value encoded with lis/ori at
	// location data contains expectedData before we patch it with
	// newData.
	uint32_t *lis = (uint32_t *)data.raw();
	uint32_t *ori = (uint32_t *)((uint32_t)data.raw() + 4);
#if DEBUG
	IonSpew(IonSpew_Codegen,
		"##patchDataWithValueCheck evaluating lis=%08x ori=%08x",
		(uint32_t)lis, (uint32_t)ori);
#endif

	// Assert encoding. lis must come first.
	// rA must be zero, but rD could be anything, so mask those bits.
	JS_ASSERT((*lis & 0xFC1F0000) == 0x3C000000); // PPC_addis
	// ori must come second.
	// If we were really paranoid, we'd make sure rD == rS.
	JS_ASSERT((*ori & 0xFC000000) == 0x60000000); // PPC_ori

	// Assert value. lis always encodes the most significant halfword.
	JS_ASSERT((((*lis & 0x0000FFFF) << 16) + (*ori & 0x0000FFFF)) ==
		expectedData.value);

	// Safe to patch.
	JSC::PPCAssembler::patchImmediate(lis, newData.value);
}

uint8_t *
Assembler::nextInstruction(uint8_t *cur, uint32_t *count) {
	// Move *cur to the next instruction, setting *count to the number
	// of bytes skipped (in our case, always four) if a non-null pointer
	// is supplied.
	if (count != NULL)
		count = (uint32_t *)((uint32_t)count + 4);
	return (uint8_t *)((uint32_t)cur + 4);
}

void
Assembler::executableCopy(uint8_t *buffer) {
	// Copy and flush the buffer (and any constant pools it has).
	masm.executableCopyAndFlush((void *)buffer);

	// Now, link all the pending jumps. These MUST be call/branch stanzas.
	for (size_t i = 0; i < jumps_.length(); i++) {
		RelativePatch &rp = jumps_[i];
#if DEBUG
		IonSpew(IonSpew_Codegen, "##pendingJump %08x -> %08x",
			buffer + rp.offset, rp.target);
#endif
		JSC::PPCAssembler::patchBranch(buffer + rp.offset, rp.target);
	}
	// Flush the cache manually or we get junk.
	AutoFlushCache::updateTop((uintptr_t)buffer, masm.size());
#if DEBUG
	IonSpew(IonSpew_Codegen, "##executableCopy finished to %08x\n",
		(uint32_t)buffer);
#endif
}

void
Assembler::processCodeLabels(uint8_t *code) {
    for (size_t i = 0; i < codeLabels_.length(); i++) {
	CodeLabel *label = codeLabels_[i];
	Bind(code, label->dest(), code + label->src()->offset());
    }
}

void
Assembler::copyJumpRelocationTable(uint8_t *buffer) {
    if (jumpRelocations_.length())
        memcpy(buffer, jumpRelocations_.buffer(), jumpRelocations_.length());
}

void
Assembler::copyDataRelocationTable(uint8_t *buffer) {
    if (dataRelocations_.length())
        memcpy(buffer, dataRelocations_.buffer(), dataRelocations_.length());
}

void
Assembler::copyPreBarrierTable(uint8_t *dest) {
    if (preBarriers_.length())
        memcpy(dest, preBarriers_.buffer(), preBarriers_.length());
}

// TODO: TraceDataRelocations needs to be aware of cmpwi/cmplwi. This is
// a relatively rare situation, but it could be helpful for low memory
// addresses.
static void
TraceDataRelocations(JSTracer *trc, uint8_t *buffer,
	CompactBufferReader &reader)
{
    while (reader.more()) {
	// Figure out the pointer encoded at this location, and then
	// pass it to GC. Unfortunately, this could be either an li, or
	// lis followed by nothing or ori. We need to support anything
	// x_li32 can generate because there is no guarantee this is a
	// patchable lis/ori generated by x_p_li32.
        size_t offset = reader.readUnsigned();

	// The instructions are located at buffer+offset.
	uint32_t *li = (uint32_t *)((uint32_t)buffer + offset);
	uint32_t *ori = (uint32_t *)((uint32_t)li + 4);
	uint32_t ptr;

	if ((*li & 0xFC1F0000) == 0x3800000) { // PPC_addi with rA == 0: li
		// This is li. The pointer is the value of the low halfword.
		ptr = (*li & 0x0000FFFF);
	} else if ((*li & 0xFC1F0000) == 0x3C000000) { // PPC_addis rA == 0
		// This is lis. The pointer's upper halfword is encoded here.
		ptr = (*li & 0x0000FFFF) << 16;
		// Check to see if the next instruction is ori. If it's not,
		// just silently continue; it must have been optimized out.
		if ((*ori & 0xFC000000) == 0x60000000) // PPC_ori
			ptr += (*ori & 0x0000FFFF);
#if DEBUG
		IonSpew(IonSpew_Codegen,
			"TraceDataRelocations: pointer at %p is %08x\n",
				li, ptr);
#endif
	} else {
#if DEBUG
		IonSpew(IonSpew_Codegen,
			"TDR: unexpected instruction %p %08x\n", li, *li);
#endif
		JS_ASSERT(0);
	}
        // No barrier needed since these are constants.
	void *fptr = (void *)ptr;
        gc::MarkGCThingUnbarriered(trc, reinterpret_cast<void **>(&fptr),
		"ppc32ptr");
    }
}

void
Assembler::TraceDataRelocations(JSTracer *trc, IonCode *code,
	CompactBufferReader &reader)
{
	// gcc hate me
	js::jit::TraceDataRelocations(trc, code->raw(), reader);
}

class RelocationIterator
{
    CompactBufferReader reader_;
    uint32_t offset_;

  public:
    RelocationIterator(CompactBufferReader &reader)
      : reader_(reader)
    { }

    bool read() {
        if (!reader_.more())
            return false;
        offset_ = reader_.readUnsigned();
        return true;
    }

    uint32_t offset() const {
        return offset_;
    }
};

static uint32_t *
branchInstToAddr(uint32_t *where)
{
	if ((*where & 0xF4000000) == 0x40000000) {
		// This is a branch.
		bool absolute = ((*where & 0x00000002) == 2);

		// Extract the address from the branch. If it is not absolute
		// (AA == 0), then it is an offset, and add *where to it.
		// This could be signed.
		uint32_t *target;
		if ((*where & 0xFC000000) == 0x40000000) { // bc
			int32_t disp = *where & 0x0000FFFC;
			disp = (disp > 32767) ? (disp - 65536) : disp;
			target = (absolute) ? (uint32_t *)disp :
				(uint32_t *)(disp + (uint32_t)where);
		} else { // b
			int32_t disp = *where & 0x03FFFFFC;
			disp = (disp > 33554431) ? (disp - 67108864) : disp;
			target = (absolute) ? (uint32_t *) disp :
				(uint32_t *)(disp + (uint32_t)where);
		}
		return target;
	}
	JS_NOT_REACHED("branchInstToAddr asked to eval not branch");
	return nullptr;
}

static IonCode *
CodeFromJump(uint32_t *where)
{
	// Given a location to a call/branch stanza, figure out how it was
	// encoded by patchBranch, and return the IonCode address it
	// corresponds to. Depending on how good a job patchBranch did,
	// this could be b(c)(l), a branch to an OOL trampoline (not G5)
	// or a lis/ori/mtctr/(nops)/b(c)ctr(l). We will only evaluate a
	// fully formed and targetted branch.
#if DEBUG
	IonSpew(IonSpew_Codegen,
		"CodeFromJump: evaluating target %p", where);
#endif

	JS_ASSERT(
		((*where & 0xF4000000) == 0x40000000) || // b, bc
		((*where & 0xFC1F0000) == 0x3C000000));  // lis
	uint32_t *ori = (uint32_t *)((uint32_t)where + 4);
	JS_ASSERT(*ori != 0x7FE00008); // must not have a trap!

	if ((*where & 0xF4000000) == 0x40000000) {
		// Extract the address from the branch. If it is not absolute
		// (AA == 0), then it is an offset, and add *where to it.
		// This could be signed.
		uint32_t target = (uint32_t)branchInstToAddr(where);
#if DEBUG
		IonSpew(IonSpew_Codegen,
		"CodeFromJump: b/bc target %p yielded %08x\n", where, target);
#endif
		return IonCode::FromExecutable((uint8_t *)target);
	} else {
		// This must be a lis. If the next instruction is a branch,
		// it's to a trampoline (on G3/G4). If the next instruction
		// is an ori, it's the target.
		if ((*ori & 0xF4000000) == 0x40000000) {
			// This is a trampoline branch. The ori is at the
			// branch target location.
			ori = branchInstToAddr(ori);
		} else {
			// Must be ori!
			JS_ASSERT((*ori & 0xFC000000) == 0x60000000);
		}
		uint32_t target = ((*where & 0x0000FFFF) << 16) +
			(*ori & 0x000FFFF);
#if DEBUG
		IonSpew(IonSpew_Codegen,
		"CodeFromJump: l/o target %p yielded %08x\n", where, target);
#endif
		return IonCode::FromExecutable((uint8_t *)target);
	}

	// placate the compiler
	JS_NOT_REACHED("CodeFromJump went off the rails");
	return IonCode::FromExecutable((uint8_t *)0);
}

// Right now CodeFromJump is predicated on this assertion. If we have
// to change that, we need more complex logic to determine what exactly
// we're working on.
JS_STATIC_ASSERT(PPC_CALL_STANZA_LENGTH == PPC_BRANCH_STANZA_LENGTH);

void
Assembler::TraceJumpRelocations(JSTracer *trc, IonCode *code,
	CompactBufferReader &reader)
{
    RelocationIterator iter(reader);
    while (iter.read()) {
	// The notion is similar to TraceDataRelocations: we figure out
	// the target of a branch and mark it unbarriered.
	// Assume the location we get is offset by the stanza length.
	uint32_t *where = (uint32_t *)(code->raw() + iter.offset()
		- PPC_CALL_STANZA_LENGTH);
        IonCode *child = CodeFromJump(where);
        MarkIonCodeUnbarriered(trc, &child, "ppc32tjr");
#if DEBUG
        JS_ASSERT(child == CodeFromJump(where));
#endif
    }
}

void
Assembler::trace(JSTracer *trc) {
    for (size_t i = 0; i < jumps_.length(); i++) {
        RelativePatch &rp = jumps_[i];
        if (rp.kind == Relocation::IONCODE) {
            IonCode *code = IonCode::FromExecutable((uint8_t *)rp.target);
            MarkIonCodeUnbarriered(trc, &code, "ppc32trace");
            JS_ASSERT(code == IonCode::FromExecutable((uint8_t *)rp.target));
        }
    }
    if (dataRelocations_.length()) {
        CompactBufferReader reader(dataRelocations_);
        js::jit::TraceDataRelocations(trc, masm.buffer(), reader);
    }
}

void
Assembler::Bind(uint8_t *raw, AbsoluteLabel *label, const void *address) {
	if (label->used()) {
	    intptr_t src = label->offset();
	    do {
		// We assume that we already have setNextJump information
		// present that needs to be properly batched.
		uint32_t *where = (uint32_t *)(raw + src
			- PPC_BRANCH_STANZA_LENGTH);
	        intptr_t next = (intptr_t)JSC::PPCAssembler::getInt32(where);
#if DEBUG
		IonSpew(IonSpew_Codegen, "::Binding %p to %p\n",
			where, address);
#endif
		*where =  0x7FE00008; // PPC_trap
		JSC::PPCAssembler::patchBranch(where, address);
	        src = next;
	    } while (src != AbsoluteLabel::INVALID_OFFSET);
	}
	label->bind();
}

// pc <- [sp]; sp += n
void
Assembler::retn(Imm32 n) {
	ispew("retn(imm)");

	// This assumes the new PC is on top of the stack. It should be,
	// because we stored it there. 
	lwz(tempRegister, stackPointerRegister, 0);
	// New dispatch group on 970
	x_mtctr(tempRegister);
	JS_ASSERT(n.value < 32768);
	addi(stackPointerRegister, stackPointerRegister, n.value);
#if defined(_PPC970_)
	x_nop();
	x_nop();
	x_nop();
	// Force bctr into second dispatch group
#endif
	bctr();
}

void
Assembler::bind(Label *label) {
        JSC::MacroAssembler::Label jsclabel;

        // If this label was used, link everything previously now.
        if (label->used()) {
            bool more = false;
            JSC::PPCAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::PPCAssembler::JmpSrc next;
                BranchMap::Ptr p;

                // Put back the original branch so that it can be patched.
                p = branchwords_.lookup(jmp.m_offset);
		JS_ASSERT(p);
                more = masm.nextJump(jmp, &next, p->value);

                masm.linkPendedJump(jmp, masm.label());
                jmp = next;
            } while (more);
        }
        label->bind(masm.label().offset());
}

void
Assembler::bind(RepatchLabel *label) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            JSC::PPCAssembler::JmpSrc jmp(label->offset());
            masm.linkJump(jmp, masm.label());
        }
        label->bind(masm.label().offset());
}

void
Assembler::retarget(Label *label, Label *target) {
        JSC::MacroAssembler::Label jsclabel;
#if DEBUG
	IonSpew(IonSpew_Codegen,
		"::retarget label(%i) -> label(%i)",
			label->offset(), target->offset());
#endif
	
        if (label->used()) {
            bool more;
            JSC::PPCAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::PPCAssembler::JmpSrc next;
                BranchMap::Ptr p;

                // Put back the original branch so that it can be patched.
                p = branchwords_.lookup(jmp.m_offset);
		JS_ASSERT(p);
                more = masm.nextJump(jmp, &next, p->value);

                if (target->bound()) {
                    // The jump can be immediately patched to the correct
                    // destination.
                    masm.linkPendedJump(jmp, JmpDst(target->offset()));
                } else {
                    // Thread the jump list through the unpatched jump targets.
                    JmpSrc prev = JmpSrc(target->use(jmp.offset()));
		    // Save the previous branch word in the map.
		    JS_ASSERT(ensureBranchwordsReady());
                    branchwords_.put(jmp.m_offset,
			masm.setNextJump(jmp, prev));
                }

                jmp = next;
            } while (more);
        }
        label->reset();
}


/* AutoFlushCache implementation, mostly a carbon copy of ARM */
void
AutoFlushCache::update(uintptr_t newStart, size_t len)
{
    uintptr_t newStop = newStart + len;
    used_ = true;
    if (!start_) {
#if DEBUG
	IonSpewCont(IonSpew_CacheFlush,  ".");
#endif
	start_ = newStart;
	stop_ = newStop;
	return;
    }

    if (newStop < start_ - 4096 || newStart > stop_ + 4096) {
	// If this would add too many pages to the range, bail and
	// just do the flush now.
#if DEBUG
	IonSpewCont(IonSpew_CacheFlush, "*");
#endif
	JSC::ExecutableAllocator::cacheFlush((void*)newStart, len);
	return;
    }
    start_ = Min(start_, newStart);
    stop_ = Max(stop_, newStop);
    IonSpewCont(IonSpew_CacheFlush, ".");
}

AutoFlushCache::~AutoFlushCache()
{
   if (!runtime_)
	return;

    flushAnyway();
    IonSpewCont(IonSpew_CacheFlush, ">", name_);
    if (runtime_->flusher() == this) {
	IonSpewFin(IonSpew_CacheFlush);
	runtime_->setFlusher(NULL);
    }
}

void
AutoFlushCache::flushAnyway()
{
    if (!runtime_)
	return;

#if DEBUG
    IonSpewCont(IonSpew_CacheFlush, "|", name_);
#endif

    if (!used_)
	return;

    if (start_) {
	JSC::ExecutableAllocator::cacheFlush((void *)start_,
		size_t(stop_ - start_ + 4)); /* always word size instruction */
    } else {
	JS_NOT_REACHED("huh?");
	JSC::ExecutableAllocator::cacheFlush(NULL, 0xff000000);
    }
    used_ = false;
}

} // namespace jit

/* This is wrongobongo but it's to stub out AsmJS handling (we don't support
   the special profile yet). */

AsmJSMachExceptionHandler::AsmJSMachExceptionHandler() {
	installed_ = false;
}
void
AsmJSMachExceptionHandler::release() {
	/* nothin' */
}
bool
AsmJSMachExceptionHandler::install(JSRuntime *rt) {
	JS_NOT_REACHED("AsmJSMachExceptionHandler not implemented on PPC");
	return false;
}
void
AsmJSMachExceptionHandler::clearCurrentThread() {
	/* nothin' */
}
void
AsmJSMachExceptionHandler::setCurrentThread() {
	/* nothin' */
}

} // namespace js
