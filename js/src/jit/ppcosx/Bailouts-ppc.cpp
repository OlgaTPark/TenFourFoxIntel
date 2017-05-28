/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jscntxt.h"
#include "jscompartment.h"
#include "jit/Bailouts.h"
#include "jit/JitCompartment.h"

using namespace js;
using namespace js::jit;

#if(0)

/*   XXX??? */

#if 0
// no clue what these asserts should be.
JS_STATIC_ASSERT(sizeof(BailoutStack) ==
                 sizeof(uintptr_t) +
                 sizeof(double) * 8 +
                 sizeof(uintptr_t) * 8 +
                 sizeof(uintptr_t));

JS_STATIC_ASSERT(sizeof(ExtendedBailoutStack) ==
                 sizeof(BailoutStack) +
                 sizeof(uintptr_t));

#endif
#if 0
BailoutEnvironment::BailoutEnvironment(IonCompartment *ion, void **sp)
  : sp_(sp)
{
    bailout_ = reinterpret_cast<ExtendedBailoutStack *>(sp);

    if (bailout_->frameClass() != FrameSizeClass::None()) {
        frameSize_ = bailout_->frameSize();
        frame_ = &sp_[sizeof(BailoutStack) / STACK_SLOT_SIZE];

        // Compute the bailout ID.
        JitCode *code = ion->getBailoutTable(bailout_->frameClass());
        uintptr_t tableOffset = bailout_->tableOffset();
        uintptr_t tableStart = reinterpret_cast<uintptr_t>(code->raw());

        JS_ASSERT(tableOffset >= tableStart &&
                  tableOffset < tableStart + code->instructionsSize());
        JS_ASSERT((tableOffset - tableStart) % BAILOUT_TABLE_ENTRY_SIZE == 0);

        bailoutId_ = ((tableOffset - tableStart) / BAILOUT_TABLE_ENTRY_SIZE) - 1;
        JS_ASSERT(bailoutId_ < BAILOUT_TABLE_SIZE);
    } else {
        frameSize_ = bailout_->frameSize();
        frame_ = &sp_[sizeof(ExtendedBailoutStack) / STACK_SLOT_SIZE];
    }
}

IonFramePrefix *
BailoutEnvironment::top() const
{
    return (IonFramePrefix *)&frame_[frameSize_ / STACK_SLOT_SIZE];
}

#endif

/* XXX?? */
#endif

namespace js {
namespace jit {

class BailoutStack
{
    uintptr_t frameClassId_;
  public:
    // This is transferred in LR.
    union {
        uintptr_t frameSize_;
        uintptr_t tableOffset_;
    };

  private:
    // This is a total size of 29*8 + 27*4 = 340 = 0x154 bytes = 0x55 words.
    // Therefore, snapshotOffset_ is at 0x154 + 0x08 = 0x15c.
    // x/88w bailout_location_on_stack
    mozilla::Array<double, FloatRegisters::Total> fpregs_;
    mozilla::Array<uintptr_t, Registers::Total> regs_;

    uintptr_t snapshotOffset_;

/*
                frameClass   offset/frameSize
0xbfffda68:     0xffffffff      0x00000080      0xc24bc195      0x87859393
0xbfffda78:     0xc24bc195      0x87859393      0xc24bc195      0x87859393
0xbfffda88:     0xc24bc195      0x87859393      0xc24bc195      0x87859393
0xbfffda98:     0xc24bc195      0x87859393      0xc24bc195      0x87859393
0xbfffdaa8:     0xc24bc195      0x87859393      0xc24bc195      0x87859393
0xbfffdab8:     0xc24bc195      0x87859393      0xc24bc195      0x87859393
0xbfffdac8:     0xc24bc195      0x87859393      0xc24bc195      0x87859393
0xbfffdad8:     0xc24bc195      0x87859393      0xc24bc195      0x87859393
0xbfffdae8:     0xc17b7740      0x00000000      0x408f4000      0x00000000
0xbfffdaf8:     0xfff80000      0x00014eb0      0x40480000      0x00000000
0xbfffdb08:     0x43300000      0x0000d2f0      0xc24bc195      0x87859393
0xbfffdb18:     0xc24bc195      0x87859393      0x00000000      0x0003bf38
0xbfffdb28:     0x4085d9f6      0xe89b5ad0      0x4215b4c7      0x5eb80000
0xbfffdb38:     0x412e8480      0x00000000      0xc24bc195      0x87859393
0xbfffdb48:     0x3e601b69      0x0539fae9      0xffffff82      0x00000000
0xbfffdb58:     0xbfffe098      0xbfffdd44      0xbfffdd58      0x01d23720
  0x100
0xbfffdb68:     0x0204d200      0x007356e0      0x00000000      0x0000ffff
0xbfffdb78:     0xbfffdcd0      0xbfffdd60      0xbfffe5a4      0x01d08e60
0xbfffdb88:     0xbfffe610      0x00000001      0x00e7557c      0x02004c08
0xbfffdb98:     0xbfffdd60      0x00000000      0x00000000      0x02a34c80
0xbfffdba8:     0x00000000      0x00000000      0x00000000      0xffffff82
  0x150                                               
0xbfffdbb8:     0x00e38000      0xbfffdbc0      0x00000080      0x00000000
                                                              snapshotOffset
*/

  public:
    FrameSizeClass frameClass() const {
        return FrameSizeClass::FromClass(frameClassId_);
    }
    uintptr_t tableOffset() const {
        JS_ASSERT(frameClass() != FrameSizeClass::None());
        return tableOffset_;
    }
    uint32_t frameSize() const {
        if (frameClass() == FrameSizeClass::None())
            return frameSize_;
        return frameClass().frameSize();
    }
    MachineState machine() {
        return MachineState::FromBailout(regs_, fpregs_);
    }
    SnapshotOffset snapshotOffset() const {
        JS_ASSERT(frameClass() == FrameSizeClass::None());
        return snapshotOffset_;
    }
    uint8_t *parentStackPointer() const {
        if (frameClass() == FrameSizeClass::None())
            return (uint8_t *)this + sizeof(BailoutStack);
        return (uint8_t *)this + offsetof(BailoutStack, snapshotOffset_);
    }
};

IonBailoutIterator::IonBailoutIterator(const JitActivationIterator &activations,
                                       BailoutStack *bailout)
  : JitFrameIterator(activations),
    machine_(bailout->machine())
{
    uint8_t *sp = bailout->parentStackPointer();
    uint8_t *fp = sp + bailout->frameSize();

#if DEBUG
    IonSpew(IonSpew_Bailouts,
    "ppc-ibi bailout=%08x activations=%08x sp=%08x fp=%08x frameSize=%i",
        (uint32_t)bailout,
        (uint32_t)&activations,
        (uint32_t)sp, (uint32_t)fp,
        (int32_t)bailout->frameSize());
#endif

    current_ = fp;
    type_ = JitFrame_IonJS;
    topFrameSize_ = current_ - sp;
    topIonScript_ = script()->ionScript();

    if (bailout->frameClass() == FrameSizeClass::None()) {
        snapshotOffset_ = bailout->snapshotOffset();
#if DEBUG
        IonSpew(IonSpew_Bailouts,
            "ppc-ibi snapshotOffset=%i", snapshotOffset_);
#endif
        return;
    }

    // Compute the snapshot offset from the bailout ID.
    JitActivation *activation = activations.activation()->asJit();
    JSRuntime *rt = activation->compartment()->runtimeFromMainThread();
    JitCode *code = rt->jitRuntime()->getBailoutTable(bailout->frameClass());
    uintptr_t tableOffset = bailout->tableOffset();
    uintptr_t tableStart = reinterpret_cast<uintptr_t>(code->raw());

    JS_ASSERT(tableOffset >= tableStart &&
              tableOffset < tableStart + code->instructionsSize());
    JS_ASSERT((tableOffset - tableStart) % BAILOUT_TABLE_ENTRY_SIZE == 0);

    uint32_t bailoutId = ((tableOffset - tableStart) / BAILOUT_TABLE_ENTRY_SIZE) - 1;
    if (bailoutId < BAILOUT_TABLE_SIZE)
        snapshotOffset_ = topIonScript_->bailoutToSnapshot(bailoutId);
    else
        snapshotOffset_ = -1;
#if DEBUG
    IonSpew(IonSpew_Bailouts,
    "ppc-ibi bailoutId=%i tableStart=%08x tableOffset=%08x snapshotOffset=%i",
        bailoutId,
        (uint32_t)tableStart,
        (uint32_t)tableOffset,
        snapshotOffset_);
#endif
    JS_ASSERT(bailoutId < BAILOUT_TABLE_SIZE);

__asm__("trap\n");
}

IonBailoutIterator::IonBailoutIterator(const JitActivationIterator &activations,
                                       InvalidationBailoutStack *bailout)
  : JitFrameIterator(activations),
    machine_(bailout->machine())
{
    returnAddressToFp_ = bailout->osiPointReturnAddress();
    topIonScript_ = bailout->ionScript();
    const OsiIndex *osiIndex = topIonScript_->getOsiIndex(returnAddressToFp_);

    current_ = (uint8_t*) bailout->fp();
    type_ = JitFrame_IonJS;
    topFrameSize_ = current_ - bailout->sp();
    snapshotOffset_ = osiIndex->snapshotOffset();
}

} // namespace jit
} // namespace js

