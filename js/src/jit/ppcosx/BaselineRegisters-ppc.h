#if !defined(jsion_baseline_registers_ppc_h__) && defined(JS_ION)
#define jsion_baseline_registers_ppc_h__

#include "jit/IonMacroAssembler.h"

namespace js {
namespace jit {

// r13 has a copy of the stack pointer for Baseline (see Trampoline). It
// is non-allocatable and non-volatile.
static const Register BaselineFrameReg = r13;
static const Register BaselineStackReg = r1;

// Use addressTempRegister for Baseline-specific stuff.
static const Register ScratchRegister = r12;

// ValueOperands R0, R1, and R2.
// R0 == JSReturnReg, and R2 uses registers not
// preserved across calls.  R1 value should be
// preserved across calls.
static const ValueOperand R0(r6, r5);
static const ValueOperand R1(r15, r14); // Saved by trampoline
static const ValueOperand R2(r4, r3);

// BaselineTailCallReg and BaselineStubReg
// These use registers that are not preserved across
// calls.
static const Register BaselineTailCallReg = r8;
static const Register BaselineStubReg     = r7;

static const Register ExtractTemp0        = InvalidReg;
static const Register ExtractTemp1        = InvalidReg;

// FloatReg0 must be equal to ReturnFloatReg.
static const FloatRegister FloatReg0      = f1;
static const FloatRegister FloatReg1      = { JSC::PPCRegisters::f2 };

} // namespace jit
} // namespace js

#endif
