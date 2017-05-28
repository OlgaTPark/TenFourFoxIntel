#ifndef jsion_baselinecompiler_ppc_h__
#define jsion_baselinecompiler_ppc_h__

#include "jit/shared/BaselineCompiler-shared.h"

namespace js {
namespace jit {

class BaselineCompilerPPC : public BaselineCompilerShared
{
  protected:
    BaselineCompilerPPC(JSContext *cx, HandleScript script);
};

typedef BaselineCompilerPPC BaselineCompilerSpecific;

} // namespace jit
} // namespace js

#endif // jsion_baselinecompiler_ppc_h__

