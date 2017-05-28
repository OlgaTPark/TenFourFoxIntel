#include "BaselineCompiler-ppc.h"

using namespace js;
using namespace js::jit;

BaselineCompilerPPC::BaselineCompilerPPC(JSContext *cx, HandleScript script)
  : BaselineCompilerShared(cx, script)
{
}

