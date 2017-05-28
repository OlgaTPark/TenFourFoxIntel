#include "BaselineCompiler-ppc.h"

using namespace js;
using namespace js::jit;

BaselineCompilerPPC::BaselineCompilerPPC(JSContext *cx, TempAllocator &alloc, JSScript *script)
  : BaselineCompilerShared(cx, alloc, script)
{
}

