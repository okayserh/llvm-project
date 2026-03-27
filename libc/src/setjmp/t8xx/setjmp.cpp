//===-- Implementation of setjmp ------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "src/__support/common.h"
#include "src/__support/macros/config.h"
#include "src/setjmp/setjmp_impl.h"

namespace LIBC_NAMESPACE_DECL {

  LLVM_LIBC_FUNCTION(int, setjmp, (jmp_buf /*buf*/)) {

  return 0;
}

} // namespace LIBC_NAMESPACE_DECL
