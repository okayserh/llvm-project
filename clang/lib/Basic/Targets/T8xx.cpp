//===--- ARC.cpp - Implement ARC target feature support -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements ARC TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "T8xx.h"
#include "clang/Basic/Builtins.h"
#include "clang/Basic/MacroBuilder.h"
#include "clang/Basic/TargetBuiltins.h"

using namespace clang;
using namespace clang::targets;

void T8xxTargetInfo::getTargetDefines(const LangOptions &Opts,
                                     MacroBuilder &Builder) const {
  Builder.defineMacro("__t8xx__");
}


bool T8xxTargetInfo::validateAsmConstraint(const char *&Name,
					   TargetInfo::ConstraintInfo &Info) const {
  printf ("clang, validateAsmCons %s\n", Name);

  switch (*Name) {
  default:
    break;
  case 'c':
    Name++;
    return true;
  case 'b':
    Name++;
    return true;
  case 'a':
    Name++;
    return true;
  case 'F':
    Name += 4;
    return true;
  }
  
  return false;
}

