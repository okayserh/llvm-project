//===-- T8xxTargetInfo.cpp - T8xx Target Implementation -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/T8xxTargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
using namespace llvm;

Target &llvm::getTheT8xxTarget() {
  static Target TheT8xxTarget;
  return TheT8xxTarget;
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeT8xxTargetInfo() {
  RegisterTarget<Triple::t8xx, /*HasJIT=*/false> X(getTheT8xxTarget(),
                                                    "t8xx", "T8xx", "T8xx");
}
