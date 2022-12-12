//===-- T8xxMachineFunctionInfo.cpp - T8xx Machine Function Info --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "T8xxMachineFunctionInfo.h"

using namespace llvm;

void T8xxMachineFunctionInfo::anchor() { }

MachineFunctionInfo *T8xxMachineFunctionInfo::clone(
    BumpPtrAllocator &Allocator, MachineFunction &DestMF,
    const DenseMap<MachineBasicBlock *, MachineBasicBlock *> &Src2DstMBB)
    const {
  return DestMF.cloneInfo<T8xxMachineFunctionInfo>(*this);
}
