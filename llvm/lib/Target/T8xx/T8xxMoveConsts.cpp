//===-- T8xxStackifyInf.cpp - Arranges the operands on the operand stack ------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the pass which converts floating point instructions from
// pseudo registers into register stack instructions.  This pass uses live
// variable information to indicate where the FPn registers are used and their
// lifetimes.
//
// The x87 hardware tracks liveness of the stack registers, so it is necessary
// to implement exact liveness tracking between basic blocks. The CFG edges are
// partitioned into bundles where the same FP registers must be live in
// identical stack positions. Instructions are inserted at the end of each basic
// block to rearrange the live registers to match the outgoing bundle.
//
// This approach avoids splitting critical edges at the potential cost of more
// live register shuffling instructions when critical edges are present.
//
//===----------------------------------------------------------------------===//

#include "T8xx.h"
#include "T8xxInstrInfo.h"
#include "T8xxSubtarget.h"
#include "T8xxMachineFunctionInfo.h"
#include "T8xxDebugValueManager.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/EdgeBundles.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/Config/llvm-config.h"
#include "llvm/IR/InlineAsm.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include <algorithm>
#include <bitset>

using namespace llvm;

#define DEBUG_TYPE "t8xx-codegen"

namespace llvm {

  class T8xxMoveConstPass : public MachineFunctionPass {
    StringRef getPassName() const override { return "T8xx Const Arranger"; }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.setPreservesCFG();
      AU.addRequired<MachineDominatorTreeWrapperPass>();
      AU.addPreserved<MachineDominatorTreeWrapperPass>();

      AU.addRequired<VirtRegMapWrapperLegacy>();
      AU.addPreserved<VirtRegMapWrapperLegacy>();

      MachineFunctionPass::getAnalysisUsage(AU);
    }

    bool runOnMachineFunction(MachineFunction &MF) override;

    MachineFunctionProperties getRequiredProperties() const override {
      return MachineFunctionProperties().set(
	  MachineFunctionProperties::Property::NoPHIs);
    }

    MachineFunctionProperties getSetProperties() const override {
      return MachineFunctionProperties();
    }

  public:
    static char ID;
    T8xxMoveConstPass() : MachineFunctionPass(ID) {}
};

} // end anonymous namespace


using namespace llvm;

char T8xxMoveConstPass::ID = 0;

INITIALIZE_PASS_BEGIN(T8xxMoveConstPass, "t8xxstackifier", "T8xx Const Arranger",
                      false, false)
INITIALIZE_PASS_END(T8xxMoveConstPass, "t8xxstackifier", "T8xx Const Arranger",
                    false, false)

FunctionPass *llvm::createT8xxMoveConstPass() {
  return new T8xxMoveConstPass();
}

/// runOnMachineFunction - Loop over all of the basic blocks, transforming FP
/// register references into FP stack references.
///
bool T8xxMoveConstPass::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "********** Const (i.e. LDC) arranging  **********\n"
                       "********** Function: "
                    << MF.getName() << '\n');

  MachineRegisterInfo &MRI = MF.getRegInfo();
  T8xxMachineFunctionInfo &MFI = *MF.getInfo<T8xxMachineFunctionInfo>();
  const auto *TII = MF.getSubtarget<T8xxSubtarget>().getInstrInfo();
  const auto *TRI = MF.getSubtarget<T8xxSubtarget>().getRegisterInfo();

  // Walk the instructions from the bottom up. Currently we don't look past
  // block boundaries, and the blocks aren't ordered so the block visitation
  // order isn't significant, but we may want to change this in the future.
  for (MachineBasicBlock &MBB : MF) {

    // Don't use a range-based for loop, because we modify the list as we're
    // iterating over it and the end iterator may change.
    for (auto MII = MBB.begin(); MII != MBB.end(); ++MII) {
      if (MII->getOpcode() == T8xx::LDC)
	MII->dump ();
    }

  }

}
