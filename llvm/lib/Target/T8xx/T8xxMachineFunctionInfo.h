//===- T8xxMachineFunctionInfo.h - T8xx Machine Function Info -*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares  T8xx specific per-machine-function information.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_SPARC_SPARCMACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_SPARC_SPARCMACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

namespace llvm {

  class T8xxMachineFunctionInfo : public MachineFunctionInfo {
    virtual void anchor();
  private:
    Register GlobalBaseReg;

    /// VarArgsFrameOffset - Frame offset to start of varargs area.
    int VarArgsFrameOffset;

    /// SRetReturnReg - Holds the virtual register into which the sret
    /// argument is passed.
    Register SRetReturnReg;

    /// IsLeafProc - True if the function is a leaf procedure.
    bool IsLeafProc;

    /// A mapping from CodeGen vreg index to a boolean value indicating whether
    /// the given register is considered to be "stackified", meaning it has been
    /// determined or made to meet the stack requirements:
    ///   - single use (per path)
    ///   - single def (per path)
    ///   - defined and used in LIFO order with other stack registers
    BitVector VRegStackified;

  public:
    T8xxMachineFunctionInfo()
      : GlobalBaseReg(0), VarArgsFrameOffset(0), SRetReturnReg(0),
        IsLeafProc(false) {}
    explicit T8xxMachineFunctionInfo(MachineFunction &MF)
      : GlobalBaseReg(0), VarArgsFrameOffset(0), SRetReturnReg(0),
        IsLeafProc(false) {}

    MachineFunctionInfo *
    clone(BumpPtrAllocator &Allocator, MachineFunction &DestMF,
          const DenseMap<MachineBasicBlock *, MachineBasicBlock *> &Src2DstMBB)
        const override;

    Register getGlobalBaseReg() const { return GlobalBaseReg; }
    void setGlobalBaseReg(Register Reg) { GlobalBaseReg = Reg; }

    int getVarArgsFrameOffset() const { return VarArgsFrameOffset; }
    void setVarArgsFrameOffset(int Offset) { VarArgsFrameOffset = Offset; }

    Register getSRetReturnReg() const { return SRetReturnReg; }
    void setSRetReturnReg(Register Reg) { SRetReturnReg = Reg; }

    void setLeafProc(bool rhs) { IsLeafProc = rhs; }
    bool isLeafProc() const { return IsLeafProc; }

    // Copied from WebAssemblyMachineFunctionInfo.h
    void stackifyVReg(MachineRegisterInfo &MRI, unsigned VReg) {
      assert(MRI.getUniqueVRegDef(VReg));
      auto I = Register::virtReg2Index(VReg);
      if (I >= VRegStackified.size())
	VRegStackified.resize(I + 1);
      VRegStackified.set(I);
    }
    void unstackifyVReg(unsigned VReg) {
      auto I = Register::virtReg2Index(VReg);
      if (I < VRegStackified.size())
	VRegStackified.reset(I);
    }
    bool isVRegStackified(unsigned VReg) const {
      auto I = Register::virtReg2Index(VReg);
      if (I >= VRegStackified.size())
	return false;
      return VRegStackified.test(I);
    }


  };
}

#endif
