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
#ifndef LLVM_LIB_TARGET_T8XX_T8XXMACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_T8XX_T8XXMACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

namespace llvm {

  class T8xxMachineFunctionInfo : public MachineFunctionInfo {
    virtual void anchor();
  private:
    Register GlobalBaseReg;

    // Taken from RISCV
    /// FrameIndex for start of varargs area
    int VarArgsFrameIndex = 0;
    /// Size of the save area used for varargs
    int VarArgsSaveSize = 0;
    
    /// IsLeafProc - True if the function is a leaf procedure.
    bool IsLeafProc;

    // A stack slot which is used to conserve the WPtr when function was entered
    int WPtrStackSlot;

    // A stack slot which is used as temporary storage when a function needs to
    // use "move" instructions to copy/align shorter ints or unaligned words.
    int MoveStackSlot = 0;

    // A stack slot which is used as temporary storage when a function needs to
    // store a double precision floating point number for bit manipulation.
    int DoubleFPStackSlot = 0;

  public:
    T8xxMachineFunctionInfo() = default;
    T8xxMachineFunctionInfo(const Function &F, const TargetSubtargetInfo *STI);

    MachineFunctionInfo *
    clone(BumpPtrAllocator &Allocator, MachineFunction &DestMF,
          const DenseMap<MachineBasicBlock *, MachineBasicBlock *> &Src2DstMBB)
        const override;

    Register getGlobalBaseReg() const { return GlobalBaseReg; }
    void setGlobalBaseReg(Register Reg) { GlobalBaseReg = Reg; }

    // Taken from RISCV
    int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
    void setVarArgsFrameIndex(int Index) { VarArgsFrameIndex = Index; }

    unsigned getVarArgsSaveSize() const { return VarArgsSaveSize; }
    void setVarArgsSaveSize(int Size) { VarArgsSaveSize = Size; }
    
    void setLeafProc(bool rhs) { IsLeafProc = rhs; }
    bool isLeafProc() const { return IsLeafProc; }

    // Access to Stack slot for WPtr
    void setWPtrSlot(int slot) { WPtrStackSlot = slot; }
    int getWPtrSlot() const { return WPtrStackSlot; }

    // Access to Stack slot for MOVE
    void setMoveSlot(int slot) { MoveStackSlot = slot; }
    int getMoveSlot() const { return MoveStackSlot; }

    // Access to Stack slot for double precision FP manipulations
    void setDoubleFPSlot(int slot) { DoubleFPStackSlot = slot; }
    int getDoubleFPSlot() const { return DoubleFPStackSlot; }
  };
}

#endif
