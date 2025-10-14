//===-- T8xxISelLowering.h - T8xx DAG Lowering Interface ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that T8xx uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_T8XX_T8XXISELLOWERING_H
#define LLVM_LIB_TARGET_T8XX_T8XXISELLOWERING_H

#include "T8xx.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {
  class T8xxSubtarget;

  namespace T8xxISD {
  enum NodeType : unsigned {
    FIRST_NUMBER = ISD::BUILTIN_OP_END,

    CALL,            // A call instruction.
    RET_FLAG,        // Return with a flag operand.
    LOAD_SYM,
    ADD_WPTR,
    ADD_IPTR,
    STL_PARM,

    // T8xx conditional moves.
    CMOV,
    BRNCOND,
    LDIFF,
    REV,
    JOIN,
    FP_SETCC
  };
  }

  class T8xxTargetLowering : public TargetLowering {
    const T8xxSubtarget *Subtarget;
  public:
    T8xxTargetLowering(const TargetMachine &TM, const T8xxSubtarget &STI);

    /// getTargetNodeName - This method returns the name of a target specific
    //  DAG node.
    const char *getTargetNodeName(unsigned Opcode) const override;

    /// ReplaceNodeResults - Replace the results of node with an illegal result
    /// type with new values built out of custom code.
    ///
    void ReplaceNodeResults(SDNode *N, SmallVectorImpl<SDValue>&Results,
                            SelectionDAG &DAG) const override;

    // TODO: Check if there are cases, where this might not be true
    virtual bool isIntDivCheap(EVT VT, AttributeList Attr) const override { return true; }
    
    bool useSoftFloat() const override;

    /// Provide custom lowering hooks for some operations
    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

    MachineBasicBlock *
    EmitInstrWithCustomInserter(MachineInstr &MI,
				MachineBasicBlock *MBB) const override;

    // TODO: Quick fix to make all jump tables entries "global".
    unsigned getJumpTableEncoding() const override;

    bool isJumpTableRelative() const override;
    
  private:
    SDValue
    LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
                         const SmallVectorImpl<ISD::InputArg> &Ins,
                         const SDLoc &dl, SelectionDAG &DAG,
                         SmallVectorImpl<SDValue> &InVals) const override;

    SDValue
      LowerCall(TargetLowering::CallLoweringInfo &CLI,
                SmallVectorImpl<SDValue> &InVals) const override;

    SDValue LowerReturn(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
                        const SmallVectorImpl<ISD::OutputArg> &Outs,
                        const SmallVectorImpl<SDValue> &OutVals,
                        const SDLoc &dl, SelectionDAG &DAG) const override;

    SDValue LowerCallResult(SDValue Chain, SDValue InGlue,
                          CallingConv::ID CallConv, bool isVarArg,
                          const SmallVectorImpl<ISD::InputArg> &Ins, SDLoc dl,
                          SelectionDAG &DAG,
                          SmallVectorImpl<SDValue> &InVals) const;

    bool CanLowerReturn(CallingConv::ID CallConv, MachineFunction &MF,
                        bool isVarArg,
                        const SmallVectorImpl<ISD::OutputArg> &Outs,
                        LLVMContext &Context) const override;

    //    SDValue LowerFMUL(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerSTORE(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerBRCOND(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerVASTART(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerSETCC(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerSELECT(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerConstantPool(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerJumpTable(SDValue Op, SelectionDAG& DAG) const;
    SDValue LowerBlockAddress(SDValue Op, SelectionDAG& DAG) const;

    MachineBasicBlock *EmitLoweredSelect(MachineInstr &I,
					 MachineBasicBlock *MBB) const;
    
    MachineBasicBlock *EmitLoweredFPSetCC(MachineInstr &I,
					 MachineBasicBlock *MBB) const;
    
    // Inline assembly
    std::pair<unsigned, const TargetRegisterClass *>
    getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
				 StringRef Constraint,
				 MVT VT) const override;

  };
} // end namespace llvm

#endif    // SPARC_ISELLOWERING_H
