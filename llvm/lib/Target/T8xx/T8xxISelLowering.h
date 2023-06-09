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

#ifndef LLVM_LIB_TARGET_SPARC_SPARCISELLOWERING_H
#define LLVM_LIB_TARGET_SPARC_SPARCISELLOWERING_H

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
    LOAD_OP_STACK,
    ADD_WPTR
  };
  }

  class T8xxTargetLowering : public TargetLowering {
    const T8xxSubtarget *Subtarget;
  public:
    T8xxTargetLowering(const TargetMachine &TM, const T8xxSubtarget &STI);
    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

  /// getTargetNodeName - This method returns the name of a target specific
  //  DAG node.
  virtual const char *getTargetNodeName(unsigned Opcode) const override;

    bool useSoftFloat() const override;

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

    SDValue LowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;
  };
} // end namespace llvm

#endif    // SPARC_ISELLOWERING_H
