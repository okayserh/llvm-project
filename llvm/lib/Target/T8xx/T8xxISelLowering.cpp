//===-- T8xxISelLowering.cpp - T8xx DAG Lowering Implementation ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the interfaces that T8xx uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "T8xxISelLowering.h"
#include "MCTargetDesc/T8xxMCExpr.h"
#include "T8xxMachineFunctionInfo.h"
#include "T8xxRegisterInfo.h"
#include "T8xxTargetMachine.h"
#include "T8xxTargetObjectFile.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/KnownBits.h"
using namespace llvm;


const char *T8xxTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default:
    return NULL;
  case T8xxISD::RET_FLAG: return "RetFlag";
  case T8xxISD::LOAD_SYM: return "LOAD_SYM";
    /*	case LEGISD::MOVEi32:  return "MOVEi32";*/
  case T8xxISD::CALL:     return "CALL";
  }
}


T8xxTargetLowering::T8xxTargetLowering(const TargetMachine &TM,
                                         const T8xxSubtarget &STI)
    : TargetLowering(TM), Subtarget(&STI) {
  MVT PtrVT = MVT::getIntegerVT(TM.getPointerSizeInBits(0));

  // Set up the register classes.
  addRegisterClass(MVT::i32, &T8xx::IntRegsRegClass);

  computeRegisterProperties(Subtarget->getRegisterInfo());

  // Was used in LEG architecture. Unclear what it does ...
  //  setSchedulingPreference (Sched::Source);

  for (auto VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1, Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1, Promote);
    setLoadExtAction(ISD::EXTLOAD, VT, MVT::i1, Promote);
  }

  // We don't accept any truncstore of integer registers.
  //  setTruncStoreAction(MVT::i64, MVT::i32, Expand);
  // setTruncStoreAction(MVT::i64, MVT::i16, Expand);
  //setTruncStoreAction(MVT::i64, MVT::i8, Expand);
  // setTruncStoreAction(MVT::i32, MVT::i8, Expand);
  //  setTruncStoreAction(MVT::i16, MVT::i8, Expand);

  //  setOperationAction(ISD::INTRINSIC_WO_CHAIN, MVT::Other, Custom);

  setMinFunctionAlignment(Align(4));

  // Nodes that require custom lowering
  setOperationAction(ISD::GlobalAddress, MVT::i32, Custom);

  // T8xx doesn't have sext_inreg, replace them with shl/sra
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i16, Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8 , Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1 , Expand);
}

bool T8xxTargetLowering::useSoftFloat() const {
  return Subtarget->useSoftFloat();
}


SDValue T8xxTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch (Op.getOpcode()) {
  default:
    llvm_unreachable("Unimplemented operand");
  case ISD::GlobalAddress:
    printf ("####### Lower GlobalAddress  #########\n");
    return LowerGlobalAddress(Op, DAG);
  }
}

SDValue T8xxTargetLowering::LowerGlobalAddress(SDValue Op, SelectionDAG& DAG) const
{
  EVT VT = Op.getValueType();
  GlobalAddressSDNode *GlobalAddr = cast<GlobalAddressSDNode>(Op.getNode());
  SDValue TargetAddr =
      DAG.getTargetGlobalAddress(GlobalAddr->getGlobal(), Op, MVT::i32);
  return DAG.getNode(T8xxISD::LOAD_SYM, Op, VT, TargetAddr);
}



//===----------------------------------------------------------------------===//
// Calling Convention Implementation
//===----------------------------------------------------------------------===//

#include "T8xxGenCallingConv.inc"


//===----------------------------------------------------------------------===//
//                  Call Calling Convention Implementation
//===----------------------------------------------------------------------===//


// Lower a call
SDValue
T8xxTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                                  SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG = CLI.DAG;
  SDLoc &Loc = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins = CLI.Ins;
  SDValue Chain = CLI.Chain;
  SDValue Callee = CLI.Callee;
  CallingConv::ID CallConv = CLI.CallConv;
  const bool isVarArg = CLI.IsVarArg;

  CLI.IsTailCall = false;

  if (isVarArg) {
    llvm_unreachable("Unimplemented");
  }

  printf ("LowerCall\n");
  
  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());
  CCInfo.AnalyzeCallOperands(Outs, CC_T8xx32);

  // Get the size of the outgoing arguments stack space requirement.
  // Note: Named "ArgsSize" in SparcISelLowering
  const unsigned NumBytes = CCInfo.getNextStackOffset();

  /* Old LEG Code
  Chain =
    DAG.getCALLSEQ_START(Chain, DAG.getIntPtrConstant(NumBytes, Loc, true), 0,
                           Loc);
  */
  Chain = DAG.getCALLSEQ_START(Chain, NumBytes, 0, Loc);

  SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;

  printf ("ArgLocs %i\n", ArgLocs.size());

  printf ("Before ArgLocs\n");
  DAG.dump ();

  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    SDValue Arg = OutVals[i];

    // We only handle fully promoted arguments.
    assert(VA.getLocInfo() == CCValAssign::Full && "Unhandled loc info");

    if (VA.isRegLoc()) {
      printf ("VA %i is Reg\n", i);
      
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
      continue;
    }

    printf ("VA %i is Mem\n", i);
    
    assert(VA.isMemLoc() &&
           "Only support passing arguments through registers or via the stack");

    // TODO: Since the "registers" are actually on the stack, at this
    // point it is not feasible to adjust the framepointer.
    // Instead negative indices should be used for function parameters that should
    // be put on the stack

    SDValue StackPtr = DAG.getRegister(T8xx::WPTR, MVT::i32);
    SDValue PtrOff = DAG.getIntPtrConstant(-VA.getLocMemOffset(), Loc);
    PtrOff = DAG.getNode(ISD::ADD, Loc, MVT::i32, StackPtr, PtrOff);
    /* LEG Original
    MemOpChains.push_back(DAG.getStore(Chain, Loc, Arg, PtrOff,
                                       MachinePointerInfo(), false, false, 0));
    */
    /* Sparc version */
    MemOpChains.push_back(DAG.getStore(Chain, Loc, Arg, PtrOff,
                                       MachinePointerInfo()));
  }

  // Emit all stores, make sure they occur before the call.
  if (!MemOpChains.empty()) {
    Chain = DAG.getNode(ISD::TokenFactor, Loc, MVT::Other, MemOpChains);
  }

  printf ("Before RegsToPass\n");
  DAG.dump ();
  
  // Build a sequence of copy-to-reg nodes chained together with token chain
  // and flag operands which copy the outgoing args into the appropriate regs.
  SDValue InFlag;
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Chain = DAG.getCopyToReg(Chain, Loc, RegsToPass[i].first,
			     RegsToPass[i].second, InFlag);
    InFlag = Chain.getValue(1);
  }
  printf ("After RegsToPass\n");
  DAG.dump ();
  
  // We only support calling global addresses.
  /* Original code
  GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee);
  assert(G && "We only support the calling of global addresses");

  EVT PtrVT = getPointerTy(DAG.getDataLayout());
  Callee = DAG.getGlobalAddress(G->getGlobal(), Loc, PtrVT, 0);
  */

  // This works with a call instruction that directly takes
  // the address as parameter
  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee))
    Callee = DAG.getTargetGlobalAddress(G->getGlobal(), Loc, MVT::i32, 0);

  
  std::vector<SDValue> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // Add argument registers to the end of the list so that they are known live
  // into the call.
  for (auto &Reg : RegsToPass) {
    Ops.push_back(DAG.getRegister(Reg.first, Reg.second.getValueType()));
  }

  // Add a register mask operand representing the call-preserved registers.
  const uint32_t *Mask;
  const TargetRegisterInfo *TRI = DAG.getSubtarget().getRegisterInfo();
  Mask = TRI->getCallPreservedMask(DAG.getMachineFunction(), CallConv);

  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(DAG.getRegisterMask(Mask));

  if (InFlag.getNode()) {
    Ops.push_back(InFlag);
  }

  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  // Returns a chain and a flag for retval copy to use.
  Chain = DAG.getNode(T8xxISD::CALL, Loc, NodeTys, Ops);
  InFlag = Chain.getValue(1);

  Chain = DAG.getCALLSEQ_END(Chain, DAG.getIntPtrConstant(NumBytes, Loc, true),
                             DAG.getIntPtrConstant(0, Loc, true), InFlag, Loc);
  if (!Ins.empty()) {
    InFlag = Chain.getValue(1);
  }

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InFlag, CallConv, isVarArg, Ins, Loc, DAG,
                         InVals);
}


SDValue T8xxTargetLowering::LowerCallResult(
    SDValue Chain, SDValue InGlue, CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, SDLoc dl, SelectionDAG &DAG,
    SmallVectorImpl<SDValue> &InVals) const {
  assert(!isVarArg && "Unsupported");

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  CCInfo.AnalyzeCallResult(Ins, RetCC_T8xx32);

  // Copy all of the result registers out of their specified physreg.
  for (auto &Loc : RVLocs) {
    Chain = DAG.getCopyFromReg(Chain, dl, Loc.getLocReg(), Loc.getValVT(),
                               InGlue).getValue(1);
    InGlue = Chain.getValue(2);
    InVals.push_back(Chain.getValue(0));
  }

  return Chain;
}


//===----------------------------------------------------------------------===//
//             Formal Arguments Calling Convention Implementation
//===----------------------------------------------------------------------===//

SDValue T8xxTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {

  printf ("LowerFormalArguments\n");
  DAG.dump ();
  printf ("Pre LowerFormalArguments\n");
  
  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();


  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());
  CCInfo.AnalyzeFormalArguments(Ins, CC_T8xx32);

  int i = 0;
  
  for (auto &VA : ArgLocs) {
    if (VA.isRegLoc()) {
      printf ("VA %i is Reg\n", i++);
      // Arguments passed in registers
      EVT RegVT = VA.getLocVT();
      assert(RegVT.getSimpleVT().SimpleTy == MVT::i32 &&
             "Only support MVT::i32 register passing");

      const unsigned VReg = RegInfo.createVirtualRegister(&T8xx::IntRegsRegClass);

      RegInfo.addLiveIn(VA.getLocReg(), VReg);
      SDValue ArgIn = DAG.getCopyFromReg(Chain, DL, VReg, RegVT);

      InVals.push_back(ArgIn);
      continue;
    }

    printf ("VA %i is Mem\n", i++);
    
    assert(VA.isMemLoc() &&
           "Can only pass arguments as either registers or via the stack");

    const unsigned Offset = VA.getLocMemOffset();

    const int FI = MF.getFrameInfo().CreateFixedObject(4, Offset, true);
    EVT PtrTy = getPointerTy(DAG.getDataLayout());
    SDValue FIPtr = DAG.getFrameIndex(FI, PtrTy);

    assert(VA.getValVT() == MVT::i32 &&
           "Only support passing arguments as i32");
    SDValue Load = DAG.getLoad(VA.getValVT(), DL, Chain, FIPtr,
                               MachinePointerInfo());

    InVals.push_back(Load);
  }

  return Chain;
}


//===----------------------------------------------------------------------===//
//               Return Value Calling Convention Implementation
//===----------------------------------------------------------------------===//

bool T8xxTargetLowering::CanLowerReturn(
    CallingConv::ID CallConv, MachineFunction &MF, bool isVarArg,
    const SmallVectorImpl<ISD::OutputArg> &Outs, LLVMContext &Context) const {
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, MF, RVLocs, Context);
  if (!CCInfo.CheckReturn(Outs, RetCC_T8xx32)) {
    return false;
  }
  if (CCInfo.getNextStackOffset() != 0 && isVarArg) {
    return false;
  }
  return true;
}


SDValue
T8xxTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                                 bool IsVarArg,
                                 const SmallVectorImpl<ISD::OutputArg> &Outs,
                                 const SmallVectorImpl<SDValue> &OutVals,
                                 const SDLoc &DL, SelectionDAG &DAG) const {
  printf ("LowerFormalReturn\n");
  DAG.dump ();
  printf ("Pre LowerFormalReturn\n");
  
  // CCValAssign - represent the assignment of the return value to locations.
  SmallVector<CCValAssign, 16> RVLocs;

  // CCState - Info about the registers and stack slot.
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  // Analyze return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_T8xx32);

  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  printf ("Temp A, RVLocs Size %i\n", RVLocs.size());
  
  // Copy the result values into the output registers.
  for (unsigned i = 0, e = RVLocs.size(); i < e; ++i) {
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), OutVals[i], Flag);

    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  printf ("In Between 2\n");
  DAG.dump ();

  unsigned RetAddrOffset = 8; // Call Inst + Delay Slot

  RetOps[0] = Chain;  // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);

  printf ("Temp B  NumOps %i\n", RetOps.size());

  int i = 0;
  for (const auto &Op : RetOps)
    {      
      printf ("Op %i %p\n", i++, &Op);
      Op.dump ();
      printf ("OpCode %i\n", Op.getOpcode());
    }
      
  SDValue ret = DAG.getNode(T8xxISD::RET_FLAG, DL, MVT::Other, RetOps);

  printf ("Post Lower Return\n");
  DAG.dump ();

  printf ("Temp C\n");

  return ret;
}
