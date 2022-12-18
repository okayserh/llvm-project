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


//===----------------------------------------------------------------------===//
// Calling Convention Implementation
//===----------------------------------------------------------------------===//

static bool CC_T8xx_Assign_SRet(unsigned &ValNo, MVT &ValVT,
                                 MVT &LocVT, CCValAssign::LocInfo &LocInfo,
                                 ISD::ArgFlagsTy &ArgFlags, CCState &State)
{
  assert (ArgFlags.isSRet());

  // Assign SRet argument.
  State.addLoc(CCValAssign::getCustomMem(ValNo, ValVT,
                                         0,
                                         LocVT, LocInfo));
  return true;
}


#include "T8xxGenCallingConv.inc"

// The calling conventions in T8xxCallingConv.td are described in terms of the
// callee's register window. This function translates registers to the
// corresponding caller window %o register.
static unsigned toCallerWindow(unsigned Reg) {
  static_assert(T8::R0 + 7 == T8::R7 && T8::R0 + 7 == T8::R7,
                "Unexpected enum");
  if (Reg >= T8::R0 && Reg <= T8::R7)
    return Reg - T8::R0 + T8::R0;  // TODO: Nonsense
  return Reg;
}

bool T8xxTargetLowering::CanLowerReturn(
    CallingConv::ID CallConv, MachineFunction &MF, bool isVarArg,
    const SmallVectorImpl<ISD::OutputArg> &Outs, LLVMContext &Context) const {
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, MF, RVLocs, Context);
  return CCInfo.CheckReturn(Outs, RetCC_T8xx32);
}

SDValue
T8xxTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                                 bool IsVarArg,
                                 const SmallVectorImpl<ISD::OutputArg> &Outs,
                                 const SmallVectorImpl<SDValue> &OutVals,
                                 const SDLoc &DL, SelectionDAG &DAG) const {
  return LowerReturn_32(Chain, CallConv, IsVarArg, Outs, OutVals, DL, DAG);
}

SDValue
T8xxTargetLowering::LowerReturn_32(SDValue Chain, CallingConv::ID CallConv,
                                    bool IsVarArg,
                                    const SmallVectorImpl<ISD::OutputArg> &Outs,
                                    const SmallVectorImpl<SDValue> &OutVals,
                                    const SDLoc &DL, SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();

  // CCValAssign - represent the assignment of the return value to locations.
  SmallVector<CCValAssign, 16> RVLocs;

  // CCState - Info about the registers and stack slot.
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  // Analyze return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_T8xx32);

  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);
  // Make room for the return address offset.
  RetOps.push_back(SDValue());

  // Copy the result values into the output registers.
  for (unsigned i = 0, realRVLocIdx = 0;
       i != RVLocs.size();
       ++i, ++realRVLocIdx) {
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), OutVals[i], Flag);

    // Guarantee that all emitted copies are stuck together with flags.
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  RetOps[0] = Chain;  // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);

  return DAG.getNode(SPISD::RET_FLAG, DL, MVT::Other, RetOps);
}


SDValue T8xxTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {

  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();

  for (const ISD::InputArg &In : Ins) {
    // Ignore In.getOrigAlign() because all our arguments are passed in
    // registers.
    /* TODO
    InVals.push_back(
        In.Used
            ? DAG.getNode(SPISD::ARGUMENT, DL, In.VT,
                          DAG.getTargetConstant(InVals.size(), DL, MVT::i32))
            : DAG.getUNDEF(In.VT));
    */

    // Record the number and types of arguments.
    //    MF.getInfo<LEGFunctionInfo>()->addParam(In.VT);
  }

  return Chain;
}


SDValue
T8xxTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                               SmallVectorImpl<SDValue> &InVals) const {
  return LowerCall_32(CLI, InVals);
}

static bool hasReturnsTwiceAttr(SelectionDAG &DAG, SDValue Callee,
                                const CallBase *Call) {
  if (Call)
    return Call->hasFnAttr(Attribute::ReturnsTwice);

  const Function *CalleeFn = nullptr;
  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
    CalleeFn = dyn_cast<Function>(G->getGlobal());
  } else if (ExternalSymbolSDNode *E =
             dyn_cast<ExternalSymbolSDNode>(Callee)) {
    const Function &Fn = DAG.getMachineFunction().getFunction();
    const Module *M = Fn.getParent();
    const char *CalleeName = E->getSymbol();
    CalleeFn = M->getFunction(CalleeName);
  }

  if (!CalleeFn)
    return false;
  return CalleeFn->hasFnAttribute(Attribute::ReturnsTwice);
}

/// IsEligibleForTailCallOptimization - Check whether the call is eligible
/// for tail call optimization.
bool T8xxTargetLowering::IsEligibleForTailCallOptimization(
    CCState &CCInfo, CallLoweringInfo &CLI, MachineFunction &MF) const {

  auto &Outs = CLI.Outs;
  auto &Caller = MF.getFunction();

  // Do not tail call opt functions with "disable-tail-calls" attribute.
  if (Caller.getFnAttribute("disable-tail-calls").getValueAsString() == "true")
    return false;

  // Do not tail call opt if the stack is used to pass parameters.
  // 64-bit targets have a slightly higher limit since the ABI requires
  // to allocate some space even when all the parameters fit inside registers.
  unsigned StackOffsetLimit = 0;
  if (CCInfo.getNextStackOffset() > StackOffsetLimit)
    return false;

  // Do not tail call opt if either the callee or caller returns
  // a struct and the other does not.
  if (!Outs.empty() && Caller.hasStructRetAttr() != Outs[0].Flags.isSRet())
    return false;

  // Byval parameters hand the function a pointer directly into the stack area
  // we want to reuse during a tail call.
  for (auto &Arg : Outs)
    if (Arg.Flags.isByVal())
      return false;

  return true;
}

// Lower a call for the 32-bit ABI.
SDValue
T8xxTargetLowering::LowerCall_32(TargetLowering::CallLoweringInfo &CLI,
                                  SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG                     = CLI.DAG;
  SDLoc &dl                             = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals     = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins   = CLI.Ins;
  SDValue Chain                         = CLI.Chain;
  SDValue Callee                        = CLI.Callee;
  bool &isTailCall                      = CLI.IsTailCall;
  CallingConv::ID CallConv              = CLI.CallConv;
  bool isVarArg                         = CLI.IsVarArg;

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());
  CCInfo.AnalyzeCallOperands(Outs, CC_T8xx32);

  isTailCall = isTailCall && IsEligibleForTailCallOptimization(
                                 CCInfo, CLI, DAG.getMachineFunction());

  // Get the size of the outgoing arguments stack space requirement.
  unsigned ArgsSize = CCInfo.getNextStackOffset();

  // Keep stack frames 8-byte aligned.
  ArgsSize = (ArgsSize+7) & ~7;

  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();

  // Create local copies for byval args.
  SmallVector<SDValue, 8> ByValArgs;
  for (unsigned i = 0,  e = Outs.size(); i != e; ++i) {
    ISD::ArgFlagsTy Flags = Outs[i].Flags;
    if (!Flags.isByVal())
      continue;

    SDValue Arg = OutVals[i];
    unsigned Size = Flags.getByValSize();
    Align Alignment = Flags.getNonZeroByValAlign();

    if (Size > 0U) {
      int FI = MFI.CreateStackObject(Size, Alignment, false);
      SDValue FIPtr = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
      SDValue SizeNode = DAG.getConstant(Size, dl, MVT::i32);

      Chain = DAG.getMemcpy(Chain, dl, FIPtr, Arg, SizeNode, Alignment,
                            false,        // isVolatile,
                            (Size <= 32), // AlwaysInline if size <= 32,
                            false,        // isTailCall
                            MachinePointerInfo(), MachinePointerInfo());
      ByValArgs.push_back(FIPtr);
    }
    else {
      SDValue nullVal;
      ByValArgs.push_back(nullVal);
    }
  }

  assert(!isTailCall || ArgsSize == 0);

  if (!isTailCall)
    Chain = DAG.getCALLSEQ_START(Chain, ArgsSize, 0, dl);

  SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;

  const unsigned StackOffset = 92;
  bool hasStructRetAttr = false;
  unsigned SRetArgSize = 0;
  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, realArgIdx = 0, byvalArgIdx = 0, e = ArgLocs.size();
       i != e;
       ++i, ++realArgIdx) {
    CCValAssign &VA = ArgLocs[i];
    SDValue Arg = OutVals[realArgIdx];

    ISD::ArgFlagsTy Flags = Outs[realArgIdx].Flags;

    // Use local copy if it is a byval arg.
    if (Flags.isByVal()) {
      Arg = ByValArgs[byvalArgIdx++];
      if (!Arg) {
        continue;
      }
    }

    // Promote the value if needed.
    switch (VA.getLocInfo()) {
    default: llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full: break;
    case CCValAssign::SExt:
      Arg = DAG.getNode(ISD::SIGN_EXTEND, dl, VA.getLocVT(), Arg);
      break;
    case CCValAssign::ZExt:
      Arg = DAG.getNode(ISD::ZERO_EXTEND, dl, VA.getLocVT(), Arg);
      break;
    case CCValAssign::AExt:
      Arg = DAG.getNode(ISD::ANY_EXTEND, dl, VA.getLocVT(), Arg);
      break;
    case CCValAssign::BCvt:
      Arg = DAG.getNode(ISD::BITCAST, dl, VA.getLocVT(), Arg);
      break;
    }

    if (Flags.isSRet()) {
      assert(VA.needsCustom());

      if (isTailCall)
        continue;

      // store SRet argument in %sp+64
      SDValue StackPtr = DAG.getRegister(T8::R6, MVT::i32);
      SDValue PtrOff = DAG.getIntPtrConstant(64, dl);
      PtrOff = DAG.getNode(ISD::ADD, dl, MVT::i32, StackPtr, PtrOff);
      MemOpChains.push_back(
          DAG.getStore(Chain, dl, Arg, PtrOff, MachinePointerInfo()));
      hasStructRetAttr = true;
      // sret only allowed on first argument
      assert(Outs[realArgIdx].OrigArgIndex == 0);
      SRetArgSize =
          DAG.getDataLayout().getTypeAllocSize(CLI.getArgs()[0].IndirectType);
      continue;
    }

    if (VA.needsCustom()) {
      assert(VA.getLocVT() == MVT::f64 || VA.getLocVT() == MVT::v2i32);

      if (VA.isMemLoc()) {
        unsigned Offset = VA.getLocMemOffset() + StackOffset;
        // if it is double-word aligned, just store.
        if (Offset % 8 == 0) {
          SDValue StackPtr = DAG.getRegister(T8::R6, MVT::i32);
          SDValue PtrOff = DAG.getIntPtrConstant(Offset, dl);
          PtrOff = DAG.getNode(ISD::ADD, dl, MVT::i32, StackPtr, PtrOff);
          MemOpChains.push_back(
              DAG.getStore(Chain, dl, Arg, PtrOff, MachinePointerInfo()));
          continue;
        }
      }

      /*
      if (VA.getLocVT() == MVT::f64) {
        // Move from the float value from float registers into the
        // integer registers.
        if (ConstantFPSDNode *C = dyn_cast<ConstantFPSDNode>(Arg))
          Arg = bitcastConstantFPToInt(C, dl, DAG);
        else
          Arg = DAG.getNode(ISD::BITCAST, dl, MVT::v2i32, Arg);
      }
      */
      
      SDValue Part0 = DAG.getNode(ISD::EXTRACT_VECTOR_ELT, dl, MVT::i32,
                                  Arg,
                                  DAG.getConstant(0, dl, getVectorIdxTy(DAG.getDataLayout())));
      SDValue Part1 = DAG.getNode(ISD::EXTRACT_VECTOR_ELT, dl, MVT::i32,
                                  Arg,
                                  DAG.getConstant(1, dl, getVectorIdxTy(DAG.getDataLayout())));

      if (VA.isRegLoc()) {
        RegsToPass.push_back(std::make_pair(VA.getLocReg(), Part0));
        assert(i+1 != e);
        CCValAssign &NextVA = ArgLocs[++i];
        if (NextVA.isRegLoc()) {
          RegsToPass.push_back(std::make_pair(NextVA.getLocReg(), Part1));
        } else {
          // Store the second part in stack.
          unsigned Offset = NextVA.getLocMemOffset() + StackOffset;
          SDValue StackPtr = DAG.getRegister(T8::R6, MVT::i32);
          SDValue PtrOff = DAG.getIntPtrConstant(Offset, dl);
          PtrOff = DAG.getNode(ISD::ADD, dl, MVT::i32, StackPtr, PtrOff);
          MemOpChains.push_back(
              DAG.getStore(Chain, dl, Part1, PtrOff, MachinePointerInfo()));
        }
      } else {
        unsigned Offset = VA.getLocMemOffset() + StackOffset;
        // Store the first part.
        SDValue StackPtr = DAG.getRegister(T8::R6, MVT::i32);
        SDValue PtrOff = DAG.getIntPtrConstant(Offset, dl);
        PtrOff = DAG.getNode(ISD::ADD, dl, MVT::i32, StackPtr, PtrOff);
        MemOpChains.push_back(
            DAG.getStore(Chain, dl, Part0, PtrOff, MachinePointerInfo()));
        // Store the second part.
        PtrOff = DAG.getIntPtrConstant(Offset + 4, dl);
        PtrOff = DAG.getNode(ISD::ADD, dl, MVT::i32, StackPtr, PtrOff);
        MemOpChains.push_back(
            DAG.getStore(Chain, dl, Part1, PtrOff, MachinePointerInfo()));
      }
      continue;
    }

    // Arguments that can be passed on register must be kept at
    // RegsToPass vector
    if (VA.isRegLoc()) {
      if (VA.getLocVT() != MVT::f32) {
        RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
        continue;
      }
      Arg = DAG.getNode(ISD::BITCAST, dl, MVT::i32, Arg);
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
      continue;
    }

    assert(VA.isMemLoc());

    // Create a store off the stack pointer for this argument.
    SDValue StackPtr = DAG.getRegister(T8::R6, MVT::i32);
    SDValue PtrOff = DAG.getIntPtrConstant(VA.getLocMemOffset() + StackOffset,
                                           dl);
    PtrOff = DAG.getNode(ISD::ADD, dl, MVT::i32, StackPtr, PtrOff);
    MemOpChains.push_back(
        DAG.getStore(Chain, dl, Arg, PtrOff, MachinePointerInfo()));
  }


  // Emit all stores, make sure the occur before any copies into physregs.
  if (!MemOpChains.empty())
    Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other, MemOpChains);

  // Build a sequence of copy-to-reg nodes chained together with token
  // chain and flag operands which copy the outgoing args into registers.
  // The InFlag in necessary since all emitted instructions must be
  // stuck together.
  SDValue InFlag;
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Register Reg = RegsToPass[i].first;
    if (!isTailCall)
      Reg = toCallerWindow(Reg);
    Chain = DAG.getCopyToReg(Chain, dl, Reg, RegsToPass[i].second, InFlag);
    InFlag = Chain.getValue(1);
  }

  bool hasReturnsTwice = hasReturnsTwiceAttr(DAG, Callee, CLI.CB);

  // If the callee is a GlobalAddress node (quite common, every direct call is)
  // turn it into a TargetGlobalAddress node so that legalize doesn't hack it.
  // Likewise ExternalSymbol -> TargetExternalSymbol.
  unsigned TF = isPositionIndependent() ? T8xxMCExpr::VK_T8xx_WPLT30
                                        : T8xxMCExpr::VK_T8xx_WDISP30;
  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee))
    Callee = DAG.getTargetGlobalAddress(G->getGlobal(), dl, MVT::i32, 0, TF);
  else if (ExternalSymbolSDNode *E = dyn_cast<ExternalSymbolSDNode>(Callee))
    Callee = DAG.getTargetExternalSymbol(E->getSymbol(), MVT::i32, TF);

  // Returns a chain & a flag for retval copy to use
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);
  if (hasStructRetAttr)
    Ops.push_back(DAG.getTargetConstant(SRetArgSize, dl, MVT::i32));
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Register Reg = RegsToPass[i].first;
    if (!isTailCall)
      Reg = toCallerWindow(Reg);
    Ops.push_back(DAG.getRegister(Reg, RegsToPass[i].second.getValueType()));
  }

  // Add a register mask operand representing the call-preserved registers.
  const T8xxRegisterInfo *TRI = Subtarget->getRegisterInfo();
  const uint32_t *Mask =
      ((hasReturnsTwice)
           ? TRI->getRTCallPreservedMask(CallConv)
           : TRI->getCallPreservedMask(DAG.getMachineFunction(), CallConv));
  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(DAG.getRegisterMask(Mask));

  if (InFlag.getNode())
    Ops.push_back(InFlag);

  if (isTailCall) {
    DAG.getMachineFunction().getFrameInfo().setHasTailCall();
    return DAG.getNode(SPISD::TAIL_CALL, dl, MVT::Other, Ops);
  }

  Chain = DAG.getNode(SPISD::CALL, dl, NodeTys, Ops);
  InFlag = Chain.getValue(1);

  Chain = DAG.getCALLSEQ_END(Chain, ArgsSize, 0, InFlag, dl);
  InFlag = Chain.getValue(1);

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState RVInfo(CallConv, isVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  RVInfo.AnalyzeCallResult(Ins, RetCC_T8xx32);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    assert(RVLocs[i].isRegLoc() && "Can only return in registers!");
    if (RVLocs[i].getLocVT() == MVT::v2i32) {
      SDValue Vec = DAG.getNode(ISD::UNDEF, dl, MVT::v2i32);
      SDValue Lo = DAG.getCopyFromReg(
          Chain, dl, toCallerWindow(RVLocs[i++].getLocReg()), MVT::i32, InFlag);
      Chain = Lo.getValue(1);
      InFlag = Lo.getValue(2);
      Vec = DAG.getNode(ISD::INSERT_VECTOR_ELT, dl, MVT::v2i32, Vec, Lo,
                        DAG.getConstant(0, dl, MVT::i32));
      SDValue Hi = DAG.getCopyFromReg(
          Chain, dl, toCallerWindow(RVLocs[i].getLocReg()), MVT::i32, InFlag);
      Chain = Hi.getValue(1);
      InFlag = Hi.getValue(2);
      Vec = DAG.getNode(ISD::INSERT_VECTOR_ELT, dl, MVT::v2i32, Vec, Hi,
                        DAG.getConstant(1, dl, MVT::i32));
      InVals.push_back(Vec);
    } else {
      Chain =
          DAG.getCopyFromReg(Chain, dl, toCallerWindow(RVLocs[i].getLocReg()),
                             RVLocs[i].getValVT(), InFlag)
              .getValue(1);
      InFlag = Chain.getValue(2);
      InVals.push_back(Chain.getValue(0));
    }
  }

  return Chain;
}

// FIXME? Maybe this could be a TableGen attribute on some registers and
// this table could be generated automatically from RegInfo.
Register T8xxTargetLowering::getRegisterByName(const char* RegName, LLT VT,
                                                const MachineFunction &MF) const {
  Register Reg = StringSwitch<Register>(RegName)
    .Case("R0", T8::R0).Case("R1", T8::R1).Case("R2", T8::R2).Case("R3", T8::R3)
    .Case("R4", T8::R4).Case("R5", T8::R5).Case("R6", T8::R6).Case("R7", T8::R7)
    .Case("R8", T8::R0).Case("R9", T8::R1).Case("R10", T8::R2).Case("R11", T8::R3)
    .Case("R12", T8::R4).Case("R13", T8::R5).Case("R14", T8::R6).Case("R15", T8::R7)
    .Default(0);

  if (Reg)
    return Reg;

  report_fatal_error("Invalid register name global variable");
}


//===----------------------------------------------------------------------===//
// TargetLowering Implementation
//===----------------------------------------------------------------------===//

TargetLowering::AtomicExpansionKind T8xxTargetLowering::shouldExpandAtomicRMWInIR(AtomicRMWInst *AI) const {
  if (AI->getOperation() == AtomicRMWInst::Xchg &&
      AI->getType()->getPrimitiveSizeInBits() == 32)
    return AtomicExpansionKind::None; // Uses xchg instruction

  return AtomicExpansionKind::CmpXChg;
}


T8xxTargetLowering::T8xxTargetLowering(const TargetMachine &TM,
                                         const T8xxSubtarget &STI)
    : TargetLowering(TM), Subtarget(&STI) {
  MVT PtrVT = MVT::getIntegerVT(TM.getPointerSizeInBits(0));

  // Instructions which use registers as conditionals examine all the
  // bits (as does the pseudo SELECT_CC expansion). I don't think it
  // matters much whether it's ZeroOrOneBooleanContent, or
  // ZeroOrNegativeOneBooleanContent, so, arbitrarily choose the
  // former.
  setBooleanContents(ZeroOrOneBooleanContent);
  setBooleanVectorContents(ZeroOrOneBooleanContent);

  // Set up the register classes.
  addRegisterClass(MVT::i32, &T8::IntRegsRegClass);

  setOperationAction(ISD::INTRINSIC_WO_CHAIN, MVT::Other, Custom);

  setMinFunctionAlignment(Align(4));

  computeRegisterProperties(Subtarget->getRegisterInfo());
}

bool T8xxTargetLowering::useSoftFloat() const {
  return Subtarget->useSoftFloat();
}


EVT T8xxTargetLowering::getSetCCResultType(const DataLayout &, LLVMContext &,
                                            EVT VT) const {
  if (!VT.isVector())
    return MVT::i32;
  return VT.changeVectorElementTypeToInteger();
}

/// isMaskedValueZeroForTargetNode - Return true if 'Op & Mask' is known to
/// be zero. Op is expected to be a target specific node. Used by DAG
/// combiner.
void T8xxTargetLowering::computeKnownBitsForTargetNode
                                (const SDValue Op,
                                 KnownBits &Known,
                                 const APInt &DemandedElts,
                                 const SelectionDAG &DAG,
                                 unsigned Depth) const {
  KnownBits Known2;
  Known.resetAll();

  switch (Op.getOpcode()) {
  default: break;
  case SPISD::SELECT_ICC:
  case SPISD::SELECT_XCC:
  case SPISD::SELECT_FCC:
    Known = DAG.computeKnownBits(Op.getOperand(1), Depth + 1);
    Known2 = DAG.computeKnownBits(Op.getOperand(0), Depth + 1);

    // Only known if known in both the LHS and RHS.
    Known = KnownBits::commonBits(Known, Known2);
    break;
  }
}

// Look at LHS/RHS/CC and see if they are a lowered setcc instruction.  If so
// set LHS/RHS and SPCC to the LHS/RHS of the setcc and SPCC to the condition.
static void LookThroughSetCC(SDValue &LHS, SDValue &RHS,
                             ISD::CondCode CC, unsigned &SPCC) {
  if (isNullConstant(RHS) && CC == ISD::SETNE &&
      (((LHS.getOpcode() == SPISD::SELECT_ICC ||
         LHS.getOpcode() == SPISD::SELECT_XCC) &&
        LHS.getOperand(3).getOpcode() == SPISD::CMPICC) ||
       (LHS.getOpcode() == SPISD::SELECT_FCC &&
        (LHS.getOperand(3).getOpcode() == SPISD::CMPFCC ||
         LHS.getOperand(3).getOpcode() == SPISD::CMPFCC_V9))) &&
      isOneConstant(LHS.getOperand(0)) && isNullConstant(LHS.getOperand(1))) {
    SDValue CMPCC = LHS.getOperand(3);
    SPCC = cast<ConstantSDNode>(LHS.getOperand(2))->getZExtValue();
    LHS = CMPCC.getOperand(0);
    RHS = CMPCC.getOperand(1);
  }
}

// Convert to a target node and set target flags.
SDValue T8xxTargetLowering::withTargetFlags(SDValue Op, unsigned TF,
                                             SelectionDAG &DAG) const {
  if (const GlobalAddressSDNode *GA = dyn_cast<GlobalAddressSDNode>(Op))
    return DAG.getTargetGlobalAddress(GA->getGlobal(),
                                      SDLoc(GA),
                                      GA->getValueType(0),
                                      GA->getOffset(), TF);

  if (const ConstantPoolSDNode *CP = dyn_cast<ConstantPoolSDNode>(Op))
    return DAG.getTargetConstantPool(CP->getConstVal(), CP->getValueType(0),
                                     CP->getAlign(), CP->getOffset(), TF);

  if (const BlockAddressSDNode *BA = dyn_cast<BlockAddressSDNode>(Op))
    return DAG.getTargetBlockAddress(BA->getBlockAddress(),
                                     Op.getValueType(),
                                     0,
                                     TF);

  if (const ExternalSymbolSDNode *ES = dyn_cast<ExternalSymbolSDNode>(Op))
    return DAG.getTargetExternalSymbol(ES->getSymbol(),
                                       ES->getValueType(0), TF);

  llvm_unreachable("Unhandled address SDNode");
}

// Split Op into high and low parts according to HiTF and LoTF.
// Return an ADD node combining the parts.
SDValue T8xxTargetLowering::makeHiLoPair(SDValue Op,
                                          unsigned HiTF, unsigned LoTF,
                                          SelectionDAG &DAG) const {
  SDLoc DL(Op);
  EVT VT = Op.getValueType();
  SDValue Hi = DAG.getNode(SPISD::Hi, DL, VT, withTargetFlags(Op, HiTF, DAG));
  SDValue Lo = DAG.getNode(SPISD::Lo, DL, VT, withTargetFlags(Op, LoTF, DAG));
  return DAG.getNode(ISD::ADD, DL, VT, Hi, Lo);
}

// Build SDNodes for producing an address from a GlobalAddress, ConstantPool,
// or ExternalSymbol SDNode.
SDValue T8xxTargetLowering::makeAddress(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  EVT VT = getPointerTy(DAG.getDataLayout());

  // Handle PIC mode first. SPARC needs a got load for every variable!
  if (isPositionIndependent()) {
    const Module *M = DAG.getMachineFunction().getFunction().getParent();
    PICLevel::Level picLevel = M->getPICLevel();
    SDValue Idx;

    if (picLevel == PICLevel::SmallPIC) {
      // This is the pic13 code model, the GOT is known to be smaller than 8KiB.
      Idx = DAG.getNode(SPISD::Lo, DL, Op.getValueType(),
                        withTargetFlags(Op, T8xxMCExpr::VK_T8xx_GOT13, DAG));
    } else {
      // This is the pic32 code model, the GOT is known to be smaller than 4GB.
      Idx = makeHiLoPair(Op, T8xxMCExpr::VK_T8xx_GOT22,
                         T8xxMCExpr::VK_T8xx_GOT10, DAG);
    }

    SDValue GlobalBase = DAG.getNode(SPISD::GLOBAL_BASE_REG, DL, VT);
    SDValue AbsAddr = DAG.getNode(ISD::ADD, DL, VT, GlobalBase, Idx);
    // GLOBAL_BASE_REG codegen'ed with call. Inform MFI that this
    // function has calls.
    MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
    MFI.setHasCalls(true);
    return DAG.getLoad(VT, DL, DAG.getEntryNode(), AbsAddr,
                       MachinePointerInfo::getGOT(DAG.getMachineFunction()));
  }

  // This is one of the absolute code models.
  switch(getTargetMachine().getCodeModel()) {
  default:
    llvm_unreachable("Unsupported absolute code model");
  case CodeModel::Small:
    // abs32.
    return makeHiLoPair(Op, T8xxMCExpr::VK_T8xx_HI,
                        T8xxMCExpr::VK_T8xx_LO, DAG);
  case CodeModel::Medium: {
    // abs44.
    SDValue H44 = makeHiLoPair(Op, T8xxMCExpr::VK_T8xx_H44,
                               T8xxMCExpr::VK_T8xx_M44, DAG);
    H44 = DAG.getNode(ISD::SHL, DL, VT, H44, DAG.getConstant(12, DL, MVT::i32));
    SDValue L44 = withTargetFlags(Op, T8xxMCExpr::VK_T8xx_L44, DAG);
    L44 = DAG.getNode(SPISD::Lo, DL, VT, L44);
    return DAG.getNode(ISD::ADD, DL, VT, H44, L44);
  }
  case CodeModel::Large: {
    // abs64.
    SDValue Hi = makeHiLoPair(Op, T8xxMCExpr::VK_T8xx_HH,
                              T8xxMCExpr::VK_T8xx_HM, DAG);
    Hi = DAG.getNode(ISD::SHL, DL, VT, Hi, DAG.getConstant(32, DL, MVT::i32));
    SDValue Lo = makeHiLoPair(Op, T8xxMCExpr::VK_T8xx_HI,
                              T8xxMCExpr::VK_T8xx_LO, DAG);
    return DAG.getNode(ISD::ADD, DL, VT, Hi, Lo);
  }
  }
}

SDValue T8xxTargetLowering::LowerGlobalAddress(SDValue Op,
                                                SelectionDAG &DAG) const {
  return makeAddress(Op, DAG);
}

SDValue T8xxTargetLowering::LowerConstantPool(SDValue Op,
                                               SelectionDAG &DAG) const {
  return makeAddress(Op, DAG);
}

SDValue T8xxTargetLowering::LowerBlockAddress(SDValue Op,
                                               SelectionDAG &DAG) const {
  return makeAddress(Op, DAG);
}

SDValue T8xxTargetLowering::LowerGlobalTLSAddress(SDValue Op,
                                                   SelectionDAG &DAG) const {
  return makeAddress(Op, DAG);
}


//===----------------------------------------------------------------------===//
//                         T8xx Inline Assembly Support
//===----------------------------------------------------------------------===//

/// getConstraintType - Given a constraint letter, return the type of
/// constraint it is for this target.
T8xxTargetLowering::ConstraintType
T8xxTargetLowering::getConstraintType(StringRef Constraint) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:  break;
    case 'r':
    case 'f':
    case 'e':
      return C_RegisterClass;
    case 'I': // SIMM13
      return C_Immediate;
    }
  }

  return TargetLowering::getConstraintType(Constraint);
}

TargetLowering::ConstraintWeight T8xxTargetLowering::
getSingleConstraintMatchWeight(AsmOperandInfo &info,
                               const char *constraint) const {
  ConstraintWeight weight = CW_Invalid;
  Value *CallOperandVal = info.CallOperandVal;
  // If we don't have a value, we can't do a match,
  // but allow it at the lowest weight.
  if (!CallOperandVal)
    return CW_Default;

  // Look at the constraint type.
  switch (*constraint) {
  default:
    weight = TargetLowering::getSingleConstraintMatchWeight(info, constraint);
    break;
  case 'I': // SIMM13
    if (ConstantInt *C = dyn_cast<ConstantInt>(info.CallOperandVal)) {
      if (isInt<13>(C->getSExtValue()))
        weight = CW_Constant;
    }
    break;
  }
  return weight;
}

/// LowerAsmOperandForConstraint - Lower the specified operand into the Ops
/// vector.  If it is invalid, don't add anything to Ops.
void T8xxTargetLowering::
LowerAsmOperandForConstraint(SDValue Op,
                             std::string &Constraint,
                             std::vector<SDValue> &Ops,
                             SelectionDAG &DAG) const {
  SDValue Result;

  // Only support length 1 constraints for now.
  if (Constraint.length() > 1)
    return;

  char ConstraintLetter = Constraint[0];
  switch (ConstraintLetter) {
  default: break;
  case 'I':
    if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
      if (isInt<13>(C->getSExtValue())) {
        Result = DAG.getTargetConstant(C->getSExtValue(), SDLoc(Op),
                                       Op.getValueType());
        break;
      }
      return;
    }
  }

  if (Result.getNode()) {
    Ops.push_back(Result);
    return;
  }
  TargetLowering::LowerAsmOperandForConstraint(Op, Constraint, Ops, DAG);
}

std::pair<unsigned, const TargetRegisterClass *>
T8xxTargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                                                  StringRef Constraint,
                                                  MVT VT) const {
}

bool
T8xxTargetLowering::isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const {
  // The T8xx target isn't yet aware of offsets.
  return false;
}

void T8xxTargetLowering::ReplaceNodeResults(SDNode *N,
                                             SmallVectorImpl<SDValue>& Results,
                                             SelectionDAG &DAG) const {

}
