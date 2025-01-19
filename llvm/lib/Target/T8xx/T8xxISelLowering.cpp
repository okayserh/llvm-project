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
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/KnownBits.h"
using namespace llvm;


const char *T8xxTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default:
    return NULL;
  case T8xxISD::RET_FLAG:
    return "RetFlag";
  case T8xxISD::LOAD_SYM:
    return "LOAD_SYM";
  case T8xxISD::CALL:
    return "CALL";
  case T8xxISD::LOAD_OP_STACK:
    return "LOAD_OP_STACK";
  case T8xxISD::ADD_WPTR:
    return "ADD_WPTR";
  case T8xxISD::CMOV:
    return "CMOV";
  case T8xxISD::EQ:
    return "EQ";
  case T8xxISD::BRNCOND:
    return "BRNCOND";
  case T8xxISD::LDIFF:
    return "LDIFF";
  case T8xxISD::REV:
    return "REV";
  }
}



// Return true if it is OK for this CMOV pseudo-opcode to be cascaded
// together with other CMOV pseudo-opcodes into a single basic-block with
// conditional jump around it.
static bool isCMOVPseudo(MachineInstr &MI) {
  switch (MI.getOpcode()) {
  case T8xx::CMOV32:
    return true;

  default:
    return false;
  }
}



T8xxTargetLowering::T8xxTargetLowering(const TargetMachine &TM,
                                         const T8xxSubtarget &STI)
    : TargetLowering(TM), Subtarget(&STI) {
  //  MVT PtrVT = MVT::getIntegerVT(TM.getPointerSizeInBits(0));

  // Set up the register classes.
  addRegisterClass(MVT::i32, &T8xx::ORegRegClass);

  if (Subtarget->useFPU ())
    {
      addRegisterClass(MVT::f32, &T8xx::FPRegRegClass);
      addRegisterClass(MVT::f64, &T8xx::DFPRegRegClass);
    }
  
  computeRegisterProperties(Subtarget->getRegisterInfo());

  // Was used in LEG architecture. Unclear what it does ...
  //  setSchedulingPreference (Sched::Source);

  for (auto VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1, Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1, Promote);
    setLoadExtAction(ISD::EXTLOAD, VT, MVT::i1, Promote);
  }

  // We don't accept any truncstore of integer registers.
  //  setTruncStoreAction(MVT::i32, MVT::i16, Custom);


  // setTruncStoreAction(MVT::i64, MVT::i16, Expand);
  //setTruncStoreAction(MVT::i64, MVT::i8, Expand);

  // Temporary disabled
  //setTruncStoreAction(MVT::i32, MVT::i8, Custom);

  //  setTruncStoreAction(MVT::i16, MVT::i8, Expand);

  //  setOperationAction(ISD::INTRINSIC_WO_CHAIN, MVT::Other, Custom);

  setMinFunctionAlignment(Align(4));

  // Nodes that require custom lowering
  setOperationAction(ISD::GlobalAddress, MVT::i32, Custom);

  setOperationAction(ISD::BRCOND, MVT::Other, Custom);

  setOperationAction(ISD::BR_CC, MVT::i8, Expand);
  setOperationAction(ISD::BR_CC, MVT::i16, Expand);
  setOperationAction(ISD::BR_CC, MVT::i32, Expand);

  setOperationAction(ISD::SELECT, MVT::i8, Custom);
  setOperationAction(ISD::SELECT, MVT::i16, Custom);
  setOperationAction(ISD::SELECT, MVT::i32, Custom);

  setOperationAction(ISD::SELECT_CC, MVT::i8, Expand);
  setOperationAction(ISD::SELECT_CC, MVT::i16, Expand);
  setOperationAction(ISD::SELECT_CC, MVT::i32, Expand);

  // TODO: Check wheter promote is correct for the other types
  setOperationAction(ISD::SETCC, MVT::i8, Promote);
  setOperationAction(ISD::SETCC, MVT::i16, Promote);
  setOperationAction(ISD::SETCC, MVT::i32, Custom);

  // T8xx doesn't have sext_inreg, replace them with shl/sra
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i16, Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8 , Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1 , Expand);

  // DIV/REM are legal on T8xx
  setOperationAction(ISD::SREM, MVT::i32, Legal);
  setOperationAction(ISD::SDIV, MVT::i32, Legal);

  setOperationAction(ISD::MULHS, MVT::i32, Expand);
  setOperationAction(ISD::MULHU, MVT::i32, Expand);

}

bool T8xxTargetLowering::useSoftFloat() const {
  if (Subtarget->useSoftFloat ())
    printf ("use Softfloat : true\n");
  else
    printf ("use Softfloat : false\n");

  if (Subtarget->useFPU ())
    printf ("use FPU : true\n");
  else
    printf ("use FPU : false\n");

  return Subtarget->useSoftFloat();
}


SDValue T8xxTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  printf ("### Lower Operation ### %i\n", Op.getOpcode ());

  switch (Op.getOpcode()) {
  default:
    llvm_unreachable("Unimplemented operand");
  case ISD::STORE:
    printf ("#### Lower Store #####\n");
    return LowerSTORE(Op, DAG);
  case ISD::SETCC:
    printf ("#### SETCC #####\n");
    return LowerSETCC(Op, DAG);
  case ISD::SELECT:
    printf ("####### Lower Select  #########\n");
    return LowerSELECT(Op, DAG);
  case ISD::BRCOND:
    return LowerBRCOND(Op, DAG);
  case ISD::GlobalAddress:
    printf ("####### Lower GlobalAddress  #########\n");
    return LowerGlobalAddress(Op, DAG);
  }
}


SDValue T8xxTargetLowering::LowerSTORE(SDValue Op, SelectionDAG &DAG) const
{
  // First test ...
  SDValue Op0 = Op.getOperand(0);
  SDValue Op1 = Op.getOperand(1);
  SDValue Op2 = Op.getOperand(2);

  Op0.dump ();
  Op1.dump ();
  Op2.dump ();

  return (Op);
}


SDValue T8xxTargetLowering::LowerSETCC(SDValue Op, SelectionDAG &DAG) const
{
  /*
  MVT VT = Op.getSimpleValueType();
  assert(VT == MVT::i8 && "SetCC type must be 8-bit integer");
  */

  SDValue Op0 = Op.getOperand(0);
  SDValue Op1 = Op.getOperand(1);
  SDLoc DL(Op);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(2))->get();

  // Catch the unsigned comparisons
  if (CC == ISD::SETUGT || CC == ISD::SETUGE ||
      CC == ISD::SETULT || CC == ISD::SETULE ||
      CC == ISD::SETUEQ || CC == ISD::SETUNE)
    {
      Op0.dump ();
      Op1.dump ();
      Op.getOperand(2).dump ();


      SDValue OneConst = DAG.getConstant(1, DL, MVT::i32);
      // TODO: In the VTList try to adapt this to the original node!
      SDValue Diff = DAG.getNode(T8xxISD::LDIFF, DL, DAG.getVTList(MVT::i32, MVT::i32),
				 OneConst, Op1, Op0);
      SDValue Rev = DAG.getNode(T8xxISD::REV, DL, MVT::i32,
				Diff.getValue(0), Diff.getValue(1));

      SDValue SetCC = DAG.getSetCC (DL, Rev.getOperand(0).getValueType (),
			    Rev.getValue(0),
			    DAG.getConstant(0, DL, MVT::i32),
			    ISD::CondCode::SETEQ);

      // TODO: Not the correct code.
      //return DAG.getNode(T8xxISD::EQ, DL, MVT::i32, Op0, Op1);
      return SetCC;
    }
  else
    return (Op);
}



SDValue T8xxTargetLowering::LowerBRCOND(SDValue Op, SelectionDAG &DAG) const {
  //  bool AddTest = true;
  SDValue Chain = Op.getOperand(0);
  SDValue Cond = Op.getOperand(1);
  SDValue Dest = Op.getOperand(2);
  SDLoc DL(Op);
  SDValue CC;
  //  bool Inverted = false;

  printf ("#### LowerBRCOND\n");

  SDValue NewCond;
  if (Cond.getOpcode() == ISD::SETCC) {
    // For SETCC use "inverse" comparison
    CondCodeSDNode *CCNode = cast<CondCodeSDNode>(Cond.getOperand(2));
    NewCond = DAG.getSetCC (DL, Cond.getOperand(0).getValueType (),
				 Cond.getOperand(0),
				 Cond.getOperand(1),
				 getSetCCInverse (CCNode->get(), Cond.getOperand(2).getValueType ()));
  } else {
    // Otherwise insert logical not (= EQ 0)
    SDValue CompConst = DAG.getConstant(0, DL, MVT::i32);
    NewCond = DAG.getNode(T8xxISD::EQ, DL, Op.getValueType (), Chain, Cond, CompConst);
  }

  // Use the "negative" BRCOND.
  return DAG.getNode(T8xxISD::BRNCOND, DL, Op.getValueType(), Chain, NewCond, Dest);
}


SDValue T8xxTargetLowering::LowerSELECT(SDValue Op, SelectionDAG &DAG) const
{
  bool addTest = true;
  SDValue Cond = Op.getOperand(0);
  SDValue Op1 = Op.getOperand(1);
  SDValue Op2 = Op.getOperand(2);
  SDLoc DL(Op);

  // TODO:
  /*
  if (Cond.getOpcode() == ISD::SETCC) {
    if (SDValue NewCond = LowerSETCC(Cond, DAG))
      Cond = NewCond;
  }
  */

  // T8xxISD::CMOV means set the result (which is operand 1) to the RHS if
  // condition is true.
  SDVTList VTs = DAG.getVTList(Op.getValueType(), MVT::Glue);
  SDValue Ops[] = {Cond, Op2, Op1};
  return DAG.getNode(T8xxISD::CMOV, DL, VTs, Ops);
}


SDValue T8xxTargetLowering::LowerGlobalAddress(SDValue Op, SelectionDAG& DAG) const
{
  SDValue Result;
  EVT VT = Op.getValueType();
  GlobalAddressSDNode *GlobalAddr = cast<GlobalAddressSDNode>(Op.getNode());
  int64_t Offset = cast<GlobalAddressSDNode>(Op)->getOffset();

  // TODO: Just a first try to see how things work.
  // Ideally a later version should be able to build position independent code as well
  // as code for a fixed address.
  Result = DAG.getTargetGlobalAddress(GlobalAddr->getGlobal(), SDLoc(Op), MVT::i32, 0, T8xxMCExpr::VK_T8xx_GLOBAL);

  Result = DAG.getNode(T8xxISD::LOAD_SYM, SDLoc(Op), VT, Result);

  if (Offset != 0)
    {
      SDValue PtrOff = DAG.getIntPtrConstant(Offset, SDLoc(Op));
      Result = DAG.getNode(ISD::ADD, SDLoc(Op), MVT::i32, Result, PtrOff);
    }

  return Result;
}


// This function creates nodes to replicate a select function
// in the DAG

MachineBasicBlock *
T8xxTargetLowering::EmitLoweredSelect(MachineInstr &MI,
                                      MachineBasicBlock *MBB) const {
  const TargetInstrInfo *TII = Subtarget->getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  printf ("EmitLoweredSelect\n");
  MI.dump ();

  // To "insert" a SELECT_CC instruction, we actually have to insert the
  // diamond control-flow pattern.  The incoming instruction knows the
  // destination vreg to set, the condition code register to branch on, the
  // true/false values to select between, and a branch opcode to use.
  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator It = ++MBB->getIterator();

  //  ThisMBB:
  //  ...
  //   TrueVal = ...
  //   cmp ccX, r1, r2
  //   bcc Copy1MBB
  //   fallthrough --> Copy0MBB
  MachineBasicBlock *ThisMBB = MBB;
  MachineFunction *F = MBB->getParent();  // *MF on AVR side

  MachineBasicBlock *Copy0MBB = F->CreateMachineBasicBlock(BB);  // falseMBB on AVR
  MachineBasicBlock *SinkMBB = F->CreateMachineBasicBlock(BB);   // trueMBB on AVR
  F->insert(It, Copy0MBB);
  F->insert(It, SinkMBB);

  // Set the call frame size on entry to the new basic blocks.
  unsigned CallFrameSize = TII->getCallFrameSizeAt(MI);
  Copy0MBB->setCallFrameSize(CallFrameSize);
  SinkMBB->setCallFrameSize(CallFrameSize);

  // Transfer the remainder of MBB and its successor edges to SinkMBB.
  // SinkMBB = bb.2
  SinkMBB->splice(SinkMBB->begin(), MBB,
                  std::next(MachineBasicBlock::iterator(MI)), MBB->end());
  SinkMBB->transferSuccessorsAndUpdatePHIs(MBB);

  MBB->addSuccessor(Copy0MBB);
  MBB->addSuccessor(SinkMBB);

  // Note:
  // cj, conditional jump
  // Areg = 0  -> Areg' = Areg
  //              Breg' = Breg
  //              Creg' = Creg
  //              Iptr' = ByteIndex NextInst Oreg0
  // Areg != 0 -> Areg' = Breg
  //              Breg' = Creg
  //              Creg' = undefined
  //              Iptr' = ByteIndex

  // Create the conditional branch instruction.
  BuildMI(MBB, DL, TII->get(T8xx::CJ)).addReg(MI.getOperand(1).getReg()).addMBB(SinkMBB);
  
  //  Copy0MBB:
  //   %FalseValue = ...
  //   # fallthrough to SinkMBB
  Copy0MBB->addSuccessor(SinkMBB);

  //  SinkMBB:
  //   %Result = phi [ %FalseValue, Copy0MBB ], [ %TrueValue, ThisMBB ]
  //  ...
  MachineBasicBlock::iterator MIItBegin = MachineBasicBlock::iterator(MI);
  MachineBasicBlock::iterator MIItEnd =
      std::next(MachineBasicBlock::iterator(MI));
  MachineBasicBlock::iterator SinkInsertionPoint = SinkMBB->begin();
  DenseMap<unsigned, std::pair<unsigned, unsigned>> RegRewriteTable;
  MachineInstrBuilder MIB;

  // As we are creating the PHIs, we have to be careful if there is more than
  // one.  Later CMOVs may reference the results of earlier CMOVs, but later
  // PHIs have to reference the individual true/false inputs from earlier PHIs.
  // That also means that PHI construction must work forward from earlier to
  // later, and that the code must maintain a mapping from earlier PHI's
  // destination registers, and the registers that went into the PHI.

  for (MachineBasicBlock::iterator MIIt = MIItBegin; MIIt != MIItEnd; ++MIIt) {
    Register DestReg = MIIt->getOperand(0).getReg();
    // Operand 1 is the condition
    Register Op1Reg = MIIt->getOperand(2).getReg();
    Register Op2Reg = MIIt->getOperand(3).getReg();

    // If this CMOV we are generating is the opposite condition from
    // the jump we generated, then we have to swap the operands for the
    // PHI that is going to be generated.
    /*
    if (MIIt->getOperand(3).getImm() == OppCC)
      std::swap(Op1Reg, Op2Reg);
    */

    if (RegRewriteTable.find(Op1Reg) != RegRewriteTable.end())
      Op1Reg = RegRewriteTable[Op1Reg].first;

    if (RegRewriteTable.find(Op2Reg) != RegRewriteTable.end())
      Op2Reg = RegRewriteTable[Op2Reg].second;

    MIB =
        BuildMI(*SinkMBB, SinkInsertionPoint, DL, TII->get(T8xx::PHI), DestReg)
            .addReg(Op1Reg)
            .addMBB(Copy0MBB)
            .addReg(Op2Reg)
            .addMBB(ThisMBB);

    // Add this PHI to the rewrite table.
    RegRewriteTable[DestReg] = std::make_pair(Op1Reg, Op2Reg);
  }

  // Now remove the CMOV(s).
  for (MachineBasicBlock::iterator MIIt = MIItBegin; MIIt != MIItEnd;)
    (MIIt++)->eraseFromParent();

  return SinkMBB;
}


MachineBasicBlock *
T8xxTargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
						MachineBasicBlock *MBB) const
{
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected instr type to insert");
  case T8xx::CMOV32:  //TODO: Need an pseudo instruction definition in the target description
    return EmitLoweredSelect(MI, MBB);
  }

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
  // Old Code (LLVM 17)
  //  const unsigned NumBytes = CCInfo.getNextStackOffset();
  // New Code (LLVM 18) ???
  unsigned NumBytes = CCInfo.getStackSize();

  /* Old LEG Code
  Chain =
    DAG.getCALLSEQ_START(Chain, DAG.getIntPtrConstant(NumBytes, Loc, true), 0,
                           Loc);
  */
  Chain = DAG.getCALLSEQ_START(Chain, NumBytes, 0, Loc);

  SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;

  printf ("ArgLocs %li\n", ArgLocs.size());

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

    // TODO: The resulting code looks like:
    // ldlp -2
    // stl %r2
    // ldl %r1
    // ldl %r2
    // stnl
    // I.e. the address of the frame location is first put into a
    // register and then the actual parameter is copied
    // via a stnl to that address
    // Might be faster to use:
    // ldl %r1
    // stl -2

    SDValue StackPtr = DAG.getRegister(T8xx::WPTR, MVT::i32);
    assert (VA.getLocMemOffset() % 4 == 0 &&
	    "Only 4 byte aligned offset allowed");
    SDValue PtrOff = DAG.getSignedConstant(-(VA.getLocMemOffset() + 4), Loc,
					   getPointerTy(DAG.getDataLayout()));
    PtrOff = DAG.getNode(T8xxISD::ADD_WPTR, Loc, MVT::i32, StackPtr, PtrOff);

    MemOpChains.push_back(DAG.getStore(Chain, Loc, Arg, PtrOff,
                                       MachinePointerInfo()));

    /*
    SDValue StackPtr = DAG.getRegister(T8xx::WPTR, MVT::i32);
    assert (VA.getLocMemOffset() % 4 == 0 &&
	    "Only 4 byte aligned offset allowed");

    int32_t Offset = VA.getLocMemOffset();
    SDValue PtrOff = DAG.getIntPtrConstant(-(Offset / 4 + 1), Loc);

    PtrOff = DAG.getNode(ISD::ADD, Loc, MVT::i32, StackPtr, PtrOff);
    MachinePointerInfo DstInfo =
            MachinePointerInfo::getStack(DAG.getMachineFunction(), Offset);

    MemOpChains.push_back(DAG.getStore(Chain, Loc, Arg, PtrOff,
                                       DstInfo));
    */
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
    Callee = DAG.getTargetGlobalAddress(G->getGlobal(), Loc, MVT::i32, 0, T8xxMCExpr::VK_T8xx_IPTRREL);
  else if (ExternalSymbolSDNode *E = dyn_cast<ExternalSymbolSDNode>(Callee))
    Callee = DAG.getTargetExternalSymbol(E->getSymbol(), MVT::i32, T8xxMCExpr::VK_T8xx_IPTRREL);


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

      const unsigned VReg = RegInfo.createVirtualRegister(&T8xx::ORegRegClass);

      RegInfo.addLiveIn(VA.getLocReg(), VReg);
      SDValue ArgIn = DAG.getCopyFromReg(Chain, DL, VReg, RegVT);

      InVals.push_back(ArgIn);
      continue;
    }

    /* This generates a sequence of "LDL" instructions, the results of which
       are not used later
       NOTE: Leads to exception when this code is not present! */
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
  return CCInfo.CheckReturn(Outs, RetCC_T8xx32);
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

  printf ("Temp A, RVLocs Size %li\n", RVLocs.size());

  // OKH: General remark, in Webassembly, the operands are directly
  // used for return. However, the return instruction follows after
  // the "epilogue" code. Hence, the registers have already
  // been displaced.

  // Copy the result values into the output registers.
  for (unsigned i = 0, e = RVLocs.size(); i < e; ++i) {
    // OKH: In VA the locations for return values are stored. These are
    // also provided as operands to the "RET_FLAG" machine ISD.
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), OutVals[i], Flag);

    Flag = Chain.getValue(1);

    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  RetOps[0] = Chain;  // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);

  SDValue ret = DAG.getNode(T8xxISD::RET_FLAG, DL, MVT::Other, RetOps);

  printf ("Post Lower Return\n");
  DAG.dump ();

  printf ("Temp C\n");

  return ret;
}


//===----------------------------------------------------------------------===//
//  Inline Assembler Implementation Methods
//===----------------------------------------------------------------------===//


/* TODO:
Currently, the inline assembler does not work properly, since
this method is used to determine the general constraint type.
Default implementation is in CodeGen/SelectionDAG/TargetLowering.cpp

T8xxTargetLowering::ConstraintType
T8xxTargetLowering::getConstraintType(StringRef Constraint) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    case 'R':
    case 'q':
    case 'Q':
      return C_RegisterClass;
    default:
      break;
    }
  }
  return TargetLowering::getConstraintType(Constraint);
}
*/

std::pair<unsigned, const TargetRegisterClass *>
T8xxTargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
						 StringRef Constraint,
						 MVT VT) const {
  switch (Constraint.size()) {
  case 1:
    // GCC ARM Constraint Letters
    switch (Constraint[0]) {
    case 'a':
    case 'b':
    case 'c':
    case 'r':
      return std::make_pair(0U, &T8xx::ORegRegClass);
    }
    break;

  default:
    break;
  }

  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
}
