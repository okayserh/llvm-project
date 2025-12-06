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
#include "llvm/CodeGen/MachineJumpTableInfo.h"
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

#define DEBUG_TYPE "t8xx-codegen"

const char *T8xxTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default:
    return NULL;
  case T8xxISD::CALL:
    return "CALL";
  case T8xxISD::RET_FLAG:
    return "RetFlag";
  case T8xxISD::LOAD_SYM:
    return "LOAD_SYM";
  case T8xxISD::ADD_WPTR:
    return "ADD_WPTR";
  case T8xxISD::AJW:
    return "AJW";
  case T8xxISD::ADD_IPTR:
    return "ADD_IPTR";
  case T8xxISD::STL_PARM:
    return "STL_PARM";
  case T8xxISD::MOVE:
    return "MOVE";
  case T8xxISD::MoveLoad:
    return "MoveLoad";
  case T8xxISD::MoveSEXTLoad:
    return "MoveSEXTLoad";
  case T8xxISD::MoveZEXTLoad:
    return "MoveZEXTLoad";
  case T8xxISD::CMOV:
    return "CMOV";
  case T8xxISD::BRNCOND:
    return "BRNCOND";
  case T8xxISD::LDIFF:
    return "LDIFF";
  case T8xxISD::REV:
    return "REV";
  case T8xxISD::JOIN:
    return "JOIN";
  case T8xxISD::FP_SETCC:
    return "FP_SETCC";
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
  : TargetLowering(TM, STI), Subtarget(STI) {
  MVT PtrVT = MVT::getIntegerVT(TM.getPointerSizeInBits(0));

  // Set up the register classes.
  addRegisterClass(MVT::i32, &T8xx::ORegRegClass);

  if (Subtarget.useFPU ())
    {
      addRegisterClass(MVT::f32, &T8xx::FPRegRegClass);
      addRegisterClass(MVT::f64, &T8xx::DFPRegRegClass);
    }

  computeRegisterProperties(Subtarget.getRegisterInfo());

  for (auto VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1, Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1, Promote);
    setLoadExtAction(ISD::EXTLOAD, VT, MVT::i1, Promote);
  }

  setTruncStoreAction(MVT::i32, MVT::i8, Legal);
  setTruncStoreAction(MVT::i32, MVT::i16, Custom);
  //setTruncStoreAction(MVT::i32, MVT::i16, Legal);
  setLoadExtAction(ISD::EXTLOAD, MVT::i32, MVT::i16, Custom);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i32, MVT::i16, Custom);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i32, MVT::i16, Custom);

  setMinFunctionAlignment(Align(4));

  // TODO: Test ...
  if (Subtarget.useFPU ())
    {
      // Transputer does not have floating-point extending loads.
      for (MVT VT : MVT::fp_valuetypes()) {
	setLoadExtAction(ISD::EXTLOAD, VT, MVT::f32, Expand);
	setLoadExtAction(ISD::EXTLOAD, VT, MVT::f16, Expand);
	setLoadExtAction(ISD::EXTLOAD, VT, MVT::bf16, Expand);
      }
      // ... or truncating stores
      setTruncStoreAction(MVT::f64, MVT::f32, Expand);
      setTruncStoreAction(MVT::f32, MVT::f16, Expand);
      setTruncStoreAction(MVT::f64, MVT::f16, Expand);
      setTruncStoreAction(MVT::f32, MVT::bf16, Expand);
      setTruncStoreAction(MVT::f64, MVT::bf16, Expand);

      // Condition codes
      setOperationAction(ISD::SETCC, MVT::f32, Custom);
      setOperationAction(ISD::SETCC, MVT::f64, Custom);

      setOperationAction(ISD::BR_CC, MVT::f32, Expand);
      setOperationAction(ISD::BR_CC, MVT::f64, Expand);

      // No native transputer instruction available
      setOperationAction(ISD::FNEG, MVT::f32, Expand);
      setOperationAction(ISD::FNEG, MVT::f64, Expand);
    }

  // Nodes that require custom lowering
  setOperationAction(ISD::GlobalAddress, PtrVT, Custom);

  // TODO: Test code to check what this does?
  setOperationAction(ISD::BlockAddress,       MVT::i32,   Custom);
  setOperationAction(ISD::GlobalTLSAddress,   MVT::i32,   Custom);
  setOperationAction(ISD::JumpTable,          MVT::i32, Custom);
  setOperationAction(ISD::ConstantPool,       MVT::i32, Custom);

  setOperationAction(ISD::BRCOND, MVT::Other, Custom);

  setOperationAction(ISD::BR_CC, MVT::i8, Expand);
  setOperationAction(ISD::BR_CC, MVT::i16, Expand);
  setOperationAction(ISD::BR_CC, MVT::i32, Expand);

  setOperationAction(ISD::BR_JT, MVT::Other, Expand);

  // Note, the Custom code only provides functionality
  // for i32 values. For the other value types, use
  // "Promote" to indicate that those types need
  // to be promoted to i32.
  // If not configured this way, the test PowerPC/testComparesieqsc.ll
  // failed!
  setOperationAction(ISD::SELECT, MVT::i8, Promote);
  setOperationAction(ISD::SELECT, MVT::i16, Promote);
  setOperationAction(ISD::SELECT, MVT::i32, Custom);

  setOperationAction(ISD::SELECT_CC, MVT::i8, Expand);
  setOperationAction(ISD::SELECT_CC, MVT::i16, Expand);
  setOperationAction(ISD::SELECT_CC, MVT::i32, Expand);

  // TODO: Implement efficiently
  setOperationAction(ISD::SHL_PARTS, MVT::i32, Expand);
  setOperationAction(ISD::SRA_PARTS, MVT::i32, Expand);
  setOperationAction(ISD::SRL_PARTS, MVT::i32, Expand);

  setOperationAction(ISD::SMUL_LOHI, MVT::i32, Expand);
  setOperationAction(ISD::UMUL_LOHI, MVT::i32, Expand);

  // TODO: Check wheter promote is correct for the other types
  setOperationAction(ISD::SETCC, MVT::i8, Promote);
  setOperationAction(ISD::SETCC, MVT::i16, Promote);
  setOperationAction(ISD::SETCC, MVT::i32, Custom);

  // Instructions not natively supported by Transputers
  // TODO: Some of these seem to be available in the T8xx series.
  setOperationAction(ISD::CTPOP,             MVT::i32, Expand);
  setOperationAction(ISD::CTTZ,              MVT::i32, Expand);
  setOperationAction(ISD::CTLZ,              MVT::i32, Expand);
  setOperationAction(ISD::ROTL,              MVT::i32, Expand);
  setOperationAction(ISD::BSWAP,             MVT::i32, Expand);
  setOperationAction(ISD::BITCAST,           MVT::i32, Expand);

  // T8xx doesn't have sext_inreg, replace them with shl/sra
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i16, Legal);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8 , Legal);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1 , Expand);

  setOperationAction(ISD::SIGN_EXTEND, MVT::i16, Legal);
  setOperationAction(ISD::ZERO_EXTEND, MVT::i16, Legal);
  setOperationAction(ISD::SIGN_EXTEND, MVT::i32, Legal);
  setOperationAction(ISD::ZERO_EXTEND, MVT::i32, Legal);

  // Operations for variadic arguments
  setOperationAction(ISD::VASTART, MVT::Other, Custom);
  setOperationAction({ISD::VAARG, ISD::VACOPY, ISD::VAEND}, MVT::Other, Expand);


  // ATOMIC Operations seem to "kill" the build.
  /*
  setOperationAction(ISD::ATOMIC_FENCE,   MVT::Other,
                       Subtarget.hasAnyDataBarrier() ? Custom : Expand);

    // Set them all for libcall, which will force libcalls.
    setOperationAction(ISD::ATOMIC_CMP_SWAP, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_SWAP, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_ADD, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_SUB, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_AND, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_OR, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_XOR, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_NAND, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_MIN, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_MAX, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_UMIN, MVT::i32, LibCall);
    setOperationAction(ISD::ATOMIC_LOAD_UMAX, MVT::i32, LibCall);
    // Mark ATOMIC_LOAD and ATOMIC_STORE custom so we can handle the
    // Unordered/Monotonic case.
    if (!InsertFencesForAtomic) {
      setOperationAction(ISD::ATOMIC_LOAD, MVT::i32, Custom);
      setOperationAction(ISD::ATOMIC_STORE, MVT::i32, Custom);
    }
  */

  // Alternatively?
  // Cortex-M (besides Cortex-M0) have 32-bit atomics.
  setMaxAtomicSizeInBitsSupported(32);


  /*
    n LLVM, the "max lock-free size" for atomic operations is primarily determined by the target architecture's capabilities and the TargetMachine/TargetLowering implementations within LLVM. It's not typically a single, easily configurable setting in a user-facing configuration file.

Here's a breakdown of where this information is defined and how it impacts atomic operations:

    Target-Specific Implementation:
        LLVM's code generation for atomic operations is highly dependent on the specific CPU architecture you're targeting (x86, ARM, MIPS, etc.).
        Each target's TargetLowering (or similar) implementation defines which atomic operations can be performed natively and lock-free for various data sizes.
        For instance, an x86 target will likely have native lock-free support for 8-byte (64-bit) atomics, and potentially 16-byte (128-bit) atomics using instructions like LOCK CMPXCHG16B on supported CPUs. Older or simpler architectures might only support smaller lock-free sizes.

    setMaxAtomicSizeInBitsSupported():
        Within the LLVM codebase, there's a function like setMaxAtomicSizeInBitsSupported() (or similar methods in the TargetLowering classes) that a target backend uses to declare the maximum size (in bits) for which it can generate inline, lock-free atomic instructions.

If an atomic operation is requested for a size larger than what the target natively supports as lock-free, LLVM's AtomicExpandPass will typically expand it into calls to library functions (e.g., __atomic_* libcalls). These library functions then use software-based locking mechanisms (like mutexes) to ensure atomicity, which means they are not lock-free. The default for setMaxAtomicSizeInBitsSupported is often 0, meaning that if a target doesn't explicitly declare support, all atomics might be expanded to libcalls.

Compiler-RT (libatomic):

    For sizes that are not natively lock-free on a given target, LLVM (via Clang and compiler-rt) relies on the libatomic library. This library provides the fallback implementations for atomic operations using mutexes when hardware support is insufficient. You can find the source for these in the LLVM project, typically in compiler-rt/lib/builtins/atomic.c or similar.

In summary, you don't "define" the max lock-free size in a simple configuration file that a user can easily change. It's inherently tied to:

    The LLVM target backend's implementation: Each backend (e.g., X86TargetLowering) specifies what its hardware can do.
    The specific CPU architecture: Different CPUs have different atomic capabilities.
    The AtomicExpandPass and compiler-rt: These components handle the fallback to library calls when native lock-free operations are not possible.

If you're developing an LLVM backend for a new architecture, you would implement the necessary logic within your target's TargetLowering class to accurately reflect the lock-free atomic capabilities of that architecture.
  */

  // DIV/REM are legal on T8xx
  setOperationAction(ISD::SREM, MVT::i32, Legal);
  setOperationAction(ISD::SDIV, MVT::i32, Legal);

  setOperationAction(ISD::MULHS, MVT::i32, Expand);
  setOperationAction(ISD::MULHU, MVT::i32, Expand);

}

bool T8xxTargetLowering::useSoftFloat() const {
  if (Subtarget.useSoftFloat ())
    LLVM_DEBUG(dbgs() << "use Softfloat : true\n");
  else
    LLVM_DEBUG(dbgs() << "use Softfloat : false\n");

  if (Subtarget.useFPU ())
    LLVM_DEBUG(dbgs() << "use FPU : true\n");
  else
    LLVM_DEBUG(dbgs() << "use FPU : false\n");

  return Subtarget.useSoftFloat();
}


void T8xxTargetLowering::ReplaceNodeResults(SDNode *N,
                                           SmallVectorImpl<SDValue>&Results,
                                           SelectionDAG &DAG) const {
  // TODO: See whether this needs to be implemented
  N->dump ();
  llvm_unreachable("ReplaceNodeResults not implemented for this target!");
}


SDValue T8xxTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  LLVM_DEBUG(dbgs() << "### Lower Operation ### " << Op.getOpcode () << "\n");

  switch (Op.getOpcode()) {
  default:
    llvm_unreachable("Unimplemented operand");
  case ISD::STORE:
    LLVM_DEBUG(dbgs() << "#### Lower Store #####\n");
    return LowerSTORE(Op, DAG);

  case ISD::LOAD:
    LLVM_DEBUG(dbgs() << "#### Lower Load #####\n");
    return LowerLOAD(Op, DAG);

  case ISD::SETCC:
    LLVM_DEBUG(dbgs() << "#### SETCC #####\n");
    return LowerSETCC(Op, DAG);
  case ISD::SELECT:
    LLVM_DEBUG(dbgs() << "####### Lower Select  #########\n");
    return LowerSELECT(Op, DAG);
  case ISD::BRCOND:
    return LowerBRCOND(Op, DAG);
  case ISD::VASTART:
    return LowerVASTART(Op, DAG);

  case ISD::GlobalAddress:
    LLVM_DEBUG(dbgs() << "####### Lower GlobalAddress  #########\n");
    return LowerGlobalAddress(Op, DAG);

    //TODO: These four may need reevaluation
  case ISD::ConstantPool:
    LLVM_DEBUG(dbgs() << "####### Lower ConstantPool  #########\n");
    return LowerConstantPool(Op, DAG);

  case ISD::JumpTable:
    LLVM_DEBUG(dbgs() << "####### Lower JumpTable  #########\n");
    return LowerJumpTable(Op, DAG);

  case ISD::BlockAddress:
    LLVM_DEBUG(dbgs() << "####### Lower BlockAddress  #########\n");
    return LowerBlockAddress(Op, DAG);

  case ISD::GlobalTLSAddress:
    LLVM_DEBUG(dbgs() << "####### Lower GlobalTLSAddress  #########\n");
    return LowerGlobalAddress(Op, DAG);
  }
}


SDValue T8xxTargetLowering::LowerSTORE(SDValue Op, SelectionDAG &DAG) const
{
  if (StoreSDNode *StoreOp = dyn_cast<StoreSDNode>(Op))
    {
      if (StoreOp->getMemoryVT().getScalarSizeInBits () == 16)
	{
	  SDLoc DL(Op);

	  // Note: The following steps have to be carried out:
	  // Store in a 4 byte aligned temporary slot in the workspace
	  // MOVE from the aligned source address towards the unaligned
	  // final address. Truncation is automatically done, due to the
	  // limited number of bytes copied.

	  // --- 2. Allocate space in the stack (workframe) for the aligned target ---
	  // Get a FrameIndex for a temporary 32-bit aligned location.
	  // This is a common pattern for targets that can't handle unaligned memory.
	  MachineFunction &MF = DAG.getMachineFunction();
	  T8xxMachineFunctionInfo *FuncInfo = MF.getInfo<T8xxMachineFunctionInfo>();

	  // Check whether a workspace location was already allocated
	  // as temporary storage for Move instructions
	  int FI = FuncInfo->getMoveSlot();
	  if (FI == 0)
	    {
	      FI = DAG.getMachineFunction().
		getFrameInfo().CreateStackObject(4, // Size in bytes for i16
						 Align(4), // Required alignment for the load to the frame
						 false); // isImmutable
	      FuncInfo->setMoveSlot(FI);
	    }

	  SDValue FIPtr = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));

	  // --- 4. Perform the aligned load from the workframe ---
	  // Now perform the final aligned 16-bit load from the frame index
	  SDValue Result = DAG.getStore(StoreOp->getChain(), DL, StoreOp->getValue(), FIPtr,
					MachinePointerInfo::getFixedStack(DAG.getMachineFunction(), FI), Align(4));

	  LLVM_DEBUG({
	      dbgs() << "Store node created\n";
	      Result->dump ();
	    });

	  // --- 3. Perform the unaligned move (a smaller byte-by-byte store/load) ---
	  SDValue MoveLen = DAG.getConstant(2, DL, MVT::i32);
	  SDValue Ptr = StoreOp->getBasePtr ();
	  SDValue Move = DAG.getNode(T8xxISD::MOVE, DL, MVT::Other, Result,
				     MoveLen, Ptr, FIPtr);

	  LLVM_DEBUG({
	      dbgs() << "Move node created\n";
	      Move->dump ();
	    });

	  /*
	  dbgs() << "StoreMove node created\n";
	  Op->dump ();

	  // --- 3. Perform the unaligned move (a smaller byte-by-byte store/load) ---
	  SDValue MoveLen = DAG.getConstant(2, DL, MVT::i32);
	  SDValue Ptr = StoreOp->getBasePtr ();
	  SDValue Chain = StoreOp->getChain();  // Output chain from original LOAD node

	  SDVTList VTs = DAG.getVTList(MVT::Other);
	  SDValue Move = DAG.getNode(T8xxISD::StoreMove, DL, VTs,
				     StoreOp->getValue(), MoveLen, FIPtr, Ptr);

	  StoreOp->getValue().dump();
	  */

	  return (Move);
	}
      else
	return (Op);
    }

  return (Op);
}


SDValue T8xxTargetLowering::LowerLOAD(SDValue Op, SelectionDAG &DAG) const
{
  if (LoadSDNode *LoadOp = dyn_cast<LoadSDNode>(Op))
    {
      // Loads arbitrary memory location into 32 bit value
      // Otherwise, skip ...
      if (Op.getSimpleValueType() == MVT::i32)
	{
	  if (LoadOp->getMemoryVT().getScalarSizeInBits () == 16)
	    {
	      SDLoc DL(Op);
	      // Note: The following steps have to be carried out:
	      // MOVE from the unaligned source address towards a 4 byte aligned
	      // temporary slot in the workspace. Then do a regular LDL with
	      // possible EXT/SEXT/ZEXT.

	      // --- 2. Allocate space in the stack (workframe) for the aligned target ---
	      // Get a FrameIndex for a temporary 32-bit aligned location.
	      // This is a common pattern for targets that can't handle unaligned memory.
	      MachineFunction &MF = DAG.getMachineFunction();
	      T8xxMachineFunctionInfo *FuncInfo = MF.getInfo<T8xxMachineFunctionInfo>();

	      // Check whether a workspace location was already allocated
	      // as temporary storage for Move instructions
	      int FI = FuncInfo->getMoveSlot();
	      if (FI == 0)
		{
		  FI = DAG.getMachineFunction().
		    getFrameInfo().CreateStackObject(4, // Size in bytes for i16
						     Align(4), // Required alignment for the load to the frame
						     false); // isImmutable
		  FuncInfo->setMoveSlot(FI);
		}

	      SDValue FIPtr = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));

	      // --- 3. Perform the unaligned move (a smaller byte-by-byte store/load) ---
	      SDValue MoveLen = DAG.getConstant(2, DL, MVT::i32);
	      SDValue Ptr = LoadOp->getBasePtr ();
	      SDValue Chain = LoadOp->getChain();  // Output chain from original LOAD node

	      unsigned MoveOpcode = T8xxISD::MoveLoad;
	      if (LoadOp->getExtensionType () == ISD::SEXTLOAD)
		MoveOpcode = T8xxISD::MoveSEXTLoad;
	      if (LoadOp->getExtensionType () == ISD::ZEXTLOAD)
		MoveOpcode = T8xxISD::MoveZEXTLoad;

	      SDVTList VTs = DAG.getVTList(MVT::i32, MVT::Other);
	      SDValue Move = DAG.getNode(MoveOpcode, DL, VTs,
					 Chain, MoveLen, FIPtr, Ptr, FIPtr);
	      return (Move);
	    }
	  else
	    return (Op);
	}
      else
	return (Op);
    }

  return (Op);
}


SDValue T8xxTargetLowering::LowerSETCC(SDValue Op, SelectionDAG &DAG) const
{
  SDValue Op0 = Op.getOperand(0);
  SDValue Op1 = Op.getOperand(1);
  SDLoc DL(Op);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(2))->get();

  /*
  if (Op0.getValueType ().isFloatingPoint ())
    {
      switch (CC)
	{
	case ISD::SETEQ:
	case ISD::SETGT:
	  return (Op);
	}

      // TODO: Quick hack to see if it catches floating point comparisons
      SDValue NewCond;
      NewCond = DAG.getSetCC (DL, Op.getValueType (),
			      Op0, Op1, ISD::SETOLT);
      return (NewCond);
    }
  */

  return (Op);
}


SDValue T8xxTargetLowering::LowerBRCOND(SDValue Op, SelectionDAG &DAG) const {
  //  bool AddTest = true;
  SDValue Chain = Op.getOperand(0);
  SDValue Cond = Op.getOperand(1);
  SDValue Dest = Op.getOperand(2);
  SDLoc DL(Op);
  SDValue CC;

  LLVM_DEBUG(dbgs() << "#### LowerBRCOND\n");

  SDValue NewCond;
  if (Cond.getOpcode() == ISD::SETCC) {

    if (Cond.getOperand(0).getValueType().isFloatingPoint())
      {
	CondCodeSDNode *CCNode = cast<CondCodeSDNode>(Cond.getOperand(2));
	ISD::CondCode invCC = getSetCCInverse (CCNode->get(), Cond.getOperand(2).getValueType ());
	ISD::CondCode origCC = CCNode->get ();

	NewCond = DAG.getSetCC (DL, Cond.getValueType (),
				Cond.getOperand(0),
				Cond.getOperand(1),
				invCC);
      }
    else
      {
	CondCodeSDNode *CCNode = cast<CondCodeSDNode>(Cond.getOperand(2));
	ISD::CondCode invCC = getSetCCInverse (CCNode->get(), Cond.getOperand(2).getValueType ());
	ISD::CondCode origCC = CCNode->get ();

	switch (origCC)
	  {
	    // The LLVM function "getSetCCInverse" provides inverse comparisons
	    // which may not always be suitable for the integer comparisons.
	    // Hence the following table provides inversions based on the resulting
	    // assembler code.
	  case ISD::SETEQ:
	    invCC = ISD::SETNE;
	    break;
	  case ISD::SETNE:
	    invCC = ISD::SETEQ;
	    break;
	  case ISD::SETLT:
	    invCC = ISD::SETGE;
	    break;
	  case ISD::SETLE:
	    invCC = ISD::SETGT;
	    break;
	  case ISD::SETGT:
	    invCC = ISD::SETLE;
	    break;
	  case ISD::SETGE:
	    invCC = ISD::SETLT;
	    break;
	  case ISD::SETUEQ:
	    invCC = ISD::SETUNE;
	    break;
	  case ISD::SETUNE:
	    invCC = ISD::SETUEQ;
	    break;
	  case ISD::SETUGE:
	    invCC = ISD::SETULT;
	    break;
	  case ISD::SETUGT:
	    invCC = ISD::SETULE;
	    break;
	  case ISD::SETULE:
	    invCC = ISD::SETUGT;
	    break;
	  case ISD::SETULT:
	    invCC = ISD::SETUGE;
	    break;
	    // Otherwise use the original condition and introduce a negation

	  default:
	    llvm_unreachable("Unsupported condition code!!!");
	  }

	NewCond = DAG.getSetCC (DL, Cond.getValueType (),
				Cond.getOperand(0),
				Cond.getOperand(1),
				invCC);
      }
  } else {

    LLVM_DEBUG(dbgs() << "#### LowerBRCOND Negation Case\n");

    SDValue Op0 = Op.getOperand(0);
    SDValue Op1 = Op.getOperand(1);
    SDValue Op2 = Op.getOperand(2);

    LLVM_DEBUG({
	Op0.dump ();
	Op1.dump ();
	Op2.dump ();
      });

    // Otherwise insert logical not (= EQ 0)
    NewCond = DAG.getSetCC (DL, Cond.getValueType (),
			    Cond.getValue(0),
			    DAG.getConstant(0, DL, MVT::i32),
			    ISD::CondCode::SETEQ);
  }

  // Use the "negative" BRCOND.
  return DAG.getNode(T8xxISD::BRNCOND, DL, Op.getValueType(), Chain, NewCond, Dest);
}


// Copied from RISCVISelLowering.cpp
SDValue T8xxTargetLowering::LowerVASTART(SDValue Op, SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();
  T8xxMachineFunctionInfo *FuncInfo = MF.getInfo<T8xxMachineFunctionInfo>();

  SDLoc DL(Op);
  SDValue FI = DAG.getFrameIndex(FuncInfo->getVarArgsFrameIndex(),
                                 getPointerTy(MF.getDataLayout()));

  // vastart just stores the address of the VarArgsFrameIndex slot into the
  // memory location argument.
  const Value *SV = cast<SrcValueSDNode>(Op.getOperand(2))->getValue();
  return DAG.getStore(Op.getOperand(0), DL, FI, Op.getOperand(1),
                      MachinePointerInfo(SV));
}


SDValue T8xxTargetLowering::LowerSELECT(SDValue Op, SelectionDAG &DAG) const
{
  bool addTest = true;
  SDValue Cond = Op.getOperand(0);
  SDValue Op1 = Op.getOperand(1);
  SDValue Op2 = Op.getOperand(2);
  SDLoc DL(Op);

  // T8xxISD::CMOV means set the result (which is operand 1) to the RHS if
  // condition is true.
  SDVTList VTs = DAG.getVTList(Op.getValueType(), MVT::Glue);
  SDValue Ops[] = {Cond, Op1, Op2};
  return DAG.getNode(T8xxISD::CMOV, DL, VTs, Ops);
}


SDValue T8xxTargetLowering::LowerGlobalAddress(SDValue Op, SelectionDAG& DAG) const
{
  SDValue Result;
  EVT VT = Op.getValueType();
  GlobalAddressSDNode *GlobalAddr = cast<GlobalAddressSDNode>(Op.getNode());
  int64_t Offset = cast<GlobalAddressSDNode>(Op)->getOffset();

  //  assert(GlobalAddr->getOffset() == 0 && "unexpected offset in global node");

  // TODO: Just a first try to see how things work.
  // Ideally a later version should be able to build position independent code as well
  // as code for a fixed address.
  Result = DAG.getTargetGlobalAddress(GlobalAddr->getGlobal(), SDLoc(Op), MVT::i32, 0, T8xxII::MO_GLOBAL);
  Result = DAG.getNode(T8xxISD::LOAD_SYM, SDLoc(Op), VT, Result);

  if (Offset != 0)
    {
      LLVM_DEBUG(dbgs() << "LowerGlobalAddress Ofset:" << Offset << "\n");

      if (Offset > 0)
	{
	  SDValue PtrOff = DAG.getIntPtrConstant(Offset, SDLoc(Op));
	  Result = DAG.getNode(ISD::ADD, SDLoc(Op), MVT::i32, Result, PtrOff);
	}
      else
	{
	  SDValue PtrOff = DAG.getIntPtrConstant(-Offset, SDLoc(Op));
	  Result = DAG.getNode(ISD::SUB, SDLoc(Op), MVT::i32, Result, PtrOff);
	}
    }

  return Result;
}


SDValue T8xxTargetLowering::LowerConstantPool(SDValue Op, SelectionDAG& DAG) const
{
  SDValue Result;
  ConstantPoolSDNode *CP = cast<ConstantPoolSDNode>(Op.getNode());

  // TODO: Just a first try to see how things work.
  // Ideally a later version should be able to build position independent code as well
  // as code for a fixed address.
  Result = DAG.getTargetConstantPool(CP->getConstVal(), CP->getValueType(0),
				     CP->getAlign(), CP->getOffset(), T8xxII::MO_PCREL_SYM);

  EVT VT = Op.getValueType();
  Result = DAG.getNode(T8xxISD::LOAD_SYM, SDLoc(Op), VT, Result);

  // Add instruction to add instruction pointer to relative address
  Result = DAG.getNode(T8xxISD::ADD_IPTR,
		       SDLoc(Op), VT, Result);

  return Result;
}


SDValue T8xxTargetLowering::LowerJumpTable(SDValue Op, SelectionDAG& DAG) const
{
  SDValue Result;
  JumpTableSDNode *CP = cast<JumpTableSDNode>(Op.getNode());

  // TODO: Just a first try to see how things work.
  // Ideally a later version should be able to build position independent code as well
  // as code for a fixed address.
  Result = DAG.getTargetJumpTable(CP->getIndex(), CP->getValueType(0), T8xxII::MO_PCREL_SYM);

  EVT VT = getPointerTy(DAG.getDataLayout ());
  Result = DAG.getNode(T8xxISD::LOAD_SYM,
		       SDLoc(Op), VT, Result);

  // Add instruction to add instruction pointer to relative address
  Result = DAG.getNode(T8xxISD::ADD_IPTR,
		       SDLoc(Op), VT, Result);

  return Result;
}


SDValue T8xxTargetLowering::LowerBlockAddress(SDValue Op, SelectionDAG& DAG) const
{
  SDValue Result;
  BlockAddressSDNode *CP = cast<BlockAddressSDNode>(Op.getNode());

  // TODO: Just a first try to see how things work.
  // Ideally a later version should be able to build position independent code as well
  // as code for a fixed address.
  Result = DAG.getTargetBlockAddress(CP->getBlockAddress(), CP->getValueType(0),
				     CP->getOffset (), T8xxII::MO_GLOBAL);

  EVT VT = Op.getValueType();
  Result = DAG.getNode(T8xxISD::LOAD_SYM, SDLoc(Op), VT, Result);

  return Result;
}


// This function creates nodes to replicate a select function
// in the DAG

MachineBasicBlock *
T8xxTargetLowering::EmitLoweredSelect(MachineInstr &MI,
                                      MachineBasicBlock *MBB) const {
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  LLVM_DEBUG({
      dbgs() << "EmitLoweredSelect\n";
      MI.dump ();
    });

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
  //              Iptr' = NextInst

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




// This function creates nodes to replicate a select function
// in the DAG

MachineBasicBlock *
T8xxTargetLowering::EmitLoweredFPSetCC(MachineInstr &MI,
				       MachineBasicBlock *MBB) const {
  const TargetInstrInfo *TII = Subtarget.getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  LLVM_DEBUG({
      dbgs() << "EmitLoweredFPSetCC\n";
      MI.dump ();
    });

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
  MachineFunction *F = MBB->getParent();

  MachineBasicBlock *Copy0MBB = F->CreateMachineBasicBlock(BB);
  MachineBasicBlock *SinkMBB = F->CreateMachineBasicBlock(BB);
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
  //              Iptr' = NextInst

  // Create the conditional branch instruction.
  MachineRegisterInfo &MRI = F->getRegInfo();
  // In Thumb mode S must not be specified if source register is the SP or
  // PC and if destination register is the SP, so restrict register class
  Register IsOrderedReg = MRI.createVirtualRegister(&T8xx::ORegRegClass);
  Register Op1Reg = MRI.cloneVirtualRegister(MI.getOperand(1).getReg());
  Register Op2Reg = MRI.cloneVirtualRegister(MI.getOperand(2).getReg());

  BuildMI(MBB, DL, TII->get(T8xx::FPORDEREDSN),IsOrderedReg)
    .addDef(Op1Reg)
    .addDef(Op2Reg)
    .addReg(MI.getOperand(1).getReg())
    .addReg(MI.getOperand(2).getReg());
  BuildMI(MBB, DL, TII->get(T8xx::CJ)).addReg(IsOrderedReg).addMBB(SinkMBB);

  //  Copy0MBB:
  //   %FalseValue = ...
  //   # fallthrough to SinkMBB
  Copy0MBB->addSuccessor(SinkMBB);

  // At this place, it has been established, that the FP registers are ordered!
  Register CondReg = MRI.createVirtualRegister(&T8xx::ORegRegClass);
  BuildMI(Copy0MBB, DL, TII->get(T8xx::FPGTSN),CondReg)
    .addReg(Op1Reg)
    .addReg(Op2Reg);

  //  SinkMBB:
  //   %Result = phi [ %FalseValue, Copy0MBB ], [ %TrueValue, ThisMBB ]
  //  ...
  MachineBasicBlock::iterator MIItBegin = MachineBasicBlock::iterator(MI);
  MachineBasicBlock::iterator MIItEnd =
      std::next(MachineBasicBlock::iterator(MI));
  MachineBasicBlock::iterator SinkInsertionPoint = SinkMBB->begin();

  Register DestReg = MI.getOperand(0).getReg();
  MachineInstrBuilder MIB;

  MIB =
    BuildMI(*SinkMBB, SinkInsertionPoint, DL, TII->get(T8xx::PHI), DestReg)
    .addReg(CondReg)
    .addMBB(Copy0MBB)
    .addReg(IsOrderedReg)
    .addMBB(ThisMBB);

  // Now remove the pseudo code instruction
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
  case T8xx::CMOV32:
    return EmitLoweredSelect(MI, MBB);
  }

}

//
// Jump Table related functions.
// The default functionality of LLVM selects a proper JumpTable Encoding
// based on code model and whether position independent code was
// requested.
// As a quick fix, the T8xx method always requests EK_BlockAddress.
//
// A proper solution should create the correct arithmetic entries,
// which allow position independent code. So far the arithemtic
// entries are created, but not properly treated during code
// relaxation in the linking process.
//
// Note: Does not work properly. Fixes the jump table itself.
// However, the conditional jump still assumes position indepence
// and adopts a double dereferencing!


unsigned T8xxTargetLowering::getJumpTableEncoding() const {
  return MachineJumpTableInfo::EK_BlockAddress;
}

// Relative jump tables are not support, yet!
bool T8xxTargetLowering::isJumpTableRelative() const
{
  return false;
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

  MachineFunction &MF = DAG.getMachineFunction();
  T8xxMachineFunctionInfo *TFI = MF.getInfo<T8xxMachineFunctionInfo>();

  SDLoc &Loc = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins = CLI.Ins;
  SDValue Chain = CLI.Chain;
  SDValue Callee = CLI.Callee;
  CallingConv::ID CallConv = CLI.CallConv;
  const bool isVarArg = CLI.IsVarArg;

  CLI.IsTailCall = false;

  LLVM_DEBUG(dbgs() << "LowerCall\n");

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());
  CCInfo.AnalyzeCallOperands(Outs, CC_T8xx32);

  // Get the size of the outgoing arguments stack space requirement.
  unsigned NumBytes = CCInfo.getStackSize();

  // Create node for CALLSEQ_START
  Chain = DAG.getCALLSEQ_START(Chain, NumBytes, 0, Loc);

  SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;

  LLVM_DEBUG(dbgs() << "ArgLocs " << ArgLocs.size() << "\n");

  LLVM_DEBUG({
      dbgs() << "Before ArgLocs\n";
      DAG.dump ();
    });

  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    SDValue Arg = OutVals[i];

    // We only handle fully promoted arguments.
    assert(VA.getLocInfo() == CCValAssign::Full && "Unhandled loc info");

    if (VA.isRegLoc()) {
      LLVM_DEBUG(dbgs() << "VA " << i << " is Reg\n");

      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
      continue;
    }

    LLVM_DEBUG(dbgs() << "VA " << i << " is Mem\n");

    assert(VA.isMemLoc() &&
           "Only support passing arguments through registers or via the stack");

    // Since the "registers" are actually on the stack, at this
    // point it is not feasible to adjust the framepointer.
    // Negative are used for function parameters that should
    // be put on the stack
    assert (VA.getLocMemOffset() % 4 == 0 &&
	    "Only 4 byte aligned offset allowed");

    // Floating point values are handled via a store instruction
    // For integers, use a short cut
    if (Arg.getValueType ().isFloatingPoint ())
      {
	// The conversion of a byte offset to an index is carried out later in expandPostRAPPseude
	int WPtrOff = (-NumBytes);
	WPtrOff += VA.getLocMemOffset();
	SDValue Off = DAG.getSignedConstant(WPtrOff, Loc,
					    getPointerTy(DAG.getDataLayout()));

	SDValue StackPtr = DAG.getRegister(T8xx::WPTR, MVT::i32);
	Off = DAG.getNode(T8xxISD::ADD_WPTR, Loc, MVT::i32, StackPtr, Off);
	MemOpChains.push_back(DAG.getStore(Chain, Loc, Arg, Off,
					   MachinePointerInfo()));
      }
    else
      {
	int WPtrOff = (-NumBytes);
	WPtrOff += VA.getLocMemOffset();
	SDValue Off = DAG.getSignedConstant(WPtrOff / 4, Loc,
					getPointerTy(DAG.getDataLayout()));

	SDVTList VTs = DAG.getVTList(MVT::Other, MVT::Glue);
	SDValue Ops[] = {Chain, Arg, Off};
	MemOpChains.push_back (DAG.getNode(T8xxISD::STL_PARM, Loc, VTs, Ops));
      }
  }

  // Emit all stores, make sure they occur before the call.
  if (!MemOpChains.empty()) {
    Chain = DAG.getNode(ISD::TokenFactor, Loc, MVT::Other, MemOpChains);
  }

  LLVM_DEBUG({
      dbgs() << "Before RegsToPass\n";
      DAG.dump ();
    });

  // Build a sequence of copy-to-reg nodes chained together with token chain
  // and flag operands which copy the outgoing args into the appropriate regs.
  SDValue InFlag;
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Chain = DAG.getCopyToReg(Chain, Loc, RegsToPass[i].first,
			     RegsToPass[i].second, InFlag);
    InFlag = Chain.getValue(1);
  }

  LLVM_DEBUG({
      dbgs() << "After RegsToPass\n";
      DAG.dump ();
    });

  // We only support calling global addresses.
  /* Original code
  GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee);
  assert(G && "We only support the calling of global addresses");

  EVT PtrVT = getPointerTy(DAG.getDataLayout());
  Callee = DAG.getGlobalAddress(G->getGlobal(), Loc, PtrVT, 0);
  */

  // For variable function arguments add a stack adjustment
  // This will take care of variable argument stacks
  if (isVarArg)
    {
      int NBytes = (int)(NumBytes >> 2);
      LLVM_DEBUG(dbgs() << "Var Arg Stacksize " << NBytes);
      LLVM_DEBUG(dbgs() << "ArgLocs  " << ArgLocs.size () << "\n");

      /* Old version, did work for integer
      SDValue Off2 = DAG.getSignedConstant(-(ArgLocs.size() + 1), Loc,
      getPointerTy(DAG.getDataLayout())); */
      SDValue Off2 = DAG.getSignedConstant(-(NBytes + 1), Loc,
					   getPointerTy(DAG.getDataLayout()));
      SDVTList VTs2 = DAG.getVTList(MVT::Other);
      SDValue Ops2[] = {Chain, Off2};
      Chain = DAG.getNode(T8xxISD::AJW, Loc, VTs2, Ops2);
    }

  // This works with a call instruction that directly takes
  // the address as parameter
  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee))
    {
      LLVM_DEBUG(dbgs() << "Lower Call: GlobalAddressSDNode\n");
      Callee = DAG.getTargetGlobalAddress(G->getGlobal(), Loc, MVT::i32, 0, T8xxII::MO_IPTRREL);
    }
  else if (ExternalSymbolSDNode *E = dyn_cast<ExternalSymbolSDNode>(Callee))
    {
      LLVM_DEBUG(dbgs() << "Lower Call: ExternalSymbolSDNode\n");
      Callee = DAG.getTargetExternalSymbol(E->getSymbol(), MVT::i32, T8xxII::MO_IPTRREL);
    }

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

  // For variable parameter calls, reset the stack adjustment after return from the call
  if (isVarArg)
    {
      InFlag = Chain.getValue(1);

      SDValue Off3 = DAG.getSignedConstant(((NumBytes >> 2) + 1), Loc,
					   getPointerTy(DAG.getDataLayout()));
	/*
      SDValue Off3 = DAG.getSignedConstant(ArgLocs.size() + 1, Loc,
					   getPointerTy(DAG.getDataLayout()));
	*/
      SDVTList VTs3 = DAG.getVTList(MVT::Other, MVT::Glue);
      SDValue Ops3[] = {Chain, Off3, InFlag};
      Chain = DAG.getNode(T8xxISD::AJW, Loc, VTs3, Ops3);
    }

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

  /*
  assert(!isVarArg && "Unsupported");
  */

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

  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());
  CCInfo.AnalyzeFormalArguments(Ins, CC_T8xx32);

  int i = 0;

  // From RISCV: CCValAssign &VA = ArgLocs

  for (auto &VA : ArgLocs) {
    if (VA.isRegLoc()) {
      LLVM_DEBUG(dbgs() << "VA " << i++ << " is Reg\n");
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
    LLVM_DEBUG(dbgs() << "VA " << i++ << " is Mem\n");

    assert(VA.isMemLoc() &&
           "Can only pass arguments as either registers or via the stack");

    const unsigned Offset = VA.getLocMemOffset();

    const uint64_t SizeInBits = VA.getValVT().getSizeInBits();

    const int FI = MF.getFrameInfo().CreateFixedObject(SizeInBits / 8, Offset, true);
    EVT PtrTy = getPointerTy(DAG.getDataLayout());
    SDValue FIPtr = DAG.getFrameIndex(FI, PtrTy);
    SDValue Load = DAG.getLoad(VA.getValVT(), DL, Chain, FIPtr,
                               MachinePointerInfo());
    InVals.push_back(Load);
  }

  // Deal with variable arguments
  if (IsVarArg)
    {
      MachineFrameInfo &MFI = MF.getFrameInfo();
      T8xxMachineFunctionInfo *TFI = MF.getInfo<T8xxMachineFunctionInfo>();
      int FI;

      // Needs to happen through the stack
      int VaArgOffset = CCInfo.getStackSize();
      //      FI = MFI.CreateFixedObject(XLenInBytes, VaArgOffset, true);
      FI = MFI.CreateFixedObject(4, VaArgOffset, true);

      // Record the frame index of the first variable argument
      // which is a value necessary to VASTART.
      TFI->setVarArgsFrameIndex(FI);

      // TODO: Just use this temporary to convey the information that varargs
      // are used.
      TFI->setVarArgsSaveSize(4);
    }

  return Chain;
}


//===----------------------------------------------------------------------===//
//               Return Value Calling Convention Implementation
//===----------------------------------------------------------------------===//

bool T8xxTargetLowering::CanLowerReturn(
    CallingConv::ID CallConv, MachineFunction &MF, bool isVarArg,
    const SmallVectorImpl<ISD::OutputArg> &Outs,
    LLVMContext &Context, const Type *RetTy) const {
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
  LLVM_DEBUG({
      dbgs() << "LowerFormalReturn\n";
      DAG.dump ();
      dbgs() << "Pre LowerFormalReturn\n";
    });

  // CCValAssign - represent the assignment of the return value to locations.
  SmallVector<CCValAssign, 16> RVLocs;

  // CCState - Info about the registers and stack slot.
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  // Analyze return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_T8xx32);

  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  LLVM_DEBUG(dbgs() << "Temp A, RVLocs Size " << RVLocs.size() << "\n");

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

  LLVM_DEBUG({
      dbgs() << "Post Lower Return\n";
      DAG.dump ();
    });

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
