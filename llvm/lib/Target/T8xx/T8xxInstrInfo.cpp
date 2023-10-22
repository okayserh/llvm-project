//===-- T8xxInstrInfo.cpp - T8xx Instruction Information ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the T8xx implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "T8xxInstrInfo.h"
#include "T8xx.h"
#include "MCTargetDesc/T8xxMCExpr.h"
#include "T8xxMachineFunctionInfo.h"
#include "T8xxSubtarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define GET_INSTRINFO_CTOR_DTOR
#include "T8xxGenInstrInfo.inc"

// Pin the vtable to this file.
void T8xxInstrInfo::anchor() {}

T8xxInstrInfo::T8xxInstrInfo(T8xxSubtarget &ST)
    : T8xxGenInstrInfo(T8xx::ADJCALLSTACKDOWN, T8xx::ADJCALLSTACKUP), RI(),
      Subtarget(ST) {}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned T8xxInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  if (MI.getOpcode() == T8xx::LDL) {
    if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }
  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned T8xxInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                            int &FrameIndex) const {
  if (MI.getOpcode() == T8xx::STL) {
    if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
        MI.getOperand(1).getImm() == 0) {
      FrameIndex = MI.getOperand(0).getIndex();
      return MI.getOperand(2).getReg();
    }
  }
  return 0;
}


//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//
//
/// AnalyzeBranch - Analyze the branching code at the end of MBB, returning
/// true if it cannot be understood (e.g. it's a switch dispatch or isn't
/// implemented for a target).  Upon success, this returns false and returns
/// with the following information in various cases:
///
/// 1. If this block ends with no branches (it just falls through to its succ)
///    just return false, leaving TBB/FBB null.
/// 2. If this block ends with only an unconditional branch, it sets TBB to be
///    the destination block.
/// 3. If this block ends with an conditional branch and it falls through to
///    an successor block, it sets TBB to be the branch destination block and a
///    list of operands that evaluate the condition. These
///    operands can be passed to other TargetInstrInfo methods to create new
///    branches.
/// 4. If this block ends with an conditional branch and an unconditional
///    block, it returns the 'true' destination in TBB, the 'false' destination
///    in FBB, and a list of operands that evaluate the condition. These
///    operands can be passed to other TargetInstrInfo methods to create new
///    branches.
///
/// Note that RemoveBranch and InsertBranch must be implemented to support
/// cases where this method returns success.
///
bool
T8xxInstrInfo::analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                            MachineBasicBlock *&FBB,
                            SmallVectorImpl<MachineOperand> &Cond,
                            bool AllowModify) const {
  bool HasCondBranch = false;
  TBB = nullptr;
  FBB = nullptr;

  printf ("T8xx::analyzeBranch\n");

  for (MachineInstr &MI : MBB) {
    if (MI.getOpcode() == T8xx::BRimm2) {
      MachineBasicBlock *TargetBB = MI.getOperand(0).getMBB();
      if (HasCondBranch) {
        FBB = TargetBB;
      } else {
        TBB = TargetBB;
      }
    } else if (MI.getOpcode() == T8xx::Bcc) {
      MachineBasicBlock *TargetBB = MI.getOperand(0).getMBB();
      TBB = TargetBB;
      Cond.push_back(MI.getOperand(0));
      HasCondBranch = true;
    }
  }
  return false;
}

/// RemoveBranch - Remove the branching code at the end of the specific MBB.
/// This is only invoked in cases where AnalyzeBranch returns success. It
/// returns the number of instructions that were removed.
unsigned
T8xxInstrInfo::removeBranch(MachineBasicBlock &MBB,
			   int *BytesRemoved) const {
  printf ("T8xx::removeBranch\n");

  if (MBB.empty())
    return 0;
  unsigned NumRemoved = 0;
  auto I = MBB.end();
  do {
    --I;
    unsigned Opc = I->getOpcode();
    if ((Opc == T8xx::BRimm2) || (Opc == T8xx::Bcc)) {
      auto ToDelete = I;
      ++I;
      MBB.erase(ToDelete);
      NumRemoved++;
    }
  } while (I != MBB.begin());
  return NumRemoved;
}

/// InsertBranch - Insert branch code into the end of the specified
/// MachineBasicBlock.  The operands to this method are the same as those
/// returned by AnalyzeBranch.  This is only invoked in cases where
/// AnalyzeBranch returns success. It returns the number of instructions
/// inserted.
///
/// It is also invoked by tail merging to add unconditional branches in
/// cases where AnalyzeBranch doesn't apply because there was no original
/// branch to analyze.  At least this much must be implemented, else tail
/// merging needs to be disabled.
unsigned T8xxInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL,
				    int *BytesAdded) const {
  unsigned NumInserted = 0;
  printf ("T8xx::insertBranch\n");

  // Insert any conditional branch.
  // TODO: Quick fix. Need to figure right way to do this
  if (!Cond.empty ())
    {
      //    BuildMI(MBB, MBB.end(), DL, get(T8xx::Bcc)).addImm(Cond[0].getImm()).addMBB(TBB);
      BuildMI(MBB, MBB.end(), DL, get(T8xx::Bcc)).addMBB(TBB);
      NumInserted++;
    }
  else
    {
      // Insert any unconditional branch.
      if (Cond.empty() || FBB) {
	BuildMI(MBB, MBB.end(), DL, get(T8xx::BRimm2)).addMBB(Cond.empty() ? TBB : FBB);
	NumInserted++;
      }
    }
  return NumInserted;
}

// ----

void T8xxInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 const DebugLoc &DL, MCRegister DestReg,
                                 MCRegister SrcReg, bool KillSrc) const {
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  uint16_t hweSrcReg = TRI->getEncodingValue (SrcReg);
  uint16_t hweDstReg = TRI->getEncodingValue (DestReg);

  /* 16 Register in Workspace approach */
  if ((SrcReg >= T8xx::R0) and (SrcReg <= T8xx::R15))
    BuildMI(MBB, I, DL, get(T8xx::LDL)).addImm(hweSrcReg);

  if ((DestReg >= T8xx::R0) and (DestReg <= T8xx::R15))
    BuildMI(MBB, I, DL, get(T8xx::STL)).addImm(hweDstReg);
}

void T8xxInstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                    Register SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI,
		    Register VReg) const {
  printf ("Store %u into %u\n", SrcReg, VReg);
  if (SrcReg.isVirtual ())
    printf ("Src Virt\n");
  if (VReg.isVirtual ())
    printf ("VReg Virt\n");

  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addReg(SrcReg, getKillRegState(true))
    .addFrameIndex(FI).addImm(0);

  /* TODO: See how to deal with these cases.
  uint16_t hweSrcReg = TRI->getEncodingValue (SrcReg.asMCReg());

  // With Stack relative to WPTR
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL)).addImm(hweSrcReg);
  if (FI % 4 == 0)
    BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addImm(FI / 4);
  else
    printf ("Error: storeRegToStackSlot, unaligned frame access\n");
  */
}

void T8xxInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     Register DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI,
		     Register VReg) const {
  printf ("Load %u from %u\n", DestReg, VReg);
  if (DestReg.isVirtual ())
    printf ("Dest Virt\n");
  if (VReg.isVirtual ())
    printf ("VReg Virt\n");

  if (RC == &T8xx::ORegRegClass)
    BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL), DestReg).addFrameIndex(FI).addImm(0);
  else
    llvm_unreachable("Can't load this register from stack slot");
  /*
  uint16_t hweDestReg = TRI->getEncodingValue (DestReg.asMCReg());

  // With Stack relative to WPTR
  if (FI % 4 == 0)
    BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL)).addImm(FI / 4);
  else
    printf ("Error: loadRegFromStackSlot, unaligned frame access\n");
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addImm(hweDestReg);
  */
}


/*
 * Load the operand to the processor register stack
 */

void T8xxInstrInfo::loadRegStack (MachineInstr &MI, const unsigned int OpNum, const int OReg) const
{
  DebugLoc DL = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  const MachineOperand::MachineOperandType MOT = MI.getOperand(OpNum).getType ();  // X
  printf ("loadRegStack TYPE: %i\n", (int) MOT);
  switch (MOT)
    {
    case MachineOperand::MO_Register:
      {
	MCRegister SrcReg = MI.getOperand(OpNum).getReg ();
	if (SrcReg == T8xx::WPTR)
	  BuildMI(MBB, MI, DL, get(T8xx::LDLP)).addImm(OReg);
	if ((SrcReg >= T8xx::R0) and (SrcReg <= T8xx::R15))
	  BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue(MI.getOperand(OpNum).getReg().asMCReg()));
      }
      break;
    case MachineOperand::MO_Immediate:
      BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(MI.getOperand(OpNum).getImm());
      break;
    case MachineOperand::MO_ExternalSymbol:
      BuildMI(MBB, MI, DL, get(T8xx::LDC)).addSym(MI.getOperand(OpNum).getMCSymbol());
      break;
    case MachineOperand::MO_GlobalAddress:
      BuildMI(MBB, MI, DL, get(T8xx::LDC)).addGlobalAddress(MI.getOperand(OpNum).getGlobal());
      break;
    }
}


void T8xxInstrInfo::addAddrOffset (MachineInstr &MI, const unsigned int OpNum) const
{
  DebugLoc DL = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  const MachineOperand::MachineOperandType MOT = MI.getOperand(OpNum).getType ();  // X
  printf ("addAddrOffset TYPE: %i\n", (int) MOT);
  switch (MOT)
    {
    case MachineOperand::MO_Register:
      loadRegStack (MI, OpNum);
      BuildMI(MBB, MI, DL, get(T8xx::ADD));
      break;
    case MachineOperand::MO_Immediate:
      if (MI.getOperand(OpNum).getImm() != 0)
	BuildMI(MBB, MI, DL, get(T8xx::ADC)).addImm(MI.getOperand(OpNum).getImm());
      break;
    }
}


void T8xxInstrInfo::storeRegStack (MachineInstr &MI, const unsigned int OpNum,
				   const bool InsertPostMI) const
{
  DebugLoc DL = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  const MachineOperand::MachineOperandType MOT = MI.getOperand(OpNum).getType ();  // X
  printf ("storeRegStack TYPE: %i\n", (int) MOT);
  switch (MOT)
    {
    case MachineOperand::MO_Register:
      {
	MachineBasicBlock::iterator MBBI = MI;
	if (InsertPostMI)
	  ++MBBI;
	BuildMI(MBB, MBBI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue(MI.getOperand(OpNum).getReg().asMCReg()));
      }
      break;

    default:
      printf ("Failed in storeRegStack! Wrong destination operand type\n");
      break;
    }
}



void T8xxInstrInfo::createComparison(MachineInstr &MI, const unsigned int OpX, const unsigned int OpY,
				     const bool negate, const bool diff) const
{
  DebugLoc DL = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  /*  loadRegStack (MI, OpX);
      loadRegStack (MI, OpY);*/
  if (diff)
    BuildMI(MBB, MI, DL, get(T8xx::DIFF), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG); // Difference
  if (diff && negate)
    BuildMI(MBB, MI, DL, get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0); // logical NOT
  BuildMI(MBB, MI, DL, get(T8xx::GT), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);
  if (negate && (~diff))
    BuildMI(MBB, MI, DL, get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0); // logical NOT
}


bool T8xxInstrInfo::expandPostRAPseudo(MachineInstr &MI) const
{
  printf ("expandPostRAPseudo %i %i\n", MI.getOpcode (), T8xx::LDC);

  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();
  DebugLoc DL = MI.getDebugLoc();

  switch (MI.getOpcode())
  {
  default:
    return false;

    // This is based on section 5.6.3 in the T8xx compiler writers guide
  case T8xx::CMP:
    {
      // Destination register (outs!?)
      const ISD::CondCode CC = (ISD::CondCode)MI.getOperand(0).getImm();
      const MachineOperand::MachineOperandType MOT_LHS = MI.getOperand(1).getType ();  // X
      const MachineOperand::MachineOperandType MOT_RHS = MI.getOperand(2).getType ();  // Y

      /* With stack relative to WPTR */
      switch (CC)
	{
	case ISD::SETLT:
	  createComparison(MI, 2, 1, true, false);
	  MBB.erase (MI);
	  break;
	case ISD::SETLE:
	  createComparison(MI, 1, 2, false, false);
	  MBB.erase (MI);
	  break;

	case ISD::SETGT:
	  createComparison(MI, 1, 2, true, false);
	  MBB.erase (MI);
	  break;
	case ISD::SETGE:
	  createComparison(MI, 2, 1, false, false);
	  MBB.erase (MI);
	  break;

	case ISD::SETEQ:
	  if (MOT_LHS == MachineOperand::MO_Immediate)
	    {
	      loadRegStack (MI, 2);
	      BuildMI(MBB, MI, DL, get(T8xx::EQC)).addImm(MI.getOperand(1).getImm());
	    }
	  else
	    {
	      if (MOT_RHS == MachineOperand::MO_Immediate)
		{
		  loadRegStack (MI, 1);
		  BuildMI(MBB, MI, DL, get(T8xx::EQC)).addImm(MI.getOperand(2).getImm());
		}
	      else
		{
		  loadRegStack (MI, 1);
		  loadRegStack (MI, 2);
		  BuildMI(MBB, MI, DL, get(T8xx::DIFF)); // Difference
		}
	    }
	  MBB.erase(MI);
	  break;

	case ISD::SETNE:
	  if (MOT_LHS == MachineOperand::MO_Immediate)
	    {
	      loadRegStack (MI, 2);
	      BuildMI(MBB, MI, DL, get(T8xx::EQC)).addImm(MI.getOperand(1).getImm());
	    }
	  else
	    {
	      if (MOT_RHS == MachineOperand::MO_Immediate)
		{
		  loadRegStack (MI, 1);
		  BuildMI(MBB, MI, DL, get(T8xx::EQC)).addImm(MI.getOperand(2).getImm());
		}
	      else
		{
		  loadRegStack (MI, 1);
		  loadRegStack (MI, 2);
		  BuildMI(MBB, MI, DL, get(T8xx::DIFF)); // Difference
		}
	    }
	  BuildMI(MBB, MI, DL, get(T8xx::EQC)).addImm(0); // logical NOT
	  MBB.erase(MI);
	  break;
	}

      return true;
    }
    break;

    // This is a special instruction to introduce a way to get effective addresses
    // that are not aligned
  case T8xx::AddWptrImm:
  case T8xx::LDLPb:
    {
      int64_t rem = MI.getOperand(2).getImm () % 4;
      MCRegister DstReg = MI.getOperand(0).getReg ();
      MCRegister SrcReg = MI.getOperand(1).getReg ();
      BuildMI (MBB, MI, DL, get(T8xx::LDLP), DstReg).addReg(SrcReg).addImm(MI.getOperand(2).getImm () - rem);
      if (rem != 0)
	{
	  MI.getOperand(2).setImm (MI.getOperand(2).getImm() - rem);
	  BuildMI (MBB, MI, DL, get(T8xx::ADC), T8xx::AREG).addReg(T8xx::AREG).addImm(rem);
	}
      MBB.erase(MI);
    }
    break;

  case T8xx::RET:
    {
      BuildMI (MBB, MI, DL, get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, MI, DL, get(T8xx::GCALL)).addReg(T8xx::AREG);
      MBB.erase(MI);
    }
    break;

  case T8xx::CALL:
    {
      for (unsigned int i = 0; i < MI.getNumOperands (); ++i)
	{
	  printf ("CALL Op%i %i\n", i, MI.getOperand (i).getType ());
	  MI.getOperand (i).dump ();
	}

      // First OP is MO_GlobalAddress
      // Second OP is MO_RegisterMask
      // Third and Fourth are MO_Register

      // Load offset to global address into AREG and correct by bytecount of LDPI and GCALL
      BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addGlobalAddress(MI.getOperand(0).getGlobal (), 0, T8xxMCExpr::VK_T8xx_GLOBAL);
      BuildMI (MBB, MI, DL, get(T8xx::ADC), T8xx::AREG).addReg(T8xx::AREG).addImm(-4);
      BuildMI (MBB, MI, DL, get(T8xx::LDPI), T8xx::AREG).addReg(T8xx::AREG);
      BuildMI (MBB, MI, DL, get(T8xx::GCALL)).addReg(T8xx::AREG);
      BuildMI (MBB, MI, DL, get(T8xx::REV));
      MBB.erase(MI);
      
      /*
      BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, MI, DL, get(T8xx::GCALL)).addReg(T8xx::AREG);
      MBB.erase(MI);
      */
    }
    break;


  }
}
