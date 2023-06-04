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
  if (MI.getOpcode() == T8xx::LDRi32regop || MI.getOpcode() == T8xx::LDRi8regop ||
      MI.getOpcode() == T8xx::LDRzi8regop || MI.getOpcode() == T8xx::LDRsi8regop) {
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
  if (MI.getOpcode() == T8xx::STRi32regop || MI.getOpcode() == T8xx::STRi32immop/* ||
								      MI.getOpcode() == T8xx::STRi16*/) {
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
  uint16_t hweSrcReg = TRI->getEncodingValue (SrcReg.asMCReg());

  /* With Stack relative to WPTR */
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL)).addImm(hweSrcReg);
  if (FI % 4 == 0)
    BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addImm(FI / 4);
  else
    printf ("Error: storeRegToStackSlot, unaligned frame access\n");
}

void T8xxInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     Register DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI,
		     Register VReg) const {
  uint16_t hweDestReg = TRI->getEncodingValue (DestReg.asMCReg());

  /* With Stack relative to WPTR */
  if (FI % 4 == 0)
    BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL)).addImm(FI / 4);
  else
    printf ("Error: loadRegFromStackSlot, unaligned frame access\n");
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addImm(hweDestReg);
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

  loadRegStack (MI, OpX);
  loadRegStack (MI, OpY);
  if (diff)
    BuildMI(MBB, MI, DL, get(T8xx::DIFF)); // Difference
  if (diff && negate)
    BuildMI(MBB, MI, DL, get(T8xx::EQC)).addImm(0); // logical NOT
  BuildMI(MBB, MI, DL, get(T8xx::GT));
  if (negate && (~diff))
    BuildMI(MBB, MI, DL, get(T8xx::EQC)).addImm(0); // logical NOT
  MBB.erase(MI);
}


bool T8xxInstrInfo::expandPostRAPseudo(MachineInstr &MI) const
{
  printf ("expandPostRAPseudo %i %i\n", MI.getOpcode (), T8xx::MOVimmr);

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
	  break;
	case ISD::SETLE:
	  createComparison(MI, 1, 2, false, false);
	  break;

	case ISD::SETGT:
	  createComparison(MI, 1, 2, true, false);
	  break;
	case ISD::SETGE:
	  createComparison(MI, 2, 1, false, false);
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
		  createComparison(MI, 1, 2, true, true);
		}
	    }
	  BuildMI(MBB, MI, DL, get(T8xx::EQC)).addImm(0); // logical NOT
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
		  createComparison(MI, 1, 2, true, true);
		}
	    }
	  MBB.erase(MI);
	  break;
	}

      return true;
    }
    break;

  case T8xx::LoadOpStack:
    {
      loadRegStack (MI, 1);
      return true;
    }
    break;


    // Return instruction
    /*
  case T8xx::RET:
    {
      for (unsigned int i = 0; i < MI.getNumOperands (); ++i)
	{
	  loadRegStack (MI, i);
	  //	  printf ("RET %i Type %i\n", i, MI.getOperand(i).getType ());
	  //	  const ISD::CondCode CC = (ISD::CondCode)MI.getOperand(0).getImm();
	 //	  const MachineOperand::MachineOperandType MOT_LHS = ;  // X
	}
      return false;
    }
    break;
*/      
   }
}
