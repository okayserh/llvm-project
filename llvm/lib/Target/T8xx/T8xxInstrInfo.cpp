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
  if (MI.getOpcode() == T8xx::LDRi || MI.getOpcode() == T8xx::LDRi8 ||
      MI.getOpcode() == T8xx::LDRzi8 || MI.getOpcode() == T8xx::LDRsi8) {
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
  if (MI.getOpcode() == T8xx::STRi || MI.getOpcode() == T8xx::STRi8/* ||
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
  BuildMI(MBB, I, DL, get(T8xx::LDL)).addImm(hweSrcReg);
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

void T8xxInstrInfo::loadRegStack (MachineInstr &MI, const unsigned int OpNum, const unsigned int OReg) const
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
      if (MI.getOperand(OpNum).getReg() == T8xx::WPTR)
	BuildMI(MBB, MI, DL, get(T8xx::LDLP)).addImm(OReg);
      else
	BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue(MI.getOperand(OpNum).getReg().asMCReg()));
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

  case T8xx::LEA_ADDri:
    {
      const unsigned FI = MI.getOperand(2).getImm();
      if ((MI.getOperand(1).getReg() == T8xx::WPTR) && (FI % 4 == 0))
	loadRegStack (MI, 1, FI / 4);
      else
	{
	  loadRegStack (MI, 1);
	  addAddrOffset (MI, 2);
	}

      storeRegStack (MI, 0);

      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::STRi:
  case T8xx::STRi16:
    {
      // Destination register (outs!?)
      const Register AddBaseReg = MI.getOperand(1).getReg();  // TODO: Clarify whether that's always the frameindex?
      const unsigned FI = MI.getOperand(2).getImm();

      /* With stack relative to WPTR */
      loadRegStack (MI, 0);

      // Truncation to lower 16 bits via "and"
      if (MI.getOpcode() == T8xx::STRi16)
	{
	  BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(0xFFFF);
	  BuildMI(MBB, MI, DL, get(T8xx::AND));
	}

      // For 16 bit values also a 32 bit alignment needs to satisfied
      if (AddBaseReg == T8xx::WPTR)
	if (FI % 4 == 0)
	  BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(FI / 4);  // Stack Offset goes via OREG
	else
	  printf ("STRi unaligned frame index %i\n", FI);
      else
	printf ("STRi wrong register in operand type\n");
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::STRi8:
  case T8xx::STRimm8:
    {
      const unsigned FI = MI.getOperand(2).getImm();
      if ((MI.getOperand(1).getReg() == T8xx::WPTR) && (FI % 4 == 0))
	loadRegStack (MI, 1, FI / 4);
      else
	{
	  loadRegStack (MI, 1);
	  addAddrOffset (MI, 2);
	}

      loadRegStack (MI, 0);
      BuildMI(MBB, MI, DL, get(T8xx::SB));
      // TODO: For the sext / zext cases add some for sign extension or zero extension

      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::LDRi:
  case T8xx::LDRi16:
  case T8xx::LDRzi16:
  case T8xx::LDRsi16:
    {
      // Destination register (outs!?)
      const Register DstReg = MI.getOperand(0).getReg();
      const unsigned AddBaseReg = MI.getOperand(1).getReg();
      const unsigned FI = MI.getOperand(2).getImm();

      // BuildMI inserts before "MI"
      if (AddBaseReg == T8xx::WPTR)
	if (FI % 4 == 0)
	  BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(FI / 4);
	else
	  printf ("LDRi unaligned frame index %i\n", FI);
      else
	printf ("LDRi wrong register in operand type\n");

      // Sign extension
      if (MI.getOpcode() == T8xx::LDRsi16)
	{
	  BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(0x8000);
	  BuildMI(MBB, MI, DL, get(T8xx::XWORD));
	}

      BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue(DstReg.asMCReg()));
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::LDRi8:
  case T8xx::LDRsi8:
  case T8xx::LDRzi8:
    {
      const unsigned FI = MI.getOperand(2).getImm();
      if ((MI.getOperand(1).getReg() == T8xx::WPTR) && (FI % 4 == 0))
	loadRegStack (MI, 1, FI / 4);
      else
	{
	  loadRegStack (MI, 1);
	  addAddrOffset (MI, 2);
	}

      // SB Takes an address in StackReg A. Hence the code above pulls the address into A
      BuildMI(MBB, MI, DL, get(T8xx::LB));
      // TODO: For the sext / zext cases add some for sign extension or zero extension
      if (MI.getOpcode() == T8xx::LDRsi8)
	{
	  BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(0x80);
	  BuildMI(MBB, MI, DL, get(T8xx::XWORD));
	}
      
      storeRegStack (MI, 0);

      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::MOVimmr:
    {
      // Destination register (outs!?)
      const Register DstReg = MI.getOperand(0).getReg();

      // BuildMI inserts before "MI"
      loadRegStack (MI,1);
      BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue(DstReg.asMCReg()));  // Stack Offset goes via OREG
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::ADDimmr:
    {
      // Destination register (outs!?)
      const Register DstReg = MI.getOperand(0).getReg();
      const Register SrcReg1 = MI.getOperand(1).getReg();

      // BuildMI inserts before "MI"
      BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue(SrcReg1.asMCReg())); // SrcReg1 to BREG

      // Inserts after MI
      MachineBasicBlock::iterator MBBI = MI;
      BuildMI(MBB, ++MBBI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue(DstReg.asMCReg()));  // Stack Offset goes via OREG
      return true;
    }
    break;

  case T8xx::ADDmemmemop:
  case T8xx::SUBmemmemop:
  case T8xx::MULmemmemop:
  case T8xx::SDIVmemmemop:
  case T8xx::SREMmemmemop:
  case T8xx::SHLmemmemop:
  case T8xx::SHRmemmemop:
  case T8xx::XORmemmemop:
  case T8xx::ORmemmemop:
  case T8xx::ANDmemmemop:
    {
      // DstReg = 0
      // Src1 = 1 (Reg/Imm)
      // FI = 2 (Reg), Offset = 3 (Imm)

      // In contrast to LDRi8 etc., here the value at an address
      // should be put into A reg.
      const unsigned FI = MI.getOperand(3).getImm();
      if (MI.getOperand(2).getReg() == T8xx::WPTR)
	if (FI % 4 == 0)
	  BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(FI / 4);
	else
	  printf ("LDRi unaligned frame index %i\n", FI);
      else
	{
	  // TODO: Create a sequence which reads from
	  // a global address (Some register + some offset)
	  printf ("LDRi wrong register in operand type\n");
	}

      // Load second operand
      loadRegStack (MI, 1);

      // Destination register
      // Inserts after MI
      storeRegStack (MI, 0, true);
      return true;
    }
    break;

  case T8xx::SRAregregop:
  case T8xx::SRAregimmop:
    {
      // Section 5.7.3 from compiler writers guide (Single length arithmetic shifts)
      // sra (P29 Compiler writing guide)
      // ldl X; xdble; ldl Y; lshr; stl X  (right)
      const Register DstReg = MI.getOperand(0).getReg();

      loadRegStack (MI, 1);  // X (value to be shifted)
      BuildMI(MBB, MI, DL, get(T8xx::XDBLE));
      loadRegStack (MI, 2);  // Y (number of bits to be shifted)
      BuildMI(MBB, MI, DL, get(T8xx::LSHR));
      BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue (DstReg.asMCReg()));  // Stack Offset goes via OREG

      MBB.erase(MI);
      return (true);
    }
    break;

    // sla
    // ldl X; xdble; ldl Y; lshr; csngl; stl X  (left)
    
  case T8xx::ROTRregregop:
  case T8xx::ROTRregimmop:
    {
      // Section 5.7.3 from compiler writers guide (Single length arithmetic shifts)
      // rotr (P30 Compilter writing guide)
      // ldl X; ldc 0; ldl Y; lshr; or; stl X   (rotate X by Y places right)
      const Register DstReg = MI.getOperand(0).getReg();

      loadRegStack (MI, 1);  // X (value to be shifted)
      BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(0);
      loadRegStack (MI, 2);  // Y (number of bits to be shifted)
      BuildMI(MBB, MI, DL, get(T8xx::LSHR));
      BuildMI(MBB, MI, DL, get(T8xx::OR));
      BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue (DstReg.asMCReg()));  // Stack Offset goes via OREG

      MBB.erase(MI);
      return (true);
    }
    break;

  case T8xx::ROTLregregop:
  case T8xx::ROTLregimmop:
    {
      // Section 5.7.3 from compiler writers guide (Single length arithmetic shifts)
      // rotl
      // ldc 0; ldl X; ldl Y; lshl; or; stl X   (rotate X by Y places left)
      const Register DstReg = MI.getOperand(0).getReg();

      BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(0);
      loadRegStack (MI, 1);  // X (value to be shifted)
      loadRegStack (MI, 2);  // Y (number of bits to be shifted)
      BuildMI(MBB, MI, DL, get(T8xx::LSHL));
      BuildMI(MBB, MI, DL, get(T8xx::OR));
      BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue (DstReg.asMCReg()));  // Stack Offset goes via OREG

      MBB.erase(MI);
      return (true);
    }
    break;

  case T8xx::UDIVregregop:
  case T8xx::UDIVregimmop:
  case T8xx::UREMregregop:
  case T8xx::UREMregimmop:
    {
      // Section 5.7.2, P28 top from compiler writers guide (Multiple length multiplication and division)
      const Register DstReg = MI.getOperand(0).getReg();

      BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(0);  // Most significant word (0)
      loadRegStack (MI, 1);  // X (value to be divided)
      loadRegStack (MI, 2);  // Y (divisor)
      BuildMI(MBB, MI, DL, get(T8xx::LDIV));
      // Result is in A, Remainder in B. If Remainder is requested, swap results.
      if ((MI.getOpcode() == T8xx::UREMregregop) ||
	  (MI.getOpcode() == T8xx::UREMregimmop))
	BuildMI(MBB, MI, DL, get(T8xx::REV));
      BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue (DstReg.asMCReg()));  // Stack Offset goes via OREG

      MBB.erase(MI);
      return (true);
    }
    break;
    
  case T8xx::ADDregregop:
  case T8xx::SUBregregop:
  case T8xx::MULregregop:
  case T8xx::SDIVregregop:
  case T8xx::SREMregregop:
  case T8xx::SHLregregop:
  case T8xx::SHRregregop:
  case T8xx::XORregregop:
  case T8xx::ORregregop:
  case T8xx::ANDregregop:
  case T8xx::ADDregimmop:
  case T8xx::SUBregimmop:
  case T8xx::MULregimmop:
  case T8xx::SDIVregimmop:
  case T8xx::SREMregimmop:
  case T8xx::SHLregimmop:
  case T8xx::SHRregimmop:
  case T8xx::XORregimmop:
  case T8xx::ORregimmop:
  case T8xx::ANDregimmop:
    {
      loadRegStack (MI, 1);
      loadRegStack (MI, 2);

      // Destination register (outs!?)
      const Register DstReg = MI.getOperand(0).getReg();
      MachineBasicBlock::iterator MBBI = MI;
      // Inserts after MI
      BuildMI(MBB, ++MBBI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue (DstReg.asMCReg()));
      return true;
    }
    break;
  }
}
