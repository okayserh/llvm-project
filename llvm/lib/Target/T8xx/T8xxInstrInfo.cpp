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
#include "llvm/ADT/ScopeExit.h"
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
Register T8xxInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
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
Register T8xxInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
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
  printf ("T8xx::analyzeBranch\n");

  auto UncondBranch =
      std::pair<MachineBasicBlock::reverse_iterator, MachineBasicBlock *>{
          MBB.rend(), nullptr};

  // Erase any instructions if allowed at the end of the scope.
  std::vector<std::reference_wrapper<llvm::MachineInstr>> EraseList;
  auto FinalizeOnReturn = llvm::make_scope_exit([&EraseList] {
    std::for_each(EraseList.begin(), EraseList.end(),
                  [](auto &ref) { ref.get().eraseFromParent(); });
  });

  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  for (auto iter = MBB.rbegin(); iter != MBB.rend(); iter = std::next(iter)) {

    unsigned Opcode = iter->getOpcode();

    if (iter->isDebugInstr())
      continue;

    // Working from the bottom, when we see a non-terminator instruction, we're
    // done.
    if (!isUnpredicatedTerminator(*iter))
      break;

    // A terminator that isn't a branch can't easily be handled by this
    // analysis.
    if (!iter->isBranch())
      return true;

    // Handle unconditional branches.
    if (Opcode == T8xx::JUMP) {
      if (!iter->getOperand(0).isMBB())
        return true;
      UncondBranch = {iter, iter->getOperand(0).getMBB()};

      // TBB is used to indicate the unconditional destination.
      TBB = UncondBranch.second;

      if (!AllowModify)
        continue;

      // If the block has any instructions after a JMP, erase them.
      EraseList.insert(EraseList.begin(), MBB.rbegin(), iter);

      Cond.clear();
      FBB = nullptr;

      // Erase the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(UncondBranch.second)) {
        TBB = nullptr;
        EraseList.push_back(*iter);
        UncondBranch = {MBB.rend(), nullptr};
      }

      continue;
    }

    // Handle conditional branches.
    // Note: On the T8xx there is only one type of conditional branch
    // Hence no BranchCode is neede
    //    auto BranchCode = M68k::GetCondFromBranchOpc(Opcode);

    // Can't handle indirect branch.
    /*
    if (BranchCode == M68k::COND_INVALID)
      return true;
    */

    // In practice we should never have an undef CCR operand, if we do
    // abort here as we are not prepared to preserve the flag.
    // ??? Is this required?
    // if (iter->getOperand(1).isUndef())
    //   return true;

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      if (!iter->getOperand(1).isMBB())
        return true;
      MachineBasicBlock *CondBranchTarget = iter->getOperand(1).getMBB();

      // If we see something like this:
      //
      //     bcc l1
      //     bra l2
      //     ...
      //   l1:
      //     ...
      //   l2:
      /*
      if (UncondBranch.first != MBB.rend()) {

        assert(std::next(UncondBranch.first) == iter && "Wrong block layout.");

        // And we are allowed to modify the block and the target block of the
        // conditional branch is the direct successor of this block:
        //
        //     bcc l1
        //     bra l2
        //   l1:
        //     ...
        //   l2:
        //
        // we change it to this if allowed:
        //
        //     bncc l2
        //   l1:
        //     ...
        //   l2:
        //
        // Which is a bit more efficient.
        if (AllowModify && MBB.isLayoutSuccessor(CondBranchTarget)) {

          BranchCode = GetOppositeBranchCondition(BranchCode);
          unsigned BNCC = GetCondBranchFromCond(BranchCode);

          BuildMI(MBB, *UncondBranch.first, MBB.rfindDebugLoc(iter), get(BNCC))
              .addMBB(UncondBranch.second);

          EraseList.push_back(*iter);
          EraseList.push_back(*UncondBranch.first);

          TBB = UncondBranch.second;
          FBB = nullptr;
          Cond.push_back(MachineOperand::CreateImm(BranchCode));

          // Otherwise preserve TBB, FBB and Cond as requested
        } else {
          TBB = CondBranchTarget;
          FBB = UncondBranch.second;
	  //          Cond.push_back(MachineOperand::CreateImm(BranchCode));
	  Cond.push_back(MachineOperand::CreateImm(ISD::CondCode::SETFALSE));
    }
	  
        UncondBranch = {MBB.rend(), nullptr};
        continue;
      }
	  */

      TBB = CondBranchTarget;
      FBB = nullptr;

      //Cond.push_back(MachineOperand::CreateImm(BranchCode));
      Cond.push_back(MachineOperand::CreateImm(ISD::CondCode::SETFALSE));

      continue;
    }

  /* TODO: Check what needs to be done here
  // Handle subsequent conditional branches. Only handle the case where all
    // conditional branches branch to the same destination and their condition
    // opcodes fit one of the special multi-branch idioms.
    assert(Cond.size() == 1);
    assert(TBB);

    // If the conditions are the same, we can leave them alone.
    auto OldBranchCode = static_cast<M68k::CondCode>(Cond[0].getImm());
    if (!iter->getOperand(0).isMBB())
      return true;
    auto NewTBB = iter->getOperand(0).getMBB();
    if (OldBranchCode == BranchCode && TBB == NewTBB)
      continue;
  */

  // If they differ we cannot do much here.
    return true;
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
    if ((Opc == T8xx::JUMP) || (Opc == T8xx::CJ)) {
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
      BuildMI(MBB, MBB.end(), DL, get(T8xx::CJ)).addReg(Cond[0].getReg()).addMBB(TBB);
      NumInserted++;
    }
  else
    {
      // Insert any unconditional branch.
      if (Cond.empty() || FBB) {
	BuildMI(MBB, MBB.end(), DL, get(T8xx::JUMP)).addMBB(Cond.empty() ? TBB : FBB);
	NumInserted++;
      }
    }
  return NumInserted;
}

// ----

void T8xxInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 const DebugLoc &DL, MCRegister DestReg,
                                 MCRegister SrcReg, bool KillSrc,
                                 bool RenamableDest, bool RenamableSrc) const {
  /*
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  uint16_t hweSrcReg = TRI->getEncodingValue (SrcReg);
  uint16_t hweDstReg = TRI->getEncodingValue (DestReg);
  */
}

void T8xxInstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                    Register SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI,
		    Register VReg) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addReg(SrcReg, getKillRegState(true))
    .addFrameIndex(FI).addImm(0);
}

void T8xxInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     Register DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI,
		     Register VReg) const {
  if (RC == &T8xx::ORegRegClass)
    BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL), DestReg).addFrameIndex(FI).addImm(0);
  else
    llvm_unreachable("Can't load this register from stack slot");
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


bool T8xxInstrInfo::expandPostRAPseudo(MachineInstr &MI) const
{
  printf ("expandPostRAPseudo %i %i\n", MI.getOpcode (), T8xx::LDC);

  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  DebugLoc DL = MI.getDebugLoc();

  switch (MI.getOpcode())
  {
  default:
    return false;

    // This is a special instruction to introduce a way to get effective addresses
    // that are not aligned
  case T8xx::AddWptrImm:
  case T8xx::LDLPb:
    {
      int64_t rem = MI.getOperand(2).getImm () % 4;
      MCRegister DstReg = MI.getOperand(0).getReg ();
      MCRegister SrcReg = MI.getOperand(1).getReg ();
      BuildMI (MBB, MI, DL, get(T8xx::LDLP), DstReg).addReg(SrcReg).
	addImm((MI.getOperand(2).getImm () - rem) / 4);  // Divide by 4 to get offset in words
      if (rem != 0)
	{
	  MI.getOperand(2).setImm (MI.getOperand(2).getImm() - rem);
	  BuildMI (MBB, MI, DL, get(T8xx::ADC), T8xx::AREG).addReg(T8xx::AREG).addImm(rem);
	}
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::RET:
    {
      BuildMI (MBB, MI, DL, get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, MI, DL, get(T8xx::GCALL)).addReg(T8xx::AREG);
      MBB.erase(MI);
      return true;
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
      if (MI.getOperand(0).isGlobal ())
	BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addGlobalAddress(MI.getOperand(0).getGlobal (), 0, T8xxMCExpr::VK_T8xx_GLOBAL);
      if (MI.getOperand(0).isSymbol ())
	BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addExternalSymbol(MI.getOperand(0).getSymbolName (), T8xxMCExpr::VK_T8xx_GLOBAL);

      /* Stuff for IPTR relative adressing
      BuildMI (MBB, MI, DL, get(T8xx::ADC), T8xx::AREG).addReg(T8xx::AREG).addImm(-4);
      BuildMI (MBB, MI, DL, get(T8xx::LDPI), T8xx::AREG).addReg(T8xx::AREG);
      */
      BuildMI (MBB, MI, DL, get(T8xx::GCALL)).addReg(T8xx::AREG);
      BuildMI (MBB, MI, DL, get(T8xx::REV));
      MBB.erase(MI);

      /*
      BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, MI, DL, get(T8xx::GCALL)).addReg(T8xx::AREG);
      MBB.erase(MI);
      */
      return true;
    }
    break;
  }
  return false;
}
