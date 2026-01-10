//===-- T8xxExpandPseudo.cpp - Expand pseudo instructions -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions to allow proper scheduling, if-conversion, and other late
// optimizations. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//===----------------------------------------------------------------------===//

#include "T8xx.h"
#include "T8xxInstrInfo.h"
#include "T8xxSubtarget.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"

using namespace llvm;

#define DEBUG_TYPE "t8xx-pseudo"

namespace {
  class T8xxExpandPseudo : public MachineFunctionPass {
  public:
    static char ID;
    T8xxExpandPseudo() : MachineFunctionPass(ID) {}

    const T8xxInstrInfo *TII;
    const T8xxSubtarget *STI;

    bool runOnMachineFunction(MachineFunction &Fn) override;

    MachineFunctionProperties getRequiredProperties() const override {
      return MachineFunctionProperties().setNoVRegs();
    }

    StringRef getPassName() const override {
      return "T8xx pseudo instruction expansion pass";
    }

  private:
    bool expandAtomicCmpSwap(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator MBBI,
                             MachineBasicBlock::iterator &NextMBBI);
    /*
    bool expandAtomicBinOp(MachineBasicBlock &BB,
                           MachineBasicBlock::iterator I,
                           MachineBasicBlock::iterator &NMBBI, unsigned Size);
    bool expandAtomicBinOpSubword(MachineBasicBlock &BB,
                                  MachineBasicBlock::iterator I,
                                  MachineBasicBlock::iterator &NMBBI);
    */
    bool expandMI(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                  MachineBasicBlock::iterator &NMBB);
    bool expandMBB(MachineBasicBlock &MBB);
   };
  char T8xxExpandPseudo::ID = 0;
}


// Note: Basis taken from "MipsExpandPseudo.cpp".

// Note: May need major adaptations. To make is useable for
// the transputer, the instruction needs to be limited to
// 3 register inputs.
//
// AtomiCmpSwap:
// In: AReg = Ptr, BReg = OldVal, CReg = NewVal
// Out = AReg = Result
//
// ## Original block
// Needs temp storage! (Use the Scratch register also used at other places)
// stl (temp)
// ldl (temp)
// ldnl 0 // AReg = *Ptr, BReg = OldVal, CReg = NewVal
// diff   // AReg = 0 when *Ptr == OldVal
// eqc 0  // AReg = 1 when *Ptr == OldVal, Continue at next,
//           AReg = 0 when *Ptr != OldVal, Jump to dest
//           -> At this point AReg = (*Ptr != OldVal), BReg = NewVal
// cj end

// ## Block Conditional (New)
// When OldVal == *Ptr
// ldl (temp)
// stnl 0

// ## Exit block  (New)
// When OldVal != *Ptr
// end:
// ldl (temp)
// ldnl 0

bool T8xxExpandPseudo::expandAtomicCmpSwap(MachineBasicBlock &BB,
					   MachineBasicBlock::iterator I,
					   MachineBasicBlock::iterator &NMBBI)
{
  MachineFunction *MF = BB.getParent();
  
  dbgs() << "++++++++++++++++ Atomic Cmp Swap expanded !!! ++++++++++++++++\n";
  LLVM_DEBUG({
      I->dump ();
      for (unsigned int i = 0; i < I->getNumOperands (); ++i)
	{
	  dbgs () << "ATOMIC SWAP Op" << i << " " << I->getOperand (i).getType () << "\n";
	  I->getOperand (i).dump ();
	}
    });

  DebugLoc DL = I->getDebugLoc();
  
  /*
  Register Dest = I->getOperand(0).getReg();
  Register Ptr = I->getOperand(1).getReg();
  Register OldVal = I->getOperand(2).getReg();
  Register NewVal = I->getOperand(3).getReg();
  Register Scratch = I->getOperand(4).getReg(); // That is WPtr
  */
  int64_t  FI_Offset = I->getOperand(5).getImm(); // That is the offset to WPtr for temp storage

  // insert new blocks after the current block
  const BasicBlock *LLVM_BB = BB.getBasicBlock();
  MachineBasicBlock *loop1MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *loop2MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB.getIterator();
  MF->insert(It, loop1MBB);
  MF->insert(It, loop2MBB);
  MF->insert(It, exitMBB);

  dbgs () << "New MBBs inserted\n";

  // Transfer the remainder of BB and its successor edges to exitMBB.
  exitMBB->splice(exitMBB->begin(), &BB,
                  std::next(MachineBasicBlock::iterator(I)), BB.end());
  exitMBB->transferSuccessorsAndUpdatePHIs(&BB);

  //  thisMBB:
  //    ...
  //    fallthrough --> loop1MBB
  BB.addSuccessor(loop1MBB, BranchProbability::getOne());

  loop1MBB->addSuccessor(exitMBB);
  loop1MBB->addSuccessor(loop2MBB);
  loop1MBB->normalizeSuccProbs();
  loop2MBB->addSuccessor(loop1MBB);
  loop2MBB->addSuccessor(exitMBB);
  loop2MBB->normalizeSuccProbs();

  // loop1MBB:
  BuildMI(loop1MBB, DL, TII->get(T8xx::STL)).addReg(T8xx::AREG).addReg(T8xx::WPTR).addImm(FI_Offset);
  BuildMI(loop1MBB, DL, TII->get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(FI_Offset);
  BuildMI(loop1MBB, DL, TII->get(T8xx::LDNL), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
  BuildMI(loop1MBB, DL, TII->get(T8xx::DIFF), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);
  BuildMI(loop1MBB, DL, TII->get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
  BuildMI(loop1MBB, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addMBB(exitMBB, 0);

  // loop2MBB:
// ldl (temp)
// stnl 0
  BuildMI(loop2MBB, DL, TII->get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(FI_Offset);
  BuildMI(loop2MBB, DL, TII->get(T8xx::STNL), T8xx::AREG).addReg(T8xx::AREG).addImm(0);

  // loop1MBB:
  //   ll dest, 0(ptr)
  //   bne dest, oldval, exitMBB

  dbgs () << "New Insts inserted\n";

  /*
  LivePhysRegs LiveRegs;
  computeAndAddLiveIns(LiveRegs, *loop1MBB);
  computeAndAddLiveIns(LiveRegs, *exitMBB);
  */

  NMBBI = BB.end();
  I->eraseFromParent();
  return true;
}


bool T8xxExpandPseudo::expandMI(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MBBI,
                                MachineBasicBlock::iterator &NMBB) {
  LLVM_DEBUG(dbgs()<< "expandMI Opcode: " << MBBI->getOpcode () << "\n");
  DebugLoc DL = MBBI->getDebugLoc();

  switch (MBBI->getOpcode())
  {
  default:
    return false;

  case T8xx::ATOMIC_CMP_SWAP_I32_POSTRA:
    {
      expandAtomicCmpSwap(MBB, MBBI, NMBB);
      return true;
    }
    break;

  case T8xx::MoveLoad:
  case T8xx::MoveSEXTLoad:
  case T8xx::MoveZEXTLoad:
    {
      LLVM_DEBUG({
	  dbgs()<<"Expand MoveLoad\n";
	  MBBI->dump ();
	});
      int64_t FI = MBBI->getOperand(5).getImm ();
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::MOVE)).addReg(T8xx::AREG).
	addReg(T8xx::BREG).addReg(T8xx::CREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(FI);

      if (MBBI->getOpcode() == T8xx::MoveSEXTLoad)
	{
	  BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDC), T8xx::AREG).addImm(32768);
	  BuildMI (MBB, *MBBI, DL, TII->get(T8xx::XWORD), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);
	}

      if (MBBI->getOpcode() == T8xx::MoveZEXTLoad)
	{
	  BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDC), T8xx::AREG).addImm(65535);
	  BuildMI (MBB, *MBBI, DL, TII->get(T8xx::AND), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);
	}

      MBBI->eraseFromParent();
      //      MBB.erase(*MBBI);
      return true;
    }
    break;

    // This is a special instruction to introduce a way to get effective addresses
    // that are not aligned
  case T8xx::AddWptrImm:
  case T8xx::LDLPb:
    {
      int64_t rem = MBBI->getOperand(2).getImm () % 4;
      MCRegister DstReg = MBBI->getOperand(0).getReg ();
      MCRegister SrcReg = MBBI->getOperand(1).getReg ();
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDLP), DstReg).addReg(SrcReg).
	addImm((MBBI->getOperand(2).getImm () - rem) / 4);  // Divide by 4 to get offset in words
      if (rem != 0)
	{
	  MBBI->getOperand(2).setImm (MBBI->getOperand(2).getImm() - rem);
	  BuildMI (MBB, *MBBI, DL, TII->get(T8xx::ADC), T8xx::AREG).addReg(T8xx::AREG).addImm(rem);
	}
      MBBI->eraseFromParent();
      return true;
    }
    break;

    // Pseudo instruction needs to be removed
  case T8xx::TxSync:
  case T8xx::SELLOW:
  case T8xx::JOIN:
    MBBI->eraseFromParent();
    break;

  case T8xx::RET:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::GCALL)).addReg(T8xx::AREG);
      MBBI->eraseFromParent();
      return true;
    }
    break;

    // Floating point comparisons
  case T8xx::FPOGTSN:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPORDEREDSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addImm(2);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPGTSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      MBBI->eraseFromParent();
      return true;
    }
    break;
  case T8xx::FPOGTDB:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPORDEREDDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addImm(2);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPGTDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      MBBI->eraseFromParent();
      return true;
    }
    break;

  case T8xx::FPOLESN:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPORDEREDSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addImm(3);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPGTSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
      MBBI->eraseFromParent();
      return true;
    }
    break;
  case T8xx::FPOLEDB:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPORDEREDDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addImm(3);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPGTDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
      MBBI->eraseFromParent();
      return true;
    }
    break;

  case T8xx::FPOEQSN:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPORDEREDSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addImm(2);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPEQSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      MBBI->eraseFromParent();
      return true;
    }
    break;
  case T8xx::FPOEQDB:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPORDEREDDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addImm(2);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPEQDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      MBBI->eraseFromParent();
      return true;
    }
    break;

  case T8xx::FPONESN:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPORDEREDSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addImm(3);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPEQSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
      MBBI->eraseFromParent();
      return true;
    }
    break;
  case T8xx::FPONEDB:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPORDEREDDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addImm(3);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPEQDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
      MBBI->eraseFromParent();
      return true;
    }
    break;

  case T8xx::FPR64TOI32:
  case T8xx::FPR32TOI32:
    {
      // Note: Implementation uses WPtr + 0. This position should be kept free by
      // regular instructions
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPURZ), T8xx::FPRMREG);
      BuildMI (MBB, *MBBI, DL, TII->get((MBBI->getOpcode() == T8xx::FPR32TOI32) ? T8xx::FPINTSN : T8xx::FPINTDB), T8xx::FAREG).
	addReg(T8xx::FAREG).addReg(T8xx::FPRMREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDLP), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPSTNLI32)).addReg(T8xx::FAREG).addReg(T8xx::AREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      MBBI->eraseFromParent();
      return true;
    }
    break;

    // The T8xx instruction set includes an instructions to convert an integer to a floating
    // point number. However, this works only from a memory address. As a workaround the
    // code stores the AReg in Workspace location 0, loads a pointer to that location in AReg
    // and then triggers the conversion.
  case T8xx::FPI32TOR32Reg:
  case T8xx::FPI32TOR64Reg:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::STL)).addReg(T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDLP), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      if (MBBI->getOpcode() == T8xx::FPI32TOR32Reg)
	BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPI32TOR32), T8xx::FAREG).addReg(T8xx::AREG);
      else
	BuildMI (MBB, *MBBI, DL, TII->get(T8xx::FPI32TOR64), T8xx::FAREG).addReg(T8xx::AREG);
      MBBI->eraseFromParent();
    }
    break;

    // Attempt to fix the jump table problem
  case T8xx::BRIND:
    {
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::GCALL)).addReg(T8xx::AREG);
      MBBI->eraseFromParent();
    }
    break;

  case T8xx::CALL:
    {
      LLVM_DEBUG({
	  for (unsigned int i = 0; i < MBBI->getNumOperands (); ++i)
	    {
	      dbgs () << "CALL Op" << i << " " << MBBI->getOperand (i).getType () << "\n";
	      MBBI->getOperand (i).dump ();
	    }
	});

      // First OP is MO_GlobalAddress
      // Second OP is MO_RegisterMask
      // Third and Fourth are MO_Register

      // Load offset to global address into AREG and correct by bytecount of LDPI and GCALL
      if (MBBI->getOperand(0).isGlobal ())
	BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDC), T8xx::AREG).addGlobalAddress(MBBI->getOperand(0).getGlobal (), 0, T8xxII::MO_GLOBAL);
      if (MBBI->getOperand(0).isSymbol ())
	BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDC), T8xx::AREG).addExternalSymbol(MBBI->getOperand(0).getSymbolName (), T8xxII::MO_GLOBAL);

      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::GCALL), T8xx::ABREG).addReg(T8xx::AREG);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::REV), T8xx::AREG).addReg(T8xx::ABREG);
      MBBI->eraseFromParent();
      return true;
    }
    break;
  }
  return false;
}

bool T8xxExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  MachineBasicBlock::iterator MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    MachineBasicBlock::iterator NMBBI = std::next(MBBI);
    Modified |= expandMI(MBB, MBBI, NMBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool T8xxExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  STI = &MF.getSubtarget<T8xxSubtarget>();
  TII = STI->getInstrInfo();

  bool Modified = false;
  for (MachineBasicBlock &MBB : MF)
    Modified |= expandMBB(MBB);

  if (Modified)
    MF.RenumberBlocks();

  return Modified;
}

/// createT8xxExpandPseudoPass - returns an instance of the pseudo instruction
/// expansion pass.
FunctionPass *llvm::createT8xxExpandPseudoPass() {
  return new T8xxExpandPseudo();
}
