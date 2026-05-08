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

    bool expandAtomicBinOp(MachineBasicBlock &BB,
                           MachineBasicBlock::iterator I,
                           MachineBasicBlock::iterator &NMBBI);
    /*
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
// Out = AReg = Value Previously at *Ptr
//
bool T8xxExpandPseudo::expandAtomicCmpSwap(MachineBasicBlock &BB,
					   MachineBasicBlock::iterator I,
					   MachineBasicBlock::iterator &NMBBI)
{
  MachineFunction *MF = BB.getParent();
  DebugLoc DL = I->getDebugLoc();
  int64_t  FI_Offset = I->getOperand(5).getImm(); // That is the offset to WPtr for temp storage

  // insert new blocks after the current block
  const BasicBlock *LLVM_BB = BB.getBasicBlock();
  MachineBasicBlock *loop1MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *loop2MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB.getIterator();
  MF->insert(It, loop1MBB);
  MF->insert(It, loop2MBB);
  MF->insert(It, sinkMBB);
  MF->insert(It, exitMBB);

  // Transfer the remainder of BB and its successor edges to exitMBB.
  exitMBB->splice(exitMBB->begin(), &BB,
                  std::next(MachineBasicBlock::iterator(I)), BB.end());
  exitMBB->transferSuccessorsAndUpdatePHIs(&BB);

  //  thisMBB:
  //    ...
  //    fallthrough --> loop1MBB
  BB.addSuccessor(loop1MBB, BranchProbability::getOne());
  loop1MBB->addSuccessor(sinkMBB);
  loop1MBB->addSuccessor(loop2MBB);
  loop1MBB->normalizeSuccProbs();
  loop2MBB->addSuccessor(loop1MBB);
  loop2MBB->addSuccessor(sinkMBB);
  loop2MBB->normalizeSuccProbs();
  sinkMBB->addSuccessor(exitMBB, BranchProbability::getOne());

  // loop1MBB:
  // stl (temp)
  // ldl (temp)
  // ldnl 0 // AReg = *Ptr, BReg = OldVal, CReg = NewVal
  // diff   // AReg = 0 when *Ptr == OldVal, BReg = NewVal

  // ldl (temp)  // AReg = Ptr, BReg = diff, CReg = NewVal
  // ldnl 0    // AReg = *Ptr, BReg = diff, CReg = NewVal
  // rev         // AReg = diff, BReg = *Ptr, CReg = NewVal
  // eqc 0  // AReg = 1 when *Ptr == OldVal, Continue at next,
  //           AReg = 0 when *Ptr != OldVal, Jump to dest
  //           -> At this point AReg = (*Ptr != OldVal), BReg = NewVal
  // cj sink

  BuildMI(loop1MBB, DL, TII->get(T8xx::STL)).addReg(T8xx::AREG).addReg(T8xx::WPTR).addImm(FI_Offset);
  BuildMI(loop1MBB, DL, TII->get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(FI_Offset);
  BuildMI(loop1MBB, DL, TII->get(T8xx::LDNL), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
  BuildMI(loop1MBB, DL, TII->get(T8xx::DIFF), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);

  BuildMI(loop1MBB, DL, TII->get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(FI_Offset);
  BuildMI(loop1MBB, DL, TII->get(T8xx::LDNL), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
  BuildMI(loop1MBB, DL, TII->get(T8xx::REV), T8xx::AREG).addReg(T8xx::ABREG);

  BuildMI(loop1MBB, DL, TII->get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
  BuildMI(loop1MBB, DL, TII->get(T8xx::CJ)).addReg(T8xx::AREG).addMBB(sinkMBB, 0);

  // loop2MBB:  AReg = *Ptr, BReg = NewVal, comparison was successful
  // rev
  // ldl (temp)
  // stnl 0
  // ldc 0
  BuildMI(loop2MBB, DL, TII->get(T8xx::REV), T8xx::AREG).addReg(T8xx::ABREG);
  BuildMI(loop2MBB, DL, TII->get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(FI_Offset);
  BuildMI(loop2MBB, DL, TII->get(T8xx::STNL)).addReg(T8xx::AREG).addReg(T8xx::AREG).addImm(0);
  BuildMI(loop2MBB, DL, TII->get(T8xx::LDC), T8xx::AREG).addImm(0);

  // sink  : AReg = 0, BReg = *Ptr, CReg = NewVal
  // rev
  BuildMI(sinkMBB, DL, TII->get(T8xx::REV), T8xx::AREG).addReg(T8xx::ABREG);

  /*
  LivePhysRegs LiveRegs;
  computeAndAddLiveIns(LiveRegs, *loop1MBB);
  computeAndAddLiveIns(LiveRegs, *exitMBB);
  */

  NMBBI = BB.end();
  I->eraseFromParent();
  return true;
}


bool T8xxExpandPseudo::expandAtomicBinOp(MachineBasicBlock &BB,
                                         MachineBasicBlock::iterator I,
                                         MachineBasicBlock::iterator &NMBBI) {
  MachineFunction *MF = BB.getParent();

  /*
  const bool ArePtrs64bit = STI->getABI().ArePtrs64bit();
  DebugLoc DL = I->getDebugLoc();

  unsigned LL, SC, ZERO, BEQ, SLT, SLTu, OR, MOVN, MOVZ, SELNEZ, SELEQZ;

  // Instruction opcodes from Mips
  LL = STI->hasMips32r6()
    ? (ArePtrs64bit ? Mips::LL64_R6 : Mips::LL_R6)
    : (ArePtrs64bit ? Mips::LL64 : Mips::LL);
  SC = STI->hasMips32r6()
    ? (ArePtrs64bit ? Mips::SC64_R6 : Mips::SC_R6)
    : (ArePtrs64bit ? Mips::SC64 : Mips::SC);
  BEQ = Mips::BEQ;
  SLT = Mips::SLT;
  SLTu = Mips::SLTu;
  OR = Mips::OR;
  MOVN = Mips::MOVN_I_I;
  MOVZ = Mips::MOVZ_I_I;
  SELNEZ = Mips::SELNEZ;
  SELEQZ = Mips::SELEQZ;
  ZERO = Mips::ZERO;

  Register OldVal = I->getOperand(0).getReg();
  Register Ptr = I->getOperand(1).getReg();
  Register Incr = I->getOperand(2).getReg();
  Register Scratch = I->getOperand(3).getReg();

  unsigned Opcode = 0;
  unsigned AND = 0;
  unsigned NOR = 0;

  bool IsOr = false;
  bool IsNand = false;
  bool IsMin = false;
  bool IsMax = false;
  bool IsUnsigned = false;

  switch (I->getOpcode()) {
  case Mips::ATOMIC_LOAD_ADD_I32_POSTRA:
    Opcode = Mips::ADDu;
    break;
  case Mips::ATOMIC_LOAD_SUB_I32_POSTRA:
    Opcode = Mips::SUBu;
    break;
  case Mips::ATOMIC_LOAD_AND_I32_POSTRA:
    Opcode = Mips::AND;
    break;
  case Mips::ATOMIC_LOAD_OR_I32_POSTRA:
    Opcode = Mips::OR;
    break;
  case Mips::ATOMIC_LOAD_XOR_I32_POSTRA:
    Opcode = Mips::XOR;
    break;
  case Mips::ATOMIC_LOAD_NAND_I32_POSTRA:
    IsNand = true;
    AND = Mips::AND;
    NOR = Mips::NOR;
    break;
  case Mips::ATOMIC_SWAP_I32_POSTRA:
    IsOr = true;
    break;
  case Mips::ATOMIC_LOAD_ADD_I64_POSTRA:
    Opcode = Mips::DADDu;
    break;
  case Mips::ATOMIC_LOAD_SUB_I64_POSTRA:
    Opcode = Mips::DSUBu;
    break;
  case Mips::ATOMIC_LOAD_AND_I64_POSTRA:
    Opcode = Mips::AND64;
    break;
  case Mips::ATOMIC_LOAD_OR_I64_POSTRA:
    Opcode = Mips::OR64;
    break;
  case Mips::ATOMIC_LOAD_XOR_I64_POSTRA:
    Opcode = Mips::XOR64;
    break;
  case Mips::ATOMIC_LOAD_NAND_I64_POSTRA:
    IsNand = true;
    AND = Mips::AND64;
    NOR = Mips::NOR64;
    break;
  case Mips::ATOMIC_SWAP_I64_POSTRA:
    IsOr = true;
    break;
  case Mips::ATOMIC_LOAD_UMIN_I32_POSTRA:
  case Mips::ATOMIC_LOAD_UMIN_I64_POSTRA:
    IsUnsigned = true;
    [[fallthrough]];
  case Mips::ATOMIC_LOAD_MIN_I32_POSTRA:
  case Mips::ATOMIC_LOAD_MIN_I64_POSTRA:
    IsMin = true;
    break;
  case Mips::ATOMIC_LOAD_UMAX_I32_POSTRA:
  case Mips::ATOMIC_LOAD_UMAX_I64_POSTRA:
    IsUnsigned = true;
    [[fallthrough]];
  case Mips::ATOMIC_LOAD_MAX_I32_POSTRA:
  case Mips::ATOMIC_LOAD_MAX_I64_POSTRA:
    IsMax = true;
    break;
  default:
    llvm_unreachable("Unknown pseudo atomic!");
  }

  bool NoMovnInstr = (IsMin || IsMax) && !STI->hasMips4() && !STI->hasMips32();
  const BasicBlock *LLVM_BB = BB.getBasicBlock();
  MachineBasicBlock *loopMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *loop1MBB = nullptr;
  MachineBasicBlock *loop2MBB = nullptr;
  if (NoMovnInstr) {
    loop1MBB = MF->CreateMachineBasicBlock(LLVM_BB);
    loop2MBB = MF->CreateMachineBasicBlock(LLVM_BB);
  }
  MachineBasicBlock *exitMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineFunction::iterator It = ++BB.getIterator();
  MF->insert(It, loopMBB);
  if (NoMovnInstr) {
    MF->insert(It, loop1MBB);
    MF->insert(It, loop2MBB);
  }
  MF->insert(It, exitMBB);

  exitMBB->splice(exitMBB->begin(), &BB, std::next(I), BB.end());
  exitMBB->transferSuccessorsAndUpdatePHIs(&BB);

  BB.addSuccessor(loopMBB, BranchProbability::getOne());
  if (NoMovnInstr) {
    loopMBB->addSuccessor(loop1MBB);
    loopMBB->addSuccessor(loop2MBB);
  } else {
    loopMBB->addSuccessor(exitMBB);
    loopMBB->addSuccessor(loopMBB);
  }
  loopMBB->normalizeSuccProbs();
  if (NoMovnInstr) {
    loop1MBB->addSuccessor(loop2MBB);
    loop2MBB->addSuccessor(loopMBB);
    loop2MBB->addSuccessor(exitMBB);
  }

  BuildMI(loopMBB, DL, TII->get(LL), OldVal).addReg(Ptr).addImm(0);
  assert((OldVal != Ptr) && "Clobbered the wrong ptr reg!");
  assert((OldVal != Incr) && "Clobbered the wrong reg!");
  if (IsMin || IsMax) {

    assert(I->getNumOperands() == 5 &&
           "Atomics min|max|umin|umax use an additional register");
    MCRegister Scratch2 = I->getOperand(4).getReg().asMCReg();

    // On Mips64 result of slt is GPR32.
    MCRegister Scratch2_32 =
        (Size == 8) ? STI->getRegisterInfo()->getSubReg(Scratch2, Mips::sub_32)
                    : Scratch2;

    unsigned SLTScratch2 = IsUnsigned ? SLTu : SLT;
    unsigned SELIncr = IsMax ? SELNEZ : SELEQZ;
    unsigned SELOldVal = IsMax ? SELEQZ : SELNEZ;
    unsigned MOVIncr = IsMax ? MOVN : MOVZ;

    // unsigned: sltu Scratch2, oldVal, Incr
    // signed:   slt Scratch2, oldVal, Incr
    BuildMI(loopMBB, DL, TII->get(SLTScratch2), Scratch2_32)
        .addReg(OldVal)
        .addReg(Incr);

    if (STI->hasMips64r6() || STI->hasMips32r6()) {
      // max: seleqz Scratch, OldVal, Scratch2
      //      selnez Scratch2, Incr, Scratch2
      //      or Scratch, Scratch, Scratch2
      // min: selnez Scratch, OldVal, Scratch2
      //      seleqz Scratch2, Incr, Scratch2
      //      or Scratch, Scratch, Scratch2
      BuildMI(loopMBB, DL, TII->get(SELOldVal), Scratch)
          .addReg(OldVal)
          .addReg(Scratch2);
      BuildMI(loopMBB, DL, TII->get(SELIncr), Scratch2)
          .addReg(Incr)
          .addReg(Scratch2);
      BuildMI(loopMBB, DL, TII->get(OR), Scratch)
          .addReg(Scratch)
          .addReg(Scratch2);
    } else if (STI->hasMips4() || STI->hasMips32()) {
      // max: move Scratch, OldVal
      //      movn Scratch, Incr, Scratch2, Scratch
      // min: move Scratch, OldVal
      //      movz Scratch, Incr, Scratch2, Scratch
      BuildMI(loopMBB, DL, TII->get(OR), Scratch)
          .addReg(OldVal)
          .addReg(ZERO);
      BuildMI(loopMBB, DL, TII->get(MOVIncr), Scratch)
          .addReg(Incr)
          .addReg(Scratch2)
          .addReg(Scratch);
    } else {
      // if min:
      // loopMBB:  move Scratch, OldVal
      //           beq Scratch2_32, 0, loop1MBB
      //           j loop2MBB
      // loop1MBB: move Scratch, Incr
      // loop2MBB: sc $2, 0($4)
      //           beqz	$2, $BB0_1
      //           nop
      //
      // if max:
      // loopMBB:  move Scratch, Incr
      //           beq Scratch2_32, 0, loop1MBB
      //           j loop2MBB
      // loop1MBB: move Scratch, OldVal
      // loop2MBB: sc $2, 0($4)
      //           beqz	$2, $BB0_1
      //           nop
      if (IsMin) {
        BuildMI(loopMBB, DL, TII->get(OR), Scratch).addReg(OldVal).addReg(ZERO);
        BuildMI(loop1MBB, DL, TII->get(OR), Scratch).addReg(Incr).addReg(ZERO);
      } else {
        BuildMI(loopMBB, DL, TII->get(OR), Scratch).addReg(Incr).addReg(ZERO);
        BuildMI(loop1MBB, DL, TII->get(OR), Scratch)
            .addReg(OldVal)
            .addReg(ZERO);
      }
      BuildMI(loopMBB, DL, TII->get(BEQ))
          .addReg(Scratch2_32)
          .addReg(ZERO)
          .addMBB(loop1MBB);
      BuildMI(loopMBB, DL, TII->get(Mips::J)).addMBB(loop2MBB);
    }

  } else if (Opcode) {
    BuildMI(loopMBB, DL, TII->get(Opcode), Scratch).addReg(OldVal).addReg(Incr);
  } else if (IsNand) {
    assert(AND && NOR &&
           "Unknown nand instruction for atomic pseudo expansion");
    BuildMI(loopMBB, DL, TII->get(AND), Scratch).addReg(OldVal).addReg(Incr);
    BuildMI(loopMBB, DL, TII->get(NOR), Scratch).addReg(ZERO).addReg(Scratch);
  } else {
    assert(IsOr && OR && "Unknown instruction for atomic pseudo expansion!");
    (void)IsOr;
    BuildMI(loopMBB, DL, TII->get(OR), Scratch).addReg(Incr).addReg(ZERO);
  }

  if (NoMovnInstr) {
    BuildMI(loop2MBB, DL, TII->get(SC), Scratch)
        .addReg(Scratch)
        .addReg(Ptr)
        .addImm(0);
    BuildMI(loop2MBB, DL, TII->get(BEQ))
        .addReg(Scratch)
        .addReg(ZERO)
        .addMBB(loopMBB);
  } else {
    BuildMI(loopMBB, DL, TII->get(SC), Scratch)
        .addReg(Scratch)
        .addReg(Ptr)
        .addImm(0);
    BuildMI(loopMBB, DL, TII->get(BEQ))
        .addReg(Scratch)
        .addReg(ZERO)
        .addMBB(loopMBB);
  }
  */

  
  //  NMBBI = BB.end();
  I->eraseFromParent();

  /*
  LivePhysRegs LiveRegs;
  computeAndAddLiveIns(LiveRegs, *loopMBB);
  if (loop1MBB) {
    assert(loop2MBB && "should have 2 loop blocks");
    computeAndAddLiveIns(LiveRegs, *loop1MBB);
    computeAndAddLiveIns(LiveRegs, *loop2MBB);
  }
  computeAndAddLiveIns(LiveRegs, *exitMBB);
  */

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

  case T8xx::ATOMIC_LOAD_ADD_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_SUB_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_AND_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_OR_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_XOR_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_NAND_I32_POSTRA:
  case T8xx::ATOMIC_SWAP_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_MIN_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_MAX_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_UMIN_I32_POSTRA:
  case T8xx::ATOMIC_LOAD_UMAX_I32_POSTRA:
    {
      return expandAtomicBinOp(MBB, MBBI, NMBB);
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
	  BuildMI (MBB, *MBBI, DL, TII->get(T8xx::LDC), T8xx::AREG).addImm(65535);
	  BuildMI (MBB, *MBBI, DL, TII->get(T8xx::AND), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);

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

  case T8xx::LONGSHL:
  case T8xx::LONGSHR:
    {
      MCRegister DstReg = MBBI->getOperand(0).getReg ();
      MCRegister X1Reg = MBBI->getOperand(1).getReg ();
      MCRegister X2Reg = MBBI->getOperand(2).getReg ();
      MCRegister CntReg = MBBI->getOperand(3).getReg ();

      unsigned opcode = (MBBI->getOpcode() == T8xx::LONGSHR) ? T8xx::LSHR : T8xx::LSHR;

      BuildMI (MBB, *MBBI, DL, TII->get(opcode), T8xx::ABREG).addReg(X1Reg).addReg(X2Reg).addReg(CntReg);
      BuildMI (MBB, *MBBI, DL, TII->get(T8xx::OR), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);

      MBBI->eraseFromParent();
      return true;
    }
    break;

    // Pseudo instruction needs to be removed
  case T8xx::SELLOW:
  case T8xx::JOIN:
    {
      MBBI->eraseFromParent();
      return true;
    }
    break;

  case T8xx::TxSync:
    {
      /*
      dbgs() << "Sync\n";
      MBB.dump();
      dbgs() << "------------\n";
      */
      MBBI->eraseFromParent();
      /*
      dbgs() << "Post delete Sync\n";
      MBB.dump();
      dbgs() << "------------\n";
      */
      return true;
    }
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
