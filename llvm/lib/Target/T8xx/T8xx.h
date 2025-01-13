//===-- T8xx.h - Top-level interface for T8xx representation --*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// T8xx back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SPARC_SPARC_H
#define LLVM_LIB_TARGET_SPARC_SPARC_H

#include "MCTargetDesc/T8xxMCTargetDesc.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class AsmPrinter;
  class FunctionPass;
  class MCInst;
  class MachineInstr;
  class PassRegistry;
  class T8xxTargetMachine;

  FunctionPass *createT8xxISelDag(T8xxTargetMachine &TM);

  FunctionPass *createT8xxStackPass();
  FunctionPass *createT8xxMoveConstPass();
  
  void LowerT8xxMachineInstrToMCInst(const MachineInstr *MI,
                                      MCInst &OutMI,
                                      AsmPrinter &AP);

  void initializeT8xxStackPassPass(PassRegistry &);
  void initializeT8xxMoveConstPassPass(PassRegistry &);
  void initializeT8xxDAGToDAGISelLegacyPass(PassRegistry &);
  
} // end namespace llvm;

#endif
