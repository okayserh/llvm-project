//===-- T8xxAsmPrinter.h - T8xx implementation of AsmPrinter ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_T8XX_T8XXASMPRINTER_H
#define LLVM_LIB_TARGET_T8XX_T8XXASMPRINTER_H

#include "T8xxSubtarget.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class T8xxFunctionInfo;
class MCOperand;
class MachineConstantPool;
class MachineOperand;
class MCSymbol;


class LLVM_LIBRARY_VISIBILITY T8xxAsmPrinter : public AsmPrinter {

  /*
  T8xxTargetStreamer &getTargetStreamer() {
      return static_cast<T8xxTargetStreamer &>(
          *OutStreamer->getTargetStreamer());
    }
  */
    
  public:
    explicit T8xxAsmPrinter(TargetMachine &TM,
			    std::unique_ptr<MCStreamer> Streamer);

    StringRef getPassName() const override { return "T8xx Assembly Printer"; }

    void printOperand(const MachineInstr *MI, int opNum, raw_ostream &OS);
    void printMemOperand(const MachineInstr *MI, int opNum, raw_ostream &OS,
                         const char *Modifier = nullptr);

    bool isBlockOnlyReachableByFallthrough(const MachineBasicBlock *MBB) const;

    // Taken from LEG machine (TODO)
    void printAddrModeMemSrc(const MachineInstr *MI, int OpNum,
			     raw_ostream &O);

    void emitInstruction(const MachineInstr *MI) override;

    MCOperand LowerSymbolOperand(const MachineOperand &MO);
    // lowerOperand - Convert a MachineOperand into the equivalent MCOperand.
    bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp);

    bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                         const char *ExtraCode, raw_ostream &O) override;
    bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                               const char *ExtraCode, raw_ostream &O) override;

  };
}  // end namespace llvm

#endif
