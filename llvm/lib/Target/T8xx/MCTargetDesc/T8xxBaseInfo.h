//===-- T8xxBaseInfo.h - Top level definitions for T8xx MC ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone helper functions and enum definitions for
// the Mips target useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_T8XX_MCTARGETDESC_MIPSBASEINFO_H
#define LLVM_LIB_TARGET_T8XX_MCTARGETDESC_MIPSBASEINFO_H

#include "T8xxFixupKinds.h"
#include "T8xxMCTargetDesc.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Support/DataTypes.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// MipsII - This namespace holds all of the target specific flags that
/// instruction info tracks.
///
namespace T8xxII {
/// Target Operand Flag enum.
enum TOF {
  //===------------------------------------------------------------------===//
  // T8xx Specific MachineOperand flags.

  MO_NO_FLAG,

  // Global address
  MO_GLOBAL,

  // Instruction pointer relative address
  MO_IPTRREL,

  // Instruction pointer relative address
  MO_PCREL_SYM,
};

} // namespace T8xxII

} // namespace llvm

#endif
