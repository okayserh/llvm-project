//===-- T8xxFixupKinds.h - T8xx Specific Fixup Entries --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_T8XX_MCTARGETDESC_T8XXFIXUPKINDS_H
#define LLVM_LIB_TARGET_T8XX_MCTARGETDESC_T8XXFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
  namespace T8xx {
    enum Fixups {
      // fixup_t8xx_addr - global address to some symbol   = 128
      // Results in 8 bytes of pfix/nfix instructions.
      fixup_t8xx_addr = FirstTargetFixupKind,

      /// fixup_t8xx_addr_npfix - global address to symbol, not needing  = 129
      /// prefix / postfix transputer instruction (i.e. results in 4 bytes with
      /// the relocated address)
      fixup_t8xx_addr_npfix,

      /// fixup_t8xx_jump - jump or conditional jump  = 130
      fixup_t8xx_jump,

      /// fixup_t8xx_pcrel_sym load
      fixup_t8xx_pcrel_sym,   // 138

      fixup_t8xx_addr_base,   // 138
      fixup_t8xx_addr_add,   // 138
      fixup_t8xx_addr_sub,   // 138

      // Marker
      LastTargetFixupKind,
      NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
    };
  }
}

#endif
