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

      /// Jumps with negative offsets or positive offset that do not fit into 4 bits
      fixup_t8xx_jump_p8,     // 131
      fixup_t8xx_jump_p12,    // 132
      fixup_t8xx_jump_p16,    // 133
      fixup_t8xx_jump_p20,    // 134
      fixup_t8xx_jump_p24,    // 135
      fixup_t8xx_jump_p28,    // 136
      fixup_t8xx_jump_p32,    // 137

      /// fixup_t8xx_pcrel_sym load
      fixup_t8xx_pcrel_sym,   // 138

      fixup_t8xx_pcrel_sym_p8,   // 139
      fixup_t8xx_pcrel_sym_p12,   // 140
      fixup_t8xx_pcrel_sym_p16,   // 141
      fixup_t8xx_pcrel_sym_p20,   // 142
      fixup_t8xx_pcrel_sym_p24,   // 143
      fixup_t8xx_pcrel_sym_p28,   // 144
      fixup_t8xx_pcrel_sym_p32,   // 145

      // Marker
      LastTargetFixupKind,
      NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
    };
  }
}

#endif
