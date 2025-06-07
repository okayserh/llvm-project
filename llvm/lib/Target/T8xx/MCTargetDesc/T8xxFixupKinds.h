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
      // A 32-bit T8xx fixup (adjust a generic 32 bit address) = 128
      fixup_t8xx_32 = FirstTargetFixupKind,

      // A 16-bit address (adjust a generic 16 bit address)  = 129
      fixup_t8xx_16,
      
      // fixup_t8xx_addr - global address to some symbol   = 130
      fixup_t8xx_addr,

      /// fixup_t8xx_jump - global symbol to jump or conditional jump  = 131
      /// branches
      fixup_t8xx_jump,

      /// fixup_t8xx_addr_npfix - global address to symbol, not needing  = 132
      /// prefix / postfix transputer instruction
      fixup_t8xx_addr_npfix,

      // Marker
      LastTargetFixupKind,
      NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
    };
  }
}

#endif
