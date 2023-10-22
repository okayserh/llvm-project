//===- T8xx.cpp ------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// T8xx is a very old processor, which has a rather unconventional
// representation of immediates. Any negative immediates or those larger than
// 0xF need to be constructed from prefix instructions.
// 
// Since it is a baremetal programming, there's usually no loader to load
// ELF files on AVRs. You are expected to link your program against address
// 0 and pull out a .text section from the result using objcopy, so that you
// can write the linked code to on-chip flush memory. You can do that with
// the following commands:
//
//   ld.lld -Ttext=0 -o foo foo.o
//   objcopy -O binary --only-section=.text foo output.bin
//
// Note that the current AVR support is very preliminary so you can't
// link any useful program yet, though.
//
//===----------------------------------------------------------------------===//

#include "InputFiles.h"
#include "Symbols.h"
#include "Target.h"
#include "Thunks.h"
#include "lld/Common/ErrorHandler.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Support/Endian.h"

using namespace llvm;
using namespace llvm::object;
using namespace llvm::support::endian;
using namespace llvm::ELF;
using namespace lld;
using namespace lld::elf;

namespace {
class T8xx final : public TargetInfo {
public:
  T8xx() { needsThunks = true; }
  uint32_t calcEFlags() const override;
  RelExpr getRelExpr(RelType type, const Symbol &s,
                     const uint8_t *loc) const override;
  bool needsThunk(RelExpr expr, RelType type, const InputFile *file,
                  uint64_t branchAddr, const Symbol &s,
                  int64_t a) const override;
  void relocate(uint8_t *loc, const Relocation &rel,
                uint64_t val) const override;
};
} // namespace

RelExpr T8xx::getRelExpr(RelType type, const Symbol &s,
                        const uint8_t *loc) const {
  switch (type) {
  case R_T8XX_ADDR:
    return R_ABS;
  case R_T8XX_JUMP:
    return R_PC;
  default:
    error(getErrorLocation(loc) + "unknown relocation (" + Twine(type) +
          ") against symbol " + toString(s));
    return R_NONE;
  }
}

static void writeLDI(uint8_t *loc, uint64_t val) {
  write16le(loc, (read16le(loc) & 0xf0f0) | (val & 0xf0) << 4 | (val & 0x0f));
}

bool T8xx::needsThunk(RelExpr expr, RelType type, const InputFile *file,
                     uint64_t branchAddr, const Symbol &s, int64_t a) const {
  /*
  switch (type) {
  case R_T8xx_LO8_LDI_GS:
  case R_T8xx_HI8_LDI_GS:
    // A thunk is needed if the symbol's virtual address is out of range
    // [0, 0x1ffff].
    return s.getVA() >= 0x20000;
  default:
    return false;
  }
  */
  return false;
}

void T8xx::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const {
  printf ("Relocation Type %i  Value %i\n", rel.type, val);

  switch (rel.type) {
  case R_T8XX_NONE:
    break;
  case R_T8XX_ADDR:
    checkUInt(loc, val, 32, rel);
    *loc = val & 0xff;
    break;
  case R_T8XX_JUMP:
    // TODO: Negative relocations are feasible for the transputer
    // See if another check should be implemented
    //    checkUInt(loc, val, 32, rel);
    *loc = (val >> 8) & 0xff;
    break;

  default:
    llvm_unreachable("unknown relocation");
  }
}

TargetInfo *elf::getT8xxTargetInfo() {
  static T8xx target;
  return &target;
}

static uint32_t getEFlags(InputFile *file) {
  return cast<ObjFile<ELF32LE>>(file)->getObj().getHeader().e_flags;
}

uint32_t T8xx::calcEFlags() const {
  assert(!ctx.objectFiles.empty());

  uint32_t flags = getEFlags(ctx.objectFiles[0]);

  /*
  bool hasLinkRelaxFlag = flags & EF_T8xx_LINKRELAX_PREPARED;

  for (InputFile *f : ArrayRef(ctx.objectFiles).slice(1)) {
    uint32_t objFlags = getEFlags(f);
    if ((objFlags & EF_T8xx_ARCH_MASK) != (flags & EF_T8xx_ARCH_MASK))
      error(toString(f) +
            ": cannot link object files with incompatible target ISA");
    if (!(objFlags & EF_T8xx_LINKRELAX_PREPARED))
      hasLinkRelaxFlag = false;
  }

  if (!hasLinkRelaxFlag)
    flags &= ~EF_T8xx_LINKRELAX_PREPARED;
  */
  
  return flags;
}
