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
#include "OutputSections.h"
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
  T8xx(Ctx &);
  uint32_t calcEFlags() const override;
  RelExpr getRelExpr(RelType type, const Symbol &s,
                     const uint8_t *loc) const override;
  bool needsThunk(RelExpr expr, RelType type, const InputFile *file,
                  uint64_t branchAddr, const Symbol &s,
                  int64_t a) const override;
  void relocate(uint8_t *loc, const Relocation &rel,
                uint64_t val) const override;

  void relocateAlloc(InputSectionBase &sec, uint8_t *buf) const override;

};
} // namespace

T8xx::T8xx(Ctx &ctx) : TargetInfo(ctx)
{
  needsThunks = true;
}


RelExpr T8xx::getRelExpr(RelType type, const Symbol &s,
                        const uint8_t *loc) const {
  switch (type) {
  case R_T8XX_ADDR:
    return R_ABS;
  case R_T8XX_ADDR_NPFIX:
    return R_ABS;
  case R_T8XX_JUMP:
    return R_PC;
  default:
    Err(ctx) << getErrorLoc(ctx, loc) << "unknown relocation (" << type.v
	     << ") against symbol " << &s;
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
  printf ("Relocation Type %i  Value %x\n", rel.type, val);

  switch (rel.type) {
  case R_T8XX_NONE:
    break;
  case R_T8XX_ADDR:
    // Fill in the prefixes
    for (int i = 0; i < 8; ++i)
      loc[i] = (loc[i] & 0xF0) | ((val >> (7-i)*4) & 0xF);

    //    checkUInt(loc, val, 32, rel);
    //    *loc = val & 0xff;
    break;
  case R_T8XX_JUMP:
    // TODO: Negative relocations are feasible for the transputer
    // See if another check should be implemented
    //    checkUInt(loc, val, 32, rel);
    //    *loc = (val >> 8) & 0xff;

    // TODO: Offset for the pfix/nfix instructions before the
    // actual jump instruction. Needs to be adapted when the
    // relaxation is operational
    val -= 8;

    // Fill in the prefixes
    for (int i = 0; i < 8; ++i)
      loc[i] = (loc[i] & 0xF0) | ((val >> (7-i)*4) & 0xF);

    //    write32(loc, 0x12345678);
    break;
  case R_T8XX_ADDR_NPFIX:
    // Fill in and address, which is not based on prefixe
    // Example is a reference to a place in the data section
    for (int i = 0; i < 4; ++i)
      loc[i] = ((val >> i*8) & 0xFF);
    break;

  default:
    llvm_unreachable("unknown relocation");
  }
}


void T8xx::relocateAlloc(InputSectionBase &sec, uint8_t *buf) const {
  uint64_t secAddr = sec.getOutputSection()->addr;
  if (auto *s = dyn_cast<InputSection>(&sec))
    secAddr += s->outSecOff;

  printf ("relocateAlloc\n");

  for (const Relocation &rel : sec.relocs()) {
    uint8_t *loc = buf + rel.offset;
    const uint64_t val = SignExtend64(
        sec.getRelocTargetVA(ctx, rel, secAddr + rel.offset),
        32);

    printf ("secAddr %li  Offset %li  relalloc %li\n", secAddr, rel.offset, val);

    relocate(loc, rel, val);

    /*
    switch (rel.expr) {
    case R_RELAX_TLS_GD_TO_IE_GOT_OFF:
      relaxTlsGdToIe(loc, rel, val);
      break;
    case R_RELAX_TLS_GD_TO_LE:
      relaxTlsGdToLe(loc, rel, val);
      break;
    case R_RELAX_TLS_LD_TO_LE_ABS:
      relaxTlsLdToLe(loc, rel, val);
      break;
    case R_RELAX_TLS_IE_TO_LE:
      relaxTlsIeToLe(loc, rel, val);
      break;
    default:
      relocate(loc, rel, val);
      break;
    }
    */
  }
}


/*
TargetInfo *elf::getT8xxTargetInfo() {
  static T8xx target;
  return &target;
}
*/

static uint32_t getEFlags(InputFile *file) {
  return cast<ObjFile<ELF32LE>>(file)->getObj().getHeader().e_flags;
}

uint32_t T8xx::calcEFlags() const {
  assert(!ctx.objectFiles.empty());

  uint32_t target = getEFlags(ctx.objectFiles.front());

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

  return target;
}

void elf::setT8xxTargetInfo(Ctx &ctx) {
  ctx.target.reset(new T8xx(ctx));
}
