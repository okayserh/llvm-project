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
  bool relaxOnce(int pass) const override;
  void finalizeRelax(int passes) const override;

};
} // end anonymous namespace


T8xx::T8xx(Ctx &ctx) : TargetInfo(ctx)
{
  // TODO: Unclear what this is good for. However, it
  // seems to prevent the relaxation from being started
  //  needsThunks = true;
}


RelExpr T8xx::getRelExpr(RelType type, const Symbol &s,
                        const uint8_t *loc) const {
  switch (type) {
  case R_T8XX_ADDR:
    return R_ABS;
  case R_T8XX_ADDR_NPFIX:
    return R_ABS;
  case R_T8XX_ADDR_BASE:
    return R_ABS;
  case R_T8XX_ADDR_ADD:
    return R_ABS;
  case R_T8XX_ADDR_SUB:
    return R_ABS;
  case R_T8XX_JUMP:
    return R_PC;
  case R_T8XX_LDPI_SYM:
    return R_PC;
  case R_T8XX_ALIGN:
    return R_ABS;
  default:
    Err(ctx) << getErrorLoc(ctx, loc) << "unknown relocation (" << type.v
	     << ") against symbol " << &s;
    return R_NONE;
  }
}


bool T8xx::needsThunk(RelExpr expr, RelType type, const InputFile *file,
                     uint64_t branchAddr, const Symbol &s, int64_t a) const {
  return false;
}

//#define NO_T8XX_RELAX

#ifndef NO_T8XX_RELAX
// Filling prefix instructions with relaxation
static void fill_pnfix (uint8_t *loc, int32_t imm, uint8_t opcode, uint32_t len)
{
  int indx = 0;

  // Add pfix and nfix as required
  int i = 7;
  uint32_t imm_dec = (imm < 0 ? (~imm) : imm) & 0xFFFFFFFFu;
  uint32_t imm_and = 0xF0000000u;
  bool enc_beg = false;

  for (; i > 0; --i)
    {
      // Determine 4 bits for pfix/nfix command
      uint32_t imm_res = imm_dec & imm_and;
      imm_and >>= 4;

      // If we had some nonzero bits before
      // continue padding with "pfix" instructions
      if (enc_beg)
	loc[indx++] = static_cast<uint8_t> (0x20 | (imm_res >> (4 * i)));

      // First nonzero bits discovered
      if ((i < len) && !enc_beg)
	{
	  enc_beg = true;
	  if (imm < 0)
	    {
	      loc[indx++] = static_cast<uint8_t> (0x60 | (imm_res >> (4 * i)));
	      imm_dec = ~imm_dec;
	    }
	  else
	    loc[indx++] = static_cast<uint8_t> (0x20 | (imm_res >> (4 * i)));
	}
    }

  loc[indx] = (opcode & 0xF0) | (imm_dec & 0xF);

  // Debug output
  /*
  for (i = 0; i <= indx; ++i)
    printf ("%02x ", loc[i]);
  printf ("\n");
  */
}

#else
static void fill_pnfix (uint8_t *loc, int32_t imm, uint8_t opcode, uint32_t len)
{
  int indx = 0;

  uint32_t *p_imm = (uint32_t *)&imm;

  // Add pfix and nfix as required
  int i = 7;
  uint32_t imm_dec = *p_imm;
  uint32_t imm_and = 0xF0000000u;

  for (; i > 0; --i)
    {
      // Determine 4 bits for pfix/nfix command
      uint32_t imm_res = imm_dec & imm_and;
      imm_and >>= 4;
      loc[indx++] = static_cast<uint8_t> (0x20 | (imm_res >> (4 * i)));
    }

  loc[indx] = (opcode & 0xF0) | (imm_dec & 0xF);

  // Debug output
  for (i = 0; i <= indx; ++i)
    printf ("%02x ", loc[i]);
  printf ("\n");
}
#endif


// This function is for absolute value, like in binary expressions
// or global addresses
static uint32_t calc_pfix_len_abs (const int64_t val)
{
  uint32_t req_bytes = 8;

#ifdef NO_T8XX_RELAX
  return (req_bytes);
#endif

  if (isInt<29>(val))
    req_bytes = 7;
  if (isInt<25>(val))
    req_bytes = 6;
  if (isInt<21>(val))
    req_bytes = 5;
  if (isInt<17>(val))
    req_bytes = 4;
  if (isInt<13>(val))
    req_bytes = 3;
  if (isInt<9>(val))
    req_bytes = 2;
  if ((val >= 0) && isUInt<4>((uint64_t)val))
    req_bytes = 1;
  return (req_bytes);
}


// Note: Displacement is calculated from beginning of instruction
// Must be adjusted by the instruction length for the transputer
// Note: +4 bit are possible in 1 byte
// +8 bit 2 bytes
// -4 bit 2 bytes
// -8 bit 2 bytes

// This function is for pc relative instructions, like jumps
// The instruction length is deducted from the value
static uint32_t calc_pfix_len_pcrel (const int64_t val)
{
  uint32_t req_bytes = 8;

  // TODO: It seems like the resulting code
  // 20 af for a jump distance of 17 (from the
  // beginning of the instruction) does not
  // make much sense.
  // Check, whether this could be implemented more
  // intelligent.

#ifdef NO_T8XX_RELAX
  return (req_bytes);
#endif
  if (isInt<29>(val - 7))
    req_bytes = 7;
  if (isInt<25>(val - 6))
    req_bytes = 6;
  if (isInt<21>(val - 5))
    req_bytes = 5;
  if (isInt<17>(val - 4))
    req_bytes = 4;
  if (isInt<13>(val - 3))
    req_bytes = 3;
  if (isInt<9>(val - 2))
    req_bytes = 2;
  if ((val >= 1) && isUInt<4>((uint64_t)val - 1))
    req_bytes = 1;
  return (req_bytes);
}

void T8xx::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const {
  //  printf ("Relocation Type %i  Value %x\n", rel.type, val);

  switch (rel.type) {
  case R_T8XX_ALIGN:
  case R_T8XX_NONE:
    break;

  case R_T8XX_ADDR:
    // Fill in the prefixes
    for (int i = 0; i < 8; ++i)
      loc[i] = (loc[i] & 0xF0) | ((val >> (7-i)*4) & 0xF);
    break;

    // Base is not relocated. Only the add/sub relocations
  case R_T8XX_ADDR_BASE:
    break;
  case R_T8XX_ADDR_ADD:
  case R_T8XX_ADDR_SUB:
    {
      //      printf ("relocate: ADDR_BASE,ADD_SUB\n");
      int32_t sval = SignExtend32 ((uint32_t)(rel.addend & 0xFFFFFFFF), 32);
      uint32_t len = calc_pfix_len_abs (rel.addend);
      //      printf ("Rel-Type %i  VAL %lu  SVal %i  Len %lu\n", rel.type, val, sval, len);
      fill_pnfix (loc, sval, loc[len-1], len);
    }
    break;

  case R_T8XX_JUMP:
    {
      int32_t sval = SignExtend32 ((uint32_t)(val & 0xFFFFFFFF), 32);
      uint32_t len = calc_pfix_len_pcrel (val);
      //      printf ("J/CJ, Len : %u  ", len);
      fill_pnfix (loc, sval - len, loc[len-1], len);
    }
    break;

  case R_T8XX_LDPI_SYM:
    {
      int32_t sval = SignExtend32 ((uint32_t)(val & 0xFFFFFFFF), 32);
      uint32_t len = calc_pfix_len_pcrel (val - 2);
      // The "-2" is two bytes for the "ldpi" instruction after the ldc.
      fill_pnfix (loc, sval - 2 - len, loc[len-1], len);
    }
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

  for (const Relocation &rel : sec.relocs()) {
    uint8_t *loc = buf + rel.offset;
    uint64_t val = sec.getRelocTargetVA(ctx, rel, secAddr + rel.offset);

    //    printf ("secAddr %08x  Offset %li  relalloc %08x\n", secAddr, rel.offset, val);
    relocate(loc, rel, val);
  }
}


// TODO
static void relaxNPFix(Ctx &ctx, const InputSection &sec, size_t i, uint64_t loc,
                      Relocation &r, uint32_t &remove) {
  const Symbol &sym = *r.sym;
  const uint64_t insnPair = read64le(sec.content().data() + r.offset);
  const uint64_t dest = sym.getVA(ctx) + r.addend;
  const int64_t displace = dest - loc;

  //printf ("relaxNPFix Displace %li\n", displace);
  /*
  printf ("Symbol %s\n", toStr(ctx, sym).c_str ());
  printf ("Sym Type %i  bind %i\n", sym.type, sym.binding);
  printf ("Sym Value %08x   Loc %08x    Dest  %08x\n\n", sym.getVA(ctx), loc, r.addend);
  */

  uint32_t req_bytes = calc_pfix_len_pcrel (dest);

  remove = 0;

  sec.relaxAux->writes.push_back(0x0); // Dummy value to keep array indices in sync
  sec.relaxAux->relocTypes[i] = r.type;
}


// Note: This is used to relax binary expressions.
// These are represented by two relocations at the same offset
// The first one is R_T8XX_ADDR_BASE, the second one either
// R_T8XX_ADDR_ADD or R_T8XX_ADDR_SUB

static void relaxBinary(Ctx &ctx, const InputSection &sec, size_t i, uint64_t loc,
                      Relocation &r, uint32_t &remove) {
  const Symbol &sym = *r.sym;
  const uint64_t dest = sym.getVA(ctx, r.addend);
  const int64_t displace = dest - loc;
  uint32_t req_bytes = 8;

  //  printf ("Sym Value %08x   Loc %08x  Add  %08x   Dest %08x\n", sym.getVA(ctx), loc, r.addend, dest);
  if (r.type == R_T8XX_ADDR_BASE)
    {
      sec.relaxAux->writes.push_back((uint32_t) dest);
      SmallVector<uint32_t>::iterator end_it = sec.relaxAux->writes.end();
      end_it--;
      remove = 0;
    }

  if ((r.type == R_T8XX_ADDR_SUB) ||
      (r.type == R_T8XX_ADDR_ADD))
    {
      SmallVector<uint32_t>::iterator end_it = sec.relaxAux->writes.end();
      end_it--;
      uint32_t base = *end_it;
      //      printf ("Bin Dest %08x Base %08x Bin Exp %08x\n", dest, base, base - dest);
      uint32_t req_bytes = calc_pfix_len_abs (displace);
      remove = 8 - req_bytes;

      // Put the resulting value into the "writes" field.
      if (r.type == R_T8XX_ADDR_SUB)
	sec.relaxAux->writes.push_back((uint32_t) base - dest);
      else
	sec.relaxAux->writes.push_back((uint32_t) base + dest);
      sec.relaxAux->relocTypes[i] = r.type;
    }
}


static void relaxLDPI(Ctx &ctx, const InputSection &sec, size_t i, uint64_t loc,
                      Relocation &r, uint32_t &remove) {
  const Symbol &sym = *r.sym;
  const uint64_t dest = sym.getVA(ctx) + r.addend;
  const int64_t displace = dest - loc;

  uint32_t req_bytes = calc_pfix_len_pcrel (displace - 2);
  /*
  printf ("Sym Value %08x   Loc %08x    Dest  %08x\n", sym.getVA(ctx), loc, r.addend);
  printf ("relaxLDPI Displace %li\n", displace);
  printf ("Current size %li\n", sec.size);
  printf ("INSN %#018"PRIx64"\n", insnPair);
  printf ("Symbol %s\n\n", toStr(ctx, sym).c_str ());
  */

  // Relocation is selected based on required bytes
  remove = 8 - req_bytes;
  if (req_bytes < 8)
    sec.relaxAux->relocTypes[i] = r.type;

  sec.relaxAux->writes.push_back(0x0); // Dummy value to keep array indices in sync
}


static void relaxJump(Ctx &ctx, const InputSection &sec, size_t i, uint64_t loc,
                      Relocation &r, uint32_t &remove) {
  const Symbol &sym = *r.sym;
  const uint64_t dest = sym.getVA(ctx) + r.addend;
  const int64_t displace = dest - loc;

  uint32_t req_bytes = calc_pfix_len_pcrel (displace);

  printf ("relaxJump Displace %li\n", displace);

  // Relocation is kept as it is
  remove = 8 - req_bytes;
  if (req_bytes < 8)
    sec.relaxAux->relocTypes[i] = r.type;

  sec.relaxAux->writes.push_back(0x0); // Dummy value to keep array indices in sync
}


static bool relax(Ctx &ctx, InputSection &sec) {
  const uint64_t secAddr = sec.getVA();
  const MutableArrayRef<Relocation> relocs = sec.relocs();
  auto &aux = *sec.relaxAux;
  bool changed = false;
  ArrayRef<SymbolAnchor> sa = ArrayRef(aux.anchors);
  uint64_t delta = 0;
  bool tlsdescRelax = false, toLeShortForm = false;

  std::fill_n(aux.relocTypes.get(), relocs.size(), R_RISCV_NONE);
  aux.writes.clear();
  for (auto [i, r] : llvm::enumerate(relocs)) {
    const uint64_t loc = secAddr + r.offset - delta;
    uint32_t &cur = aux.relocDeltas[i], remove = 0;
    switch (r.type) {
    case R_T8XX_ALIGN: {
      const uint64_t nextLoc = loc + r.addend;
      const uint64_t align = PowerOf2Ceil(r.addend);
      // All bytes beyond the alignment boundary should be removed.
      remove = nextLoc - ((loc + align - 1) & -align);
      // If we can't satisfy this alignment, we've found a bad input.
      if (LLVM_UNLIKELY(static_cast<int32_t>(remove) < 0)) {
        Err(ctx) << getErrorLoc(ctx, (const uint8_t *)loc)
                 << "insufficient padding bytes for " << r.type << ": "
                 << r.addend
                 << " bytes available "
                    "for requested alignment of "
                 << align << " bytes";
        remove = 0;
      }
      aux.writes.push_back (0x0);  // Dummy value to keep array indices in sync
      break;
    }

      // T8XX_ADDR is a global absolute address, used in a LDC command for example
    case R_T8XX_ADDR:
      relaxNPFix(ctx, sec, i, loc, r, remove);
      break;

    case R_T8XX_JUMP:
      relaxJump(ctx, sec, i, loc, r, remove);
      break;

    case R_T8XX_ADDR_BASE:
    case R_T8XX_ADDR_ADD:
    case R_T8XX_ADDR_SUB:
      relaxBinary(ctx, sec, i, loc, r, remove);
      break;

    case R_T8XX_LDPI_SYM:
      relaxLDPI(ctx, sec, i, loc, r, remove);
      break;

    default:
      aux.writes.push_back (0x0);  // Dummy value to keep array indices in sync
    }

    // For all anchors whose offsets are <= r.offset, they are preceded by
    // the previous relocation whose `relocDeltas` value equals `delta`.
    // Decrease their st_value and update their st_size.
    for (; sa.size() && sa[0].offset <= r.offset; sa = sa.slice(1)) {
      if (sa[0].end)
        sa[0].d->size = sa[0].offset - delta - sa[0].d->value;
      else
        sa[0].d->value = sa[0].offset - delta;
    }
    delta += remove;
    if (delta != cur) {
      cur = delta;
      changed = true;
    }
  }

  for (const SymbolAnchor &a : sa) {
    if (a.end)
      a.d->size = a.offset - delta - a.d->value;
    else
      a.d->value = a.offset - delta;
  }

  // Inform assignAddresses that the size has changed.
  if (!isUInt<32>(delta))
    Fatal(ctx) << "section size decrease is too large: " << delta;
  sec.bytesDropped = delta;
  return changed;
}


// When relaxing just R_RISCV_ALIGN, relocDeltas is usually changed only once in
// the absence of a linker script. For call and load/store R_RISCV_RELAX, code
// shrinkage may reduce displacement and make more relocations eligible for
// relaxation. Code shrinkage may increase displacement to a call/load/store
// target at a higher fixed address, invalidating an earlier relaxation. Any
// change in section sizes can have cascading effect and require another
// relaxation pass.
bool T8xx::relaxOnce(int pass) const {
  //  llvm::TimeTraceScope timeScope("RISC-V relaxOnce");
  if (ctx.arg.relocatable)
    return false;

  printf ("LLD.relaxOnce Pass: %i\n", pass);

  if (pass == 0)
    initSymbolAnchors(ctx);

  SmallVector<InputSection *, 0> storage;
  bool changed = false;
  for (OutputSection *osec : ctx.outputSections) {
    if (!(osec->flags & SHF_EXECINSTR))
      continue;
    for (InputSection *sec : getInputSections(*osec, storage))
      changed |= relax(ctx, *sec);
  }
  return changed;
}

void T8xx::finalizeRelax(int passes) const {
  //  llvm::TimeTraceScope timeScope("Finalize RISC-V relaxation");
  Log(ctx) << "relaxation passes: " << passes;
  SmallVector<InputSection *, 0> storage;

  for (OutputSection *osec : ctx.outputSections) {
    if (!(osec->flags & SHF_EXECINSTR))
      continue;
    for (InputSection *sec : getInputSections(*osec, storage)) {
      RelaxAux &aux = *sec->relaxAux;
      if (!aux.relocDeltas)
        continue;

      MutableArrayRef<Relocation> rels = sec->relocs();
      ArrayRef<uint8_t> old = sec->content();
      size_t newSize = old.size() - aux.relocDeltas[rels.size() - 1];
      size_t writesIdx = 0;
      uint8_t *p = ctx.bAlloc.Allocate<uint8_t>(newSize);
      uint64_t offset = 0;
      int64_t delta = 0;
      sec->content_ = p;
      sec->size = newSize;
      sec->bytesDropped = 0;

      // Update section content: remove NOPs for R_RISCV_ALIGN and rewrite
      // instructions for relaxed relocations.
      for (size_t i = 0, e = rels.size(); i != e; ++i) {
        uint32_t remove = aux.relocDeltas[i] - delta;
        delta = aux.relocDeltas[i];
        if (remove == 0 && aux.relocTypes[i] == R_T8XX_NONE)
          continue;

        // Copy from last location to the current relocated location.
        const Relocation &r = rels[i];
        uint64_t size = r.offset - offset;
        memcpy(p, old.data() + offset, size);
        p += size;

        // For R_RISCV_ALIGN, we will place `offset` in a location (among NOPs)
        // to satisfy the alignment requirement. If both `remove` and r.addend
        // are multiples of 4, it is as if we have skipped some NOPs. Otherwise
        // we are in the middle of a 4-byte NOP, and we need to rewrite the NOP
        // sequence.
	int64_t skip = 0;
	/*
        if (r.type == R_T8XX_ALIGN) {
          if (remove % 4 || r.addend % 4) {
            skip = r.addend - remove;
            int64_t j = 0;
            for (; j + 4 <= skip; j += 4)
              write32le(p + j, 0x00000013); // nop
            if (j != skip) {
              assert(j + 2 == skip);
              write16le(p + j, 0x0001); // c.nop
            }
          }
	  } else*/

	if (RelType newType = aux.relocTypes[i]) {
	  // TODO: Here the code for putting in the difference in case of binary
	  // relocations needs to be filled in.
	  switch (newType)
	    {
	    case R_T8XX_ADDR_ADD:
	    case R_T8XX_ADDR_SUB:
	      {
		uint32_t len = calc_pfix_len_abs (aux.writes[i]);
		printf ("Finalize SUB %i   Size %i  Rem %i\n", aux.writes[i], size, remove);
		rels[i].addend = aux.writes[i];
	      }
	      break;
	    }
	}

        p += skip;
        offset = r.offset + skip + remove;
      }
      memcpy(p, old.data() + offset, old.size() - offset);

      // Subtract the previous relocDeltas value from the relocation offset.
      // For a pair of R_RISCV_CALL/R_RISCV_RELAX with the same offset, decrease
      // their r_offset by the same delta.
      delta = 0;
      for (size_t i = 0, e = rels.size(); i != e;) {
        uint64_t cur = rels[i].offset;
        do {
          rels[i].offset -= delta;
          if (aux.relocTypes[i] != R_RISCV_NONE)
            rels[i].type = aux.relocTypes[i];
        } while (++i != e && rels[i].offset == cur);
        delta = aux.relocDeltas[i - 1];
      }
    }
  }
}


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
