//===--- ARC.h - Declare ARC target feature support -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares ARC TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_BASIC_TARGETS_T8XX_H
#define LLVM_CLANG_LIB_BASIC_TARGETS_T8XX_H

#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "llvm/Support/Compiler.h"
#include "llvm/TargetParser/Triple.h"

namespace clang {
namespace targets {

class LLVM_LIBRARY_VISIBILITY T8xxTargetInfo : public TargetInfo {
 protected:
  std::string ABI;

 public:
  T8xxTargetInfo(const llvm::Triple &Triple, const TargetOptions &)
      : TargetInfo(Triple) {
    NoAsmVariants = true;
    LongLongAlign = 32;
    SuitableAlign = 32;
    DoubleAlign = LongDoubleAlign = 32;
    SizeType = UnsignedInt;
    PtrDiffType = SignedInt;
    IntPtrType = SignedInt;

    setABI("txl");

    //    UseZeroLengthBitfieldAlignment = true;
    resetDataLayout("e-m:e-p:32:32:32-i8:8:8-i16:16:16-"
                    "i32:32:32-i64:32:32-f64:32:32-f32:32:32-n32-S32");
  }


  StringRef getABI() const override { return ABI; }

  bool setABI(const std::string &Name) override {
    if (Name == "txl")   // ABI name for the LLVM / ELF convention.
      {
	ABI = Name;

	// TODO: Initial guess. Need checking
	Int64Type = SignedLongLong;
	IntMaxType = Int64Type;
	LongDoubleFormat = &llvm::APFloat::IEEEdouble();
	LongDoubleWidth = 64;
	LongDoubleAlign = 32;
	LongWidth = LongAlign = 32;
	MaxAtomicPromoteWidth = MaxAtomicInlineWidth = 32;
	PointerWidth = PointerAlign = 32;
	return (true);
      }
    return false;
  }
  
  void getTargetDefines(const LangOptions &Opts,
                        MacroBuilder &Builder) const override;

  llvm::SmallVector<Builtin::InfosShard> getTargetBuiltins() const override {
    // FIXME: Implement!
    return {};
  }

  BuiltinVaListKind getBuiltinVaListKind() const override {
    return TargetInfo::VoidPtrBuiltinVaList;
  }

  std::string_view getClobbers() const override { return ""; }

  ArrayRef<const char *> getGCCRegNames() const override {
    static const char *const GCCRegNames[] = {
        "r0",  "r1",  "r2",  "r3",  "r4",  "r5",     "r6",  "r7",
        "r8",  "r9",  "r10", "r11", "r12", "r13",    "r14", "r15",
        "FAreg", "FBreg", "FCreg"};
    return llvm::ArrayRef(GCCRegNames);
  }

  ArrayRef<TargetInfo::GCCRegAlias> getGCCRegAliases() const override {
    // FIXME: Does not make sens, since there's not real GCC equivalent for T8xx
    static const TargetInfo::GCCRegAlias O32RegAliases[] = {
        {{"r0"}, "r0"},  {{"r1"}, "r1"},         {{"r2"}, "r2"},
        {{"r3"}, "r3"},  {{"r4"}, "r4"},         {{"r5"}, "r5"},
        {{"r6"}, "r6"},  {{"r7"}, "r7"},         {{"r8"}, "r8"},
        {{"r9"}, "r9"},  {{"r10"}, "r10"},       {{"r11"}, "r11"},
        {{"r12"}, "r12"}, {{"r13"}, "r13"},      {{"r14"}, "r14"},
        {{"r15"}, "r15"}, {{"FAreg"}, "FAreg"},  {{"FBreg"}, "FBreg"},
        {{"FCreg"}, "FCreg"}
    };
    return llvm::ArrayRef(O32RegAliases);
  }

  bool validateAsmConstraint(const char *&Name,
                             TargetInfo::ConstraintInfo &Info) const override;

  bool hasBitIntType() const override { return true; }

  bool isCLZForZeroUndef() const override { return false; }
};

} // namespace targets
} // namespace clang

#endif // LLVM_CLANG_LIB_BASIC_TARGETS_T8XX_H
