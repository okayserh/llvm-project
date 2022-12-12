//===-- T8xxTargetMachine.h - Define TargetMachine for T8xx ---*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the T8xx specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SPARC_SPARCTARGETMACHINE_H
#define LLVM_LIB_TARGET_SPARC_SPARCTARGETMACHINE_H

#include "T8xxInstrInfo.h"
#include "T8xxSubtarget.h"
#include "llvm/Target/TargetMachine.h"
#include <optional>

namespace llvm {

class T8xxTargetMachine : public LLVMTargetMachine {
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  T8xxSubtarget Subtarget;
  bool is64Bit;
  mutable StringMap<std::unique_ptr<T8xxSubtarget>> SubtargetMap;

public:
  T8xxTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                     StringRef FS, const TargetOptions &Options,
                     std::optional<Reloc::Model> RM,
                     std::optional<CodeModel::Model> CM, CodeGenOpt::Level OL,
                     bool JIT, bool is64bit);
  ~T8xxTargetMachine() override;

  const T8xxSubtarget *getSubtargetImpl() const { return &Subtarget; }
  const T8xxSubtarget *getSubtargetImpl(const Function &) const override;

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;
  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }
};

/// T8xx 32-bit target machine
///
class T8xxV8TargetMachine : public T8xxTargetMachine {
  virtual void anchor();

public:
  T8xxV8TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                       StringRef FS, const TargetOptions &Options,
                       std::optional<Reloc::Model> RM,
                       std::optional<CodeModel::Model> CM, CodeGenOpt::Level OL,
                       bool JIT);
};

/// T8xx 64-bit target machine
///
class T8xxV9TargetMachine : public T8xxTargetMachine {
  virtual void anchor();

public:
  T8xxV9TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                       StringRef FS, const TargetOptions &Options,
                       std::optional<Reloc::Model> RM,
                       std::optional<CodeModel::Model> CM, CodeGenOpt::Level OL,
                       bool JIT);
};

class T8xxelTargetMachine : public T8xxTargetMachine {
  virtual void anchor();

public:
  T8xxelTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                       StringRef FS, const TargetOptions &Options,
                       std::optional<Reloc::Model> RM,
                       std::optional<CodeModel::Model> CM, CodeGenOpt::Level OL,
                       bool JIT);
};

} // end namespace llvm

#endif
