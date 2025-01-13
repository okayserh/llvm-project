//===-- T8xxTargetMachine.cpp - Define TargetMachine for T8xx -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

#include "T8xxTargetMachine.h"
#include "T8xx.h"
#include "T8xxTargetObjectFile.h"
#include "TargetInfo/T8xxTargetInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/MC/TargetRegistry.h"
#include <optional>
using namespace llvm;

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeT8xxTarget() {
  // Register the target.
  RegisterTargetMachine<T8xxTargetMachine> X(getTheT8xxTarget());

  PassRegistry &PR = *PassRegistry::getPassRegistry();
  initializeT8xxDAGToDAGISelLegacyPass(PR);
  initializeT8xxStackPassPass(PR);
  initializeT8xxMoveConstPassPass(PR);
}

static std::string computeDataLayout(const Triple &T) {
  // Note: This needs to be equivalent to the target string that
  // is provided in clang/lib/Basic/Targets/T8xx.h (method T8xxTargetInfo)
  
  // T8xx is typically little endian
  std::string Ret = "e";
  Ret += "-m:e";

  // Some ABIs have 32bit pointers.
  Ret += "-p:32:32:32";

  // Alignments
  // 8 bit can be accessed by special instructions,
  // Unaligned 16 bit integers need substantial effort to convert and retrieve
  // 32 bit is natural alignment
  Ret += "-i8:8:8-i16:32:32-i32:32:32"; // 32 bit integers should naturally be aligned 32 bit

  // On T8xx 128 floats are aligned to 128 bits, on others only to 64.
  // On T8xxV9 registers can hold 64 or 32 bits, on others only 32.
  Ret += "-f64:32:32-f32:32-n32";

  Ret += "-S32";

  return Ret;
}

static Reloc::Model getEffectiveRelocModel(std::optional<Reloc::Model> RM) {
  return RM.value_or(Reloc::Static);
}

// Code models. Some only make sense for 64-bit code.
//
// SunCC  Reloc   CodeModel  Constraints
// abs32  Static  Small      text+data+bss linked below 2^32 bytes
// abs44  Static  Medium     text+data+bss linked below 2^44 bytes
// abs64  Static  Large      text smaller than 2^31 bytes
// pic13  PIC_    Small      GOT < 2^13 bytes
// pic32  PIC_    Medium     GOT < 2^32 bytes
//
// All code models require that the text segment is smaller than 2GB.
static CodeModel::Model
getEffectiveT8xxCodeModel(std::optional<CodeModel::Model> CM, Reloc::Model RM, bool JIT) {
  if (CM) {
    if (*CM == CodeModel::Tiny)
      report_fatal_error("Target does not support the tiny CodeModel", false);
    if (*CM == CodeModel::Kernel)
      report_fatal_error("Target does not support the kernel CodeModel", false);
    return *CM;
  }
  return CodeModel::Small;
}

/// Create an ILP32 architecture model
T8xxTargetMachine::T8xxTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       std::optional<Reloc::Model> RM,
                                       std::optional<CodeModel::Model> CM,
                                       CodeGenOptLevel OL, bool JIT)
    : CodeGenTargetMachineImpl(T, computeDataLayout(TT), TT, CPU, FS, Options,
                        getEffectiveRelocModel(RM),
                        getEffectiveT8xxCodeModel(
                            CM, getEffectiveRelocModel(RM), JIT),
                        OL),
      TLOF(std::make_unique<T8xxELFTargetObjectFile>()),
      Subtarget(TT, std::string(CPU), std::string(FS), *this) {
  initAsmInfo();
}

T8xxTargetMachine::~T8xxTargetMachine() = default;

const T8xxSubtarget *
T8xxTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU =
      CPUAttr.isValid() ? CPUAttr.getValueAsString().str() : TargetCPU;
  std::string FS =
      FSAttr.isValid() ? FSAttr.getValueAsString().str() : TargetFS;

  // FIXME: This is related to the code below to reset the target options,
  // we need to know whether or not the soft float flag is set on the
  // function, so we can enable it as a subtarget feature.
  bool softFloat = F.getFnAttribute("use-soft-float").getValueAsBool();

  if (softFloat)
    FS += FS.empty() ? "+soft-float" : ",+soft-float";

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = std::make_unique<T8xxSubtarget>(TargetTriple, CPU, FS, *this);
  }
  return I.get();
}

namespace {
/// T8xx Code Generator Pass Configuration Options.
class T8xxPassConfig : public TargetPassConfig {
public:
  T8xxPassConfig(T8xxTargetMachine &TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  T8xxTargetMachine &getT8xxTargetMachine() const {
    return getTM<T8xxTargetMachine>();
  }

  FunctionPass *createTargetRegisterAllocator(bool) override;
  
  void addIRPasses() override;
  bool addInstSelector() override;

  //  void addPreRegAlloc() override;

  void addPostRegAlloc() override;
  void addPreEmitPass() override;


  // No reg alloc
  bool addRegAssignAndRewriteFast() override { return false; }

  // No reg alloc
  bool addRegAssignAndRewriteOptimized() override { return false; }

};
} // namespace

TargetPassConfig *T8xxTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new T8xxPassConfig(*this, PM);
}

// OKH, Taken from WebAssembly. See what comes out ...
FunctionPass *T8xxPassConfig::createTargetRegisterAllocator(bool) {
  return nullptr; // No reg alloc
}

void T8xxPassConfig::addIRPasses() {
  addPass(createAtomicExpandLegacyPass());

  TargetPassConfig::addIRPasses();
}

bool T8xxPassConfig::addInstSelector() {
  addPass(createT8xxISelDag(getT8xxTargetMachine()));
  return false;
}

// This should move LDC instructions to the appropriate blocks
/*
void T8xxPassConfig::addPreRegAlloc() {
  printf ("Added T8xx Move Const Pass\n");
  addPass(createT8xxMoveConstPass());
}
*/

void T8xxPassConfig::addPostRegAlloc() {
  // TODO: Initially intended to do an allocation of the
  // processor stack registers.
  // Not working and seems to lead to problems.
  printf ("Added T8xx Stack Pass\n");
  addPass(createT8xxStackPass());
}


void T8xxPassConfig::addPreEmitPass(){
}

