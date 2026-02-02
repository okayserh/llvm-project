//===- T8xx.cpp -----------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "ABIInfoImpl.h"
#include "TargetInfo.h"

using namespace clang;
using namespace clang::CodeGen;

//===----------------------------------------------------------------------===//
// MIPS ABI Implementation.  This works for both little-endian and
// big-endian variants.
//===----------------------------------------------------------------------===//

namespace {
class T8xxABIInfo : public ABIInfo {
  const unsigned MinABIStackAlignInBytes, StackAlignInBytes;
  /*
  void CoerceToIntArgs(uint64_t TySize,
                       SmallVectorImpl<llvm::Type *> &ArgList) const;
  llvm::Type* HandleAggregates(QualType Ty, uint64_t TySize) const;
  llvm::Type* returnAggregateInRegs(QualType RetTy, uint64_t Size) const;
  llvm::Type* getPaddingType(uint64_t Align, uint64_t Offset) const;
  */
public:
  T8xxABIInfo(CodeGenTypes &CGT) :
    ABIInfo(CGT), MinABIStackAlignInBytes(4),
    StackAlignInBytes(4) {}
  /*
  ABIArgInfo classifyReturnType(QualType RetTy) const;
  ABIArgInfo classifyArgumentType(QualType RetTy, uint64_t &Offset) const;
  */
  void computeInfo(CGFunctionInfo &FI) const override;
  RValue EmitVAArg(CodeGenFunction &CGF, Address VAListAddr, QualType Ty,
                   AggValueSlot Slot) const override;
  //  ABIArgInfo extendType(QualType Ty) const;
};


class T8xxTargetCodeGenInfo : public TargetCodeGenInfo {
  unsigned SizeOfUnwindException;
public:
  T8xxTargetCodeGenInfo(CodeGenTypes &CGT)
      : TargetCodeGenInfo(std::make_unique<T8xxABIInfo>(CGT)),
        SizeOfUnwindException(24) {}

  int getDwarfEHStackPointer(CodeGen::CodeGenModule &CGM) const override {
    return 29;
  }

  void setTargetAttributes(const Decl *D, llvm::GlobalValue *GV,
                           CodeGen::CodeGenModule &CGM) const override {
    const FunctionDecl *FD = dyn_cast_or_null<FunctionDecl>(D);
    if (!FD) return;
    llvm::Function *Fn = cast<llvm::Function>(GV);
  }

  bool initDwarfEHRegSizeTable(CodeGen::CodeGenFunction &CGF,
                               llvm::Value *Address) const override;

  unsigned getSizeOfUnwindException() const override {
    return SizeOfUnwindException;
  }
};


} // End namespace
  
// Based on MipsABIInfo implementation

void T8xxABIInfo::computeInfo(CGFunctionInfo &FI) const {
  ABIArgInfo &RetInfo = FI.getReturnInfo();
  //  if (!getCXXABI().classifyReturnType(FI))
  //    RetInfo = classifyReturnType(FI.getReturnType());

  // Check if a pointer to an aggregate is passed as a hidden argument.
  uint64_t Offset = RetInfo.isIndirect() ? MinABIStackAlignInBytes : 0;
  /*
  for (auto &I : FI.arguments())
    I.info = classifyArgumentType(I.type, Offset);
  */
}

RValue T8xxABIInfo::EmitVAArg(CodeGenFunction &CGF, Address VAListAddr,
                              QualType OrigTy, AggValueSlot Slot) const {
  QualType Ty = OrigTy;

  // Integer arguments are promoted to 32-bit on O32 and 64-bit on N32/N64.
  // Pointers are also promoted in the same way but this only matters for N32.
  unsigned SlotSizeInBits = 32;
  unsigned PtrWidth = getTarget().getPointerWidth(LangAS::Default);
  bool DidPromote = false;
  if ((Ty->isIntegerType() &&
          getContext().getIntWidth(Ty) < SlotSizeInBits) ||
      (Ty->isPointerType() && PtrWidth < SlotSizeInBits)) {
    DidPromote = true;
    Ty = getContext().getIntTypeForBitwidth(SlotSizeInBits,
                                            Ty->isSignedIntegerType());
  }

  auto TyInfo = getContext().getTypeInfoInChars(Ty);

  // The alignment of things in the argument area is never larger than
  // StackAlignInBytes.
  TyInfo.Align =
    std::min(TyInfo.Align, CharUnits::fromQuantity(StackAlignInBytes));

  // MinABIStackAlignInBytes is the size of argument slots on the stack.
  CharUnits ArgSlotSize = CharUnits::fromQuantity(MinABIStackAlignInBytes);

  RValue Res = emitVoidPtrVAArg(CGF, VAListAddr, Ty, /*indirect*/ false, TyInfo,
                                ArgSlotSize, /*AllowHigherAlign*/ true, Slot);

  // If there was a promotion, "unpromote".
  // TODO: can we just use a pointer into a subset of the original slot?
  if (DidPromote) {
    llvm::Type *ValTy = CGF.ConvertType(OrigTy);
    llvm::Value *Promoted = Res.getScalarVal();

    // Truncate down to the right width.
    llvm::Type *IntTy = (OrigTy->isIntegerType() ? ValTy : CGF.IntPtrTy);
    llvm::Value *V = CGF.Builder.CreateTrunc(Promoted, IntTy);
    if (OrigTy->isPointerType())
      V = CGF.Builder.CreateIntToPtr(V, ValTy);

    return RValue::get(V);
  }

  return Res;
}


// TODO: Copied from MIPS. Needs to be adapted to T8xx architecture
bool
T8xxTargetCodeGenInfo::initDwarfEHRegSizeTable(CodeGen::CodeGenFunction &CGF,
                                               llvm::Value *Address) const {
  // This information comes from gcc's implementation, which seems to
  // as canonical as it gets.

  // Everything on MIPS is 4 bytes.  Double-precision FP registers
  // are aliased to pairs of single-precision FP registers.
  llvm::Value *Four8 = llvm::ConstantInt::get(CGF.Int8Ty, 4);

  // 0-31 are the general purpose registers, $0 - $31.
  // 32-63 are the floating-point registers, $f0 - $f31.
  // 64 and 65 are the multiply/divide registers, $hi and $lo.
  // 66 is the (notional, I think) register for signal-handler return.
  AssignToArrayRange(CGF.Builder, Address, Four8, 0, 65);

  // 67-74 are the floating-point status registers, $fcc0 - $fcc7.
  // They are one bit wide and ignored here.

  // 80-111 are the coprocessor 0 registers, $c0r0 - $c0r31.
  // (coprocessor 1 is the FP unit)
  // 112-143 are the coprocessor 2 registers, $c2r0 - $c2r31.
  // 144-175 are the coprocessor 3 registers, $c3r0 - $c3r31.
  // 176-181 are the DSP accumulator registers.
  AssignToArrayRange(CGF.Builder, Address, Four8, 80, 181);
  return false;
}


std::unique_ptr<TargetCodeGenInfo>
CodeGen::createT8xxTargetCodeGenInfo(CodeGenModule &CGM) {
  return std::make_unique<T8xxTargetCodeGenInfo>(CGM.getTypes());
}
