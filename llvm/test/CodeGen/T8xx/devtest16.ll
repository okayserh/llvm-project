; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test16(i32 noundef %0, i32 noundef %1) #0 {
  %3 = alloca i32, align 4
  %4 = alloca i32, align 4
  %5 = alloca i16, align 2
  %6 = alloca i32, align 4
  %7 = alloca i8, align 1
  store i32 %0, i32* %3, align 4
  store i32 %1, i32* %4, align 4
  %8 = load i32, i32* %3, align 4
  %9 = mul nsw i32 %8, 2
  %10 = trunc i32 %9 to i16
  store i16 %10, i16* %5, align 2
  %11 = load i16, i16* %5, align 2
  %12 = sext i16 %11 to i32
  %13 = load i32, i32* %4, align 4
  %14 = add nsw i32 %12, %13
  %15 = trunc i32 %14 to i16
  store i16 %15, i16* %5, align 2
  %16 = load i16, i16* %5, align 2
  %17 = sext i16 %16 to i32
  store i32 %17, i32* %6, align 4
  %18 = load i32, i32* %6, align 4
  %19 = trunc i32 %18 to i8
  store i8 %19, i8* %7, align 1
  %20 = load i8, i8* %7, align 1
  %21 = sext i8 %20 to i32
  %22 = load i32, i32* %3, align 4
  %23 = mul nsw i32 %21, %22
  ret i32 %23
; CHECK-LABEL: test16:
; CHECK: stl 0
; CHECK: ajw -11
; CHECK: ldl 10
; CHECK: stl 1
; CHECK: ldc 16
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: ldc 17
; CHECK: shl
; CHECK: ldl 2
; CHECK: sra
; CHECK: ldl 9
; CHECK: add
; CHECK: stl 3
; CHECK: ldlp 8
; CHECK: stl 4
; CHECK: ldl 3
; CHECK: ldc 8
; CHECK: shr
; CHECK: ldl 4
; CHECK: ldc 1
; CHECK: or
; CHECK: sb
; CHECK: ldl 3
; CHECK: ldl 4
; CHECK: sb
; CHECK: ldl 3
; CHECK: ldlp 6
; CHECK: adc 3
; CHECK: sb
; CHECK: ldc 24
; CHECK: stl 5
; CHECK: ldl 3
; CHECK: ldl 2
; CHECK: shl
; CHECK: ldl 2
; CHECK: sra
; CHECK: stl 7
; CHECK: ldl 3
; CHECK: ldl 5
; CHECK: shl
; CHECK: ldl 5
; CHECK: sra
; CHECK: ldl 1
; CHECK: mul
; CHECK: ajw 11
; CHECK: ldl 0
; CHECK: gcall
}
