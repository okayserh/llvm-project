; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test7(i32, i32) #0 {
  %3 = alloca i32, align 4
  %4 = alloca i32, align 4
  %5 = alloca i32, align 4
  %6 = alloca i32, align 4
  %7 = alloca i32, align 4
  store i32 %0, i32* %3, align 4
  store i32 %1, i32* %4, align 4
  %8 = load i32, i32* %3, align 4
  %9 = mul nsw i32 %8, 2
  store i32 %9, i32* %5, align 4
  %10 = load i32, i32* %5, align 4
  %11 = load i32, i32* %4, align 4
  %12 = add nsw i32 %10, %11
  store i32 %12, i32* %5, align 4
  %13 = load i32, i32* %5, align 4
  store i32 %13, i32* %6, align 4
  %14 = load i32, i32* %6, align 4
  store i32 %14, i32* %7, align 4
  %15 = load i32, i32* %7, align 4
  %16 = load i32, i32* %3, align 4
  %17 = mul nsw i32 %15, %16
  ret i32 %17
; CHECK-LABEL: test7:
; CHECK: stl 0
; CHECK: ajw -8
; CHECK: ldl 7
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: ldc 1
; CHECK: shl
; CHECK: ldl 6
; CHECK: add
; CHECK: stl 2
; CHECK: ldl 2
; CHECK: stl 5
; CHECK: ldl 2
; CHECK: stl 4
; CHECK: ldl 2
; CHECK: stl 3
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: mul
; CHECK: ajw 8
; CHECK: ldl 0
; CHECK: gcall
}
