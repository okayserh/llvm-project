; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

; Function Attrs: noinline nounwind optnone uwtable
define dso_local i32 @test19a(i32 noundef %0, i32 noundef %1, i32 noundef %2) #0 {
  %4 = alloca i32, align 4
  %5 = alloca i32, align 4
  %6 = alloca i32, align 4
  %7 = alloca i32, align 4
  %8 = alloca i32, align 4
  %9 = alloca i32, align 4
  store i32 %0, i32* %4, align 4
  store i32 %1, i32* %5, align 4
  store i32 %2, i32* %6, align 4
  %10 = load i32, i32* %4, align 4
  %11 = load i32, i32* %5, align 4
  %12 = mul nsw i32 %10, %11
  store i32 %12, i32* %7, align 4
  %13 = load i32, i32* %7, align 4
  %14 = load i32, i32* %6, align 4
  %15 = mul nsw i32 %13, %14
  store i32 %15, i32* %8, align 4
  %16 = load i32, i32* %8, align 4
  %17 = load i32, i32* %4, align 4
  %18 = mul nsw i32 %16, %17
  store i32 %18, i32* %9, align 4
  %19 = load i32, i32* %9, align 4
  %20 = load i32, i32* %5, align 4
  %21 = mul nsw i32 %19, %20
  ret i32 %21
; CHECK-LABEL: test19a:
; CHECK: stl 0
; CHECK: ajw -12
; CHECK: ldl 11
; CHECK: stl 1
; CHECK: ldl 10
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: ldl 2
; CHECK: mul
; CHECK: stl 3
; CHECK: ldl 3
; CHECK: stl 8
; CHECK: ldl 3
; CHECK: ldl 9
; CHECK: mul
; CHECK: stl 4
; CHECK: ldl 4
; CHECK: stl 7
; CHECK: ldl 4
; CHECK: ldl 1
; CHECK: mul
; CHECK: stl 5
; CHECK: ldl 5
; CHECK: stl 6
; CHECK: ldl 5
; CHECK: ldl 2
; CHECK: mul
; CHECK: ajw 12
; CHECK: ldl 0
; CHECK: gcall
}

; Function Attrs: noinline nounwind optnone uwtable
define dso_local i32 @test19b(i32 noundef %0) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4
  store i32 %0, i32* %2, align 4
  %4 = call i32 @test19a(i32 noundef 13, i32 noundef 12, i32 noundef 14)
  store i32 %4, i32* %3, align 4
  %5 = load i32, i32* %3, align 4
  %6 = mul nsw i32 5, %5
  ret i32 %6
; CHECK-LABEL: test19b:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldc 14
; CHECK: ldlp 4294967293
; CHECK: stnl 0
; CHECK: ldc 12
; CHECK: ldlp 4294967294
; CHECK: stnl 0
; CHECK: ldc 13
; CHECK: ldlp 4294967295
; CHECK: stnl 0
; CHECK: ldc test19a
; CHECK: gcall
; CHECK: rev
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: ldc 5
; CHECK: mul
; CHECK: ajw 4
; CHECK: ldl 0
; CHECK: gcall
}