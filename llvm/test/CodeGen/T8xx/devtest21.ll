; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test21a(i32 noundef %0, i32 noundef %1, i32 noundef %2, i32 noundef %3) #0 {
  %5 = alloca i32, align 4
  %6 = alloca i32, align 4
  %7 = alloca i32, align 4
  %8 = alloca i32, align 4
  %9 = alloca i32, align 4
  %10 = alloca i32, align 4
  %11 = alloca i32, align 4
  store i32 %0, i32* %5, align 4
  store i32 %1, i32* %6, align 4
  store i32 %2, i32* %7, align 4
  store i32 %3, i32* %8, align 4
  %12 = load i32, i32* %5, align 4
  %13 = load i32, i32* %6, align 4
  %14 = mul nsw i32 %12, %13
  store i32 %14, i32* %9, align 4
  %15 = load i32, i32* %9, align 4
  %16 = load i32, i32* %7, align 4
  %17 = mul nsw i32 %15, %16
  store i32 %17, i32* %10, align 4
  %18 = load i32, i32* %10, align 4
  %19 = load i32, i32* %8, align 4
  %20 = mul nsw i32 %18, %19
  store i32 %20, i32* %11, align 4
  %21 = load i32, i32* %11, align 4
  %22 = load i32, i32* %5, align 4
  %23 = mul nsw i32 %21, %22
  ret i32 %23
; CHECK-LABEL: test21a:
; CHECK: stl 0
; CHECK: ajw -12
; CHECK: ldl 11
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: ldl 10
; CHECK: mul
; CHECK: stl 2
; CHECK: ldl 2
; CHECK: stl 7
; CHECK: ldl 2
; CHECK: ldl 9
; CHECK: mul
; CHECK: stl 3
; CHECK: ldl 3
; CHECK: stl 6
; CHECK: ldl 3
; CHECK: ldl 8
; CHECK: mul
; CHECK: stl 4
; CHECK: ldl 4
; CHECK: stl 5
; CHECK: ldl 4
; CHECK: ldl 1
; CHECK: mul
; CHECK: ajw 12
; CHECK: ldl 0
; CHECK: gcall
}

; Function Attrs: noinline nounwind optnone uwtable
define dso_local i32 @test21b(i32 noundef %0) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4
  store i32 %0, i32* %2, align 4
  %4 = load i32, i32* %2, align 4
  %5 = load i32, i32* %2, align 4
  %6 = load i32, i32* %2, align 4
  %7 = load i32, i32* %2, align 4
  %8 = call i32 @test21a(i32 noundef %4, i32 noundef %5, i32 noundef %6, i32 noundef
 %7)
  store i32 %8, i32* %3, align 4
  %9 = load i32, i32* %3, align 4
  %10 = mul nsw i32 5, %9
  ret i32 %10
; CHECK-LABEL: test21b:
; CHECK: stl 0
; CHECK: ajw -5
; CHECK: ldl 4
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: ldlp 4294967292
; CHECK: stnl 0
; CHECK: ldl 1
; CHECK: ldlp 4294967293
; CHECK: stnl 0
; CHECK: ldl 1
; CHECK: ldlp 4294967294
; CHECK: stnl 0
; CHECK: ldl 1
; CHECK: ldlp 4294967295
; CHECK: stnl 0
; CHECK: ldc test21a
; CHECK: gcall
; CHECK: rev
; CHECK: stl 2
; CHECK: ldl 2
; CHECK: stl 3
; CHECK: ldl 2
; CHECK: ldc 5
; CHECK: mul
; CHECK: ajw 5
; CHECK: ldl 0
; CHECK: gcall
}
