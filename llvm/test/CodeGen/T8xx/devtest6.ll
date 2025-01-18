; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test6a(i32) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4
  %4 = alloca i32, align 4
  %5 = alloca i32, align 4
  store i32 %0, i32* %2, align 4
  %6 = load i32, i32* %2, align 4
  %7 = add nsw i32 %6, 2
  store i32 %7, i32* %3, align 4
  %8 = load i32, i32* %3, align 4
  store i32 %8, i32* %4, align 4
  %9 = load i32, i32* %4, align 4
  store i32 %9, i32* %5, align 4
  %10 = load i32, i32* %5, align 4
  %11 = load i32, i32* %2, align 4
  %12 = add nsw i32 %10, %11
  ret i32 %12
; CHECK-LABEL: test6a:
; CHECK: stl 0
; CHECK: ajw -7
; CHECK: ldl 6
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: adc 2
; CHECK: stl 2
; CHECK: ldl 2
; CHECK: stl 5
; CHECK: ldl 2
; CHECK: stl 4
; CHECK: ldl 2
; CHECK: stl 3
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: add
; CHECK: ajw 7
; CHECK: ldl 0
; CHECK: gcall
}

define i32 @test6b(i32) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4
  store i32 %0, i32* %2, align 4
  %4 = load i32, i32* %2, align 4
  %5 = call i32 @test6a(i32 %4)
  store i32 %5, i32* %3, align 4
  %6 = load i32, i32* %3, align 4
  %7 = add nsw i32 5, %6
  ret i32 %7
; CHECK-LABEL: test6b:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldl 3
; CHECK: ldlp 4294967295
; CHECK: stnl 0
; CHECK: ldc test6a
; CHECK: gcall
; CHECK: rev
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: adc 5
; CHECK: ajw 4
; CHECK: ldl 0
; CHECK: gcall
}
