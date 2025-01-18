; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test3(i32) #0 {
  %2 = alloca i32, align 4
  store i32 %0, i32* %2, align 4
  %3 = load i32, i32* %2, align 4
  %4 = add nsw i32 %3, 1
  store i32 %4, i32* %2, align 4
  %5 = load i32, i32* %2, align 4
  ret i32 %5
; CHECK-LABEL: test3:
; CHECK: stl 0
; CHECK: ajw -3
; CHECK: ldl 2
; CHECK: adc 1
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: ajw 3
; CHECK: ldl 0
; CHECK: gcall
}

