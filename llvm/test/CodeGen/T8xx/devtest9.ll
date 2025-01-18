; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test9(i8 noundef signext %0) #0 {
  %2 = alloca i8, align 1
  store i8 %0, i8* %2, align 1
  %3 = load i8, i8* %2, align 1
  %4 = sext i8 %3 to i32
  ret i32 %4
; CHECK-LABEL: test9:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldl 3
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: ldlp 2
; CHECK: adc 3
; CHECK: sb
; CHECK: ldl 1
; CHECK: ajw 4
; CHECK: ldl 0
; CHECK: gcall
}
