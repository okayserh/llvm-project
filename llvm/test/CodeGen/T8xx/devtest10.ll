; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test10(i8 noundef signext %0) #0 {
  %2 = alloca i8, align 1
  %3 = alloca [8 x i8], align 1
  store i8 %0, i8* %2, align 1
  %4 = getelementptr inbounds [8 x i8], [8 x i8]* %3, i64 0, i64 4
  store i8 6, i8* %4, align 1
  %5 = load i8, i8* %2, align 1
  %6 = sext i8 %5 to i32
  ret i32 %6
; CHECK-LABEL: test10:
; CHECK: stl 0
; CHECK: ajw -6
; CHECK: ldl 5
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: ldlp 4
; CHECK: adc 3
; CHECK: sb
; CHECK: ldc 6
; CHECK: ldlp 2
; CHECK: adc 3
; CHECK: adc 4
; CHECK: sb
; CHECK: ldl 1
; CHECK: ajw 6
; CHECK: ldl 0
; CHECK: gcall
}
