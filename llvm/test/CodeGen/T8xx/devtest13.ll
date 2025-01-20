; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test13(i8 signext) #0 {
  %2 = alloca i8, align 1
  %3 = alloca [8 x i8], align 1
  store i8 %0, i8* %2, align 1
  %4 = load i8, i8* %2, align 1
  %5 = sext i8 %4 to i32
  %6 = icmp sgt i32 %5, 14
  br i1 %6, label %7, label %9

7:
  %8 = getelementptr inbounds [8 x i8], [8 x i8]* %3, i64 0, i64 4
  store i8 0, i8* %8, align 1
  br label %9

9:                                                ; preds = %7, %1
  %10 = load i8, i8* %2, align 1
  %11 = sext i8 %10 to i32
  ret i32 %11
; CHECK-LABEL: test13:
; CHECK: stl 0
; CHECK: ajw -7
; CHECK: ldl 6
; CHECK: stl 1
; CHECK: ldlp 5
; CHECK: adc 3
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: ldl 2
; CHECK: sb
; CHECK: ldl 1
; CHECK: ldc 14
; CHECK: gt
; CHECK: cj .LBB0_2
; CHECK: ldc 0
; CHECK: ldlp 3
; CHECK: adc 3
; CHECK: adc 4
; CHECK: sb
; CHECK-LABEL: .LBB0_2:
; CHECK: ldl 2
; CHECK: lb
; CHECK: ldc 128
; CHECK: xword
; CHECK: ajw 7
; CHECK: ldl 0
; CHECK: gcall
}
