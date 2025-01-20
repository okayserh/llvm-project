; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test14(i8 noundef signext %0) #0 {
  %2 = alloca i8, align 1
  %3 = alloca [8 x i8], align 1
  store i8 %0, i8* %2, align 1
  %4 = load i8, i8* %2, align 1
  %5 = sext i8 %4 to i32
  %6 = getelementptr inbounds [8 x i8], [8 x i8]* %3, i64 0, i64 4
  store i8 0, i8* %6, align 1
  br label %7

7:                                                ; preds = %7, %1
  %8 = load i8, i8* %2, align 1
  %9 = sext i8 %8 to i32
  ret i32 %9
; CHECK-LABEL: test14:
; CHECK: stl 0
; CHECK: ajw -6
; CHECK: ldlp 4
; CHECK: adc 3
; CHECK: stl 1
; CHECK: ldl 5
; CHECK: ldl 1
; CHECK: sb
; CHECK: ldc 0
; CHECK: ldlp 2
; CHECK: adc 3
; CHECK: adc 4
; CHECK: sb
; CHECK: ldl 1
; CHECK: lb
; CHECK: ldc 128
; CHECK: xword
; CHECK: ajw 6
; CHECK: ldl 0
; CHECK: gcall
}
