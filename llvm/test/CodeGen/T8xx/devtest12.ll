; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test12(i8 noundef signext %0) #0 {
  %2 = alloca i8, align 1
  %3 = alloca [8 x i8], align 1
  %4 = alloca i32, align 4
  store i8 %0, i8* %2, align 1
  store i32 0, i32* %4, align 4
  br label %5

5:                                                ; preds = %12, %1
  %6 = load i32, i32* %4, align 4
  %7 = icmp slt i32 %6, 8
  br i1 %7, label %8, label %15

8:                                                ; preds = %5
  %9 = load i32, i32* %4, align 4
  %10 = sext i32 %9 to i64
  %11 = getelementptr inbounds [8 x i8], [8 x i8]* %3, i64 0, i64 %10
  store i8 4, i8* %11, align 1
  br label %12

12:                                               ; preds = %8
  %13 = load i32, i32* %4, align 4
  %14 = add nsw i32 %13, 1
  store i32 %14, i32* %4, align 4
  br label %5

15:                                               ; preds = %5
  %16 = load i8, i8* %2, align 1
  %17 = sext i8 %16 to i32
  ret i32 %17
; CHECK-LABEL: test12:
; CHECK: stl 0
; CHECK: ajw -7
; CHECK: ldlp 5
; CHECK: adc 3
; CHECK: stl 1
; CHECK: ldl 6
; CHECK: ldl 1
; CHECK: sb
; CHECK: ldc 0
; CHECK-LABEL: .LBB0_1:
; CHECK: stl 2
; CHECK: ldc 8
; CHECK: ldl 2
; CHECK: gt
; CHECK: cj .LBB0_3
; CHECK: ldlp 3
; CHECK: adc 3
; CHECK: ldl 2
; CHECK: add
; CHECK: ldc 4
; CHECK: sb
; CHECK: ldl 2
; CHECK: adc 1
; CHECK: j .LBB0_1
; CHECK-LABEL: .LBB0_3:
; CHECK: ldl 1
; CHECK: lb
; CHECK: ldc 128
; CHECK: xword
; CHECK: ajw 7
; CHECK: ldl 0
; CHECK: gcall
}
