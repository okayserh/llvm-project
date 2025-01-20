; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test17(i16 noundef signext %0) #0 {
  %2 = alloca i16, align 2
  %3 = alloca [8 x i16], align 16
  %4 = alloca i32, align 4
  store i16 %0, i16* %2, align 2
  store i32 0, i32* %4, align 4
  br label %5

5:                                                ; preds = %12, %1
  %6 = load i32, i32* %4, align 4
  %7 = icmp slt i32 %6, 8
  br i1 %7, label %8, label %15

8:                                                ; preds = %5
  %9 = load i32, i32* %4, align 4
  %10 = sext i32 %9 to i64
  %11 = getelementptr inbounds [8 x i16], [8 x i16]* %3, i64 0, i64 %10
  store i16 44, i16* %11, align 2
  br label %12

12:                                               ; preds = %8
  %13 = load i32, i32* %4, align 4
  %14 = add nsw i32 %13, 1
  store i32 %14, i32* %4, align 4
  br label %5

15:                                               ; preds = %5
  %16 = load i16, i16* %2, align 2
  %17 = sext i16 %16 to i32
  ret i32 %17

; CHECK-LABEL: test17:
; CHECK: stl 0
; CHECK: ajw -22
; CHECK: ldl 21
; CHECK: stl 2
; CHECK: ldlp 20
; CHECK: stl 3
; CHECK: ldl 2
; CHECK: ldl 3
; CHECK: sb
; CHECK: ldc 8
; CHECK: stl 4
; CHECK: ldc 1
; CHECK: stl 5
; CHECK: ldl 3
; CHECK: ldl 5
; CHECK: or
; CHECK: stl 6
; CHECK: ldl 2
; CHECK: ldl 4
; CHECK: shr
; CHECK: ldl 6
; CHECK: sb
; CHECK: ldc 0
; CHECK: stl 7
; CHECK: ldl 7
; CHECK-LABEL: .LBB0_1:
; CHECK: stl 8
; CHECK: ldl 4
; CHECK: ldl 8
; CHECK: gt
; CHECK: cj .LBB0_3
; CHECK: ldl 8
; CHECK: ldc 2
; CHECK: shl
; CHECK: ldlp 9
; CHECK: add
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: ldc 44
; CHECK: sb
; CHECK: ldl 1
; CHECK: ldl 5
; CHECK: or
; CHECK: ldl 7
; CHECK: sb
; CHECK: ldl 8
; CHECK: adc 1
; CHECK: j .LBB0_1
; CHECK-LABEL: .LBB0_3:
; CHECK: ldl 6
; CHECK: lb
; CHECK: ldc 128
; CHECK: xword
; CHECK: ldl 4
; CHECK: shl
; CHECK: ldl 3
; CHECK: lb
; CHECK: or
; CHECK: ajw 22
; CHECK: ldl 0
; CHECK: gcall
}
