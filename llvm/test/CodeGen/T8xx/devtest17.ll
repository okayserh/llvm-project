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
; CHECK: ajw -1
; CHECK: ldlp 0
; CHECK: adc -96
; CHECK: ldc 15
; CHECK: not
; CHECK: and
; CHECK: gajw
; CHECK: stl 20
; CHECK: ldl 20
; CHECK: ldnl 0
; CHECK: stl 18
; CHECK: ldlp 1
; CHECK: stl 17
; CHECK: ldl 18
; CHECK: ldl 17
; CHECK: sb
; CHECK: ldc 8
; CHECK: stl 16
; CHECK: ldc 1
; CHECK: stl 15
; CHECK: ldl 17
; CHECK: ldl 15
; CHECK: or
; CHECK: stl 14
; CHECK: ldl 18
; CHECK: ldl 16
; CHECK: shr
; CHECK: ldl 14
; CHECK: sb
; CHECK: ldc 0
; CHECK: stl 13
; CHECK: ldl 13
; CHECK-LABEL: .LBB0_1:
; CHECK: stl 12
; CHECK: ldl 16
; CHECK: ldl 12
; CHECK: gt
; CHECK: cj .LBB0_3
; CHECK: ldl 12
; CHECK: ldc 2
; CHECK: shl
; CHECK: ldlp 4
; CHECK: rev
; CHECK: add
; CHECK: stl 19
; CHECK: ldl 19
; CHECK: ldc 44
; CHECK: rev
; CHECK: sb
; CHECK: ldl 19
; CHECK: ldl 15
; CHECK: or
; CHECK: ldl 13
; CHECK: rev
; CHECK: sb
; CHECK: ldl 12
; CHECK: adc 1
; CHECK: j .LBB0_1
; CHECK-LABEL: .LBB0_3:
; CHECK: ldl 14
; CHECK: lb
; CHECK: ldc 128
; CHECK: xword
; CHECK: ldl 16
; CHECK: shl
; CHECK: ldl 17
; CHECK: lb
; CHECK: or
; CHECK: ldl 20
; CHECK: gajw
; CHECK: rev
; CHECK: ajw 1
; CHECK: ldl 0
; CHECK: gcall
}
