; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test4(i32) #0 {
  %2 = alloca i32, align 4
  %3 = alloca [8 x i8], align 1
  store i32 %0, i32* %2, align 4
  %4 = load i32, i32* %2, align 4
  %5 = add nsw i32 %4, 1
  store i32 %5, i32* %2, align 4
  %6 = getelementptr inbounds [8 x i8], [8 x i8]* %3, i64 0, i64 4
  store i8 4, i8* %6, align 1
  %7 = getelementptr inbounds [8 x i8], [8 x i8]* %3, i32 0, i32 0
  %8 = bitcast i8* %7 to i32*
  %9 = load i32, i32* %8, align 1
  %10 = load i32, i32* %2, align 4
  %11 = add nsw i32 %9, %10
  ret i32 %11
; CHECK-LABEL: test4:
; CHECK: stl 0
; CHECK: ajw -8
; CHECK: ldlp 1
; CHECK: stl 5
; CHECK: ldc 4
; CHECK: ldl 5
; CHECK: adc 4
; CHECK: sb
; CHECK: ldl 7
; CHECK: adc 1
; CHECK: stl 4
; CHECK: ldl 4
; CHECK: stl 7
; CHECK: ldl 5
; CHECK: ldlp 3
; CHECK: ldc 2
; CHECK: move
; CHECK: ldl 3
; CHECK: stl 6
; CHECK: ldl 5
; CHECK: adc 2
; CHECK: ldlp 3
; CHECK: ldc 2
; CHECK: move
; CHECK: ldl 3
; CHECK: ldc 16
; CHECK: shl
; CHECK: ldl 6
; CHECK: or
; CHECK: ldl 4
; CHECK: add
; CHECK: ajw 8
; CHECK: ldl 0
; CHECK: gcall
}
