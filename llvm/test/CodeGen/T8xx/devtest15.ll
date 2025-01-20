; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test15(i8* noundef %0) #0 {
  %2 = alloca i8*, align 8
  %3 = alloca [8 x i8], align 1
  store i8* %0, i8** %2, align 8
  %4 = load i8*, i8** %2, align 8
  %5 = load i8, i8* %4, align 1
  %6 = sext i8 %5 to i32
  %7 = icmp sgt i32 %6, 14
  br i1 %7, label %8, label %13

8:                                                ; preds = %1
  %9 = load i8*, i8** %2, align 8
  %10 = getelementptr inbounds i8, i8* %9, i64 3
  %11 = load i8, i8* %10, align 1
  %12 = getelementptr inbounds [8 x i8], [8 x i8]* %3, i64 0, i64 4
  store i8 %11, i8* %12, align 1
  br label %13

13:                                               ; preds = %8, %1
  %14 = getelementptr inbounds [8 x i8], [8 x i8]* %3, i64 0, i64 4
  %15 = load i8, i8* %14, align 1
  %16 = sext i8 %15 to i32
  ret i32 %16
; CHECK-LABEL: test15:
; CHECK: stl 0
; CHECK: ajw -9
; CHECK: ldl 8
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 6
; CHECK: ldc 128
; CHECK: stl 2
; CHECK: ldlp 4
; CHECK: stl 3
; CHECK: ldl 1
; CHECK: lb
; CHECK: ldl 2
; CHECK: xword
; CHECK: ldc 14
; CHECK: gt
; CHECK: cj .LBB0_2
; CHECK: ldl 6
; CHECK: adc 3
; CHECK: lb
; CHECK: ldl 3
; CHECK: adc 4
; CHECK: sb
; CHECK-LABEL: .LBB0_2:
; CHECK: ldl 3
; CHECK: adc 4
; CHECK: lb
; CHECK: ldl 2
; CHECK: xword
; CHECK: ajw 9
; CHECK: ldl 0
; CHECK: gcall
}
