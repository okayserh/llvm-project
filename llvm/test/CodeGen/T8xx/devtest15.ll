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
; CHECK: ajw -1
; CHECK: ldlp 0
; CHECK: adc -40
; CHECK: ldc 7
; CHECK: not
; CHECK: and
; CHECK: gajw
; CHECK: stl 8
; CHECK: ldl 8
; CHECK: ldnl 0
; CHECK: stl 7
; CHECK: ldl 7
; CHECK: stl 2
; CHECK: ldc 128
; CHECK: stl 6
; CHECK: ldlp 3
; CHECK: stl 5
; CHECK: ldl 7
; CHECK: lb
; CHECK: ldl 6
; CHECK: xword
; CHECK: ldc 14
; CHECK: gt
; CHECK: cj .LBB0_2
; CHECK: ldl 2
; CHECK: adc 3
; CHECK: lb
; CHECK: ldl 5
; CHECK: adc 4
; CHECK: sb
; CHECK-LABEL: .LBB0_2:
; CHECK: ldl 5
; CHECK: adc 4
; CHECK: lb
; CHECK: ldl 6
; CHECK: xword
; CHECK: ldl 8
; CHECK: gajw
; CHECK: rev
; CHECK: ajw 1
; CHECK: ldl 0
; CHECK: gcall
}
