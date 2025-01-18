; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

@.str = private unnamed_addr constant [4 x i8] c"ABC\00", align 1

define i32 @test8a(i32) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4
  %4 = alloca i8*, align 8
  store i32 %0, i32* %2, align 4
  store i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str, i64 0, i64 0), i8** %4, align 8
  %5 = load i8*, i8** %4, align 8
  %6 = load i32, i32* %2, align 4
  %7 = sext i32 %6 to i64
  %8 = getelementptr inbounds i8, i8* %5, i64 %7
  %9 = load i8, i8* %8, align 1
  %10 = sext i8 %9 to i32
  store i32 %10, i32* %3, align 4
  %11 = load i32, i32* %3, align 4
  ret i32 %11
; CHECK-LABEL: test8a:
; CHECK: stl 0
; CHECK: ajw -6
; CHECK: ldc .L.str
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 3
; CHECK: ldl 5
; CHECK: ldl 1
; CHECK: add
; CHECK: lb
; CHECK: ldc 128
; CHECK: xword
; CHECK: stl 2
; CHECK: ldl 2
; CHECK: stl 4
; CHECK: ldl 2
; CHECK: ajw 6
; CHECK: ldl 0
; CHECK: gcall
}

define i32 @test8b(i32) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4
  store i32 %0, i32* %2, align 4
  %4 = load i32, i32* %2, align 4
  %5 = call i32 @test8a(i32 %4)
  store i32 %5, i32* %3, align 4
  %6 = load i32, i32* %3, align 4
  %7 = mul nsw i32 5, %6
  ret i32 %7
; CHECK-LABEL: test8b:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldl 3
; CHECK: ldlp 4294967295
; CHECK: stnl 0
; CHECK: ldc test8a
; CHECK: gcall
; CHECK: rev
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: ldc 5
; CHECK: mul
; CHECK: ajw 4
; CHECK: ldl 0
; CHECK: gcall
}
