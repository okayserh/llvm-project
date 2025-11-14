; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test1(i32 noundef %a, i32 noundef %b) {
entry:
  %a.addr = alloca i32, align 4
  %b.addr = alloca i32, align 4
  %res = alloca i32, align 4
  store i32 %a, ptr %a.addr, align 4
  store i32 %b, ptr %b.addr, align 4
  %0 = load i32, ptr %a.addr, align 4
  %1 = load i32, ptr %b.addr, align 4
  %cmp = icmp ule i32 %0, %1
  br i1 %cmp, label %lor.rhs, label %lor.rhs
lor.rhs:
  %conv = zext i1 %cmp to i32
  store i32 %conv, ptr %res, align 4
  %2 = load i32, ptr %res, align 4
  ret i32 %2

%c = add i32 %a, %b
	ret i32 %c
; CHECK-LABEL: test1:
; CHECK: stl 0
; CHECK: ajw -5
; CHECK: ldc 1
; CHECK: ldl 3
; CHECK: ldl 4
; CHECK: ldiff
; CHECK: rev
; CHECK: ldc 1
; CHECK: and
; CHECK: stl 2
; CHECK: ldl 2
; CHECK: stl 1
; CHECK: ldl 2
; CHECK: ajw 5
; CHECK: ldl 0
; CHECK: gcall
}
