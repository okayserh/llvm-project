; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test1(i32 %a, i32 %b) {
	%c = add i32 %a, %b
	ret i32 %c
; CHECK-LABEL: test1:
; CHECK: stl 0
; CHECK: ajw -3
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: add
; CHECK: ajw 3
; CHECK: ldl 0
; CHECK: gcall
}
