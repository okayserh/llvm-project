; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test_add(i32 %a, i32 %b) {
	%c = add i32 %a, %b
	ret i32 %c
; CHECK-LABEL: test_add:
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: add
}

define i32 @test_sub(i32 %a, i32 %b) {
	%c = sub i32 %a, %b
	ret i32 %c
; CHECK-LABEL: test_sub:
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: sub
}


