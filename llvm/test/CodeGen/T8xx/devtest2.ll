; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test2(i32 %a) {
	%c = add nsw i32 %a, 1
	ret i32 %c
; CHECK-LABEL: test2:
; CHECK: stl 0
; CHECK: ajw -2
; CHECK: ldl 1
; CHECK: adc 1
; CHECK: ajw 2
; CHECK: ldl 0
; CHECK: gcall
}
