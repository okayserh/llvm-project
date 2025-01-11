; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define i32 @test0(i32 %X) {
	%tmp.1 = add i32 %X, 1
	ret i32 %tmp.1
; CHECK-LABEL: test0:
; CHECK: ldl 1
; CHECK: adc 1
}


;; xnor tests.
define i32 @test1(i32 %X, i32 %Y) {
        %A = xor i32 %X, %Y
        %B = xor i32 %A, -1
        ret i32 %B
; CHECK-LABEL: test1:
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: xor
; CHECK: ldc -1
; CHECK: xor
}

define i32 @test2(i32 %X, i32 %Y) {
        %A = xor i32 %X, -1
        %B = xor i32 %A, %Y
        ret i32 %B
; CHECK-LABEL: test2:
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: xor
; CHECK: ldc -1
; CHECK: xor
}

; CHECK-LABEL: store_zero:
; CHECK: ldc 0
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: ldl 3
; CHECK: stnl 0
; CHECK: ldl 1
; CHECK: ldl 2
; CHECK: adc 4
; CHECK: stnl 0
; CHECK: ldl 1
define i32 @store_zero(i32* %a, i32* %b) {
entry:
  store i32 0, i32* %a, align 4
  %0 = getelementptr inbounds i32, i32* %b, i32 1
  store i32 0, i32* %0, align 4
  ret i32 0
}


; CHECK-LABEL: signed_divide:
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: div
define i32 @signed_divide(i32 %a, i32 %b) {
  %r = sdiv i32 %a, %b
  ret i32 %r
}


; CHECK-LABEL: unsigned_divide:
; CHECK: ldc 0
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: ldiv
define i32 @unsigned_divide(i32 %a, i32 %b) {
  %r = udiv i32 %a, %b
  ret i32 %r
}


; CHECK-LABEL: multiply_32x32:
; CHECK: ldl 2
; CHECK: ldl 1
; CHECK: mul
define i32 @multiply_32x32(i32 %a, i32 %b) {
  %r = mul i32 %a, %b
  ret i32 %r
}

