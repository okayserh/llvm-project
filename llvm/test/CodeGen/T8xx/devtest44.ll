; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

; Function Attrs: noinline nounwind optnone
define dso_local float @foo(i32 noundef %c) #0 {
entry:
  %c.addr = alloca i32, align 4
  %i = alloca float, align 4
  store i32 %c, ptr %c.addr, align 4
  %0 = load i32, ptr %c.addr, align 4
  %tobool = icmp ne i32 %0, 0
  %1 = zext i1 %tobool to i64
  %cond = select i1 %tobool, double 1.050000e+01, double 2.060000e+01
  %conv = fptrunc double %cond to float
  store float %conv, ptr %i, align 4
  %2 = load float, ptr %i, align 4
  ret float %2
}

; CHECK-LABEL: foo:
; CHECK: stl 0
; CHECK: ajw -5
; CHECK: ldc 0
; CHECK: stl 3
; CHECK: ldl 3
; CHECK: ldl 4
; CHECK: diff
; CHECK: eqc 0
; CHECK: eqc 0
; CHECK: cj .LBB0_2
; CHECK-LABEL:.LBB0_1:
; CHECK: ldc 8
; CHECK: stl 3
; CHECK-LABEL:.LBB0_2:
; CHECK: ldc %iptr_sym(.LCPI0_0)
; CHECK: ldpi
; CHECK: ldl 3
; CHECK: add
; CHECK: fpldnldb
; CHECK: fpur64tor32
; CHECK: ldlp 2
; CHECK: fpstnlsn
; CHECK: ldlp 2
; CHECK: fpldnlsn
; CHECK: ldlp 1
; CHECK: fpstnlsn
; CHECK: ldlp 2
; CHECK: fpldnlsn
; CHECK: ajw 5
; CHECK: ldl 0
; CHECK: gcall
