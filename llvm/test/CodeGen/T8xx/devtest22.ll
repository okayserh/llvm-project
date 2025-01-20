; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

@test = external dso_local global ptr, align 4

; Function Attrs: noinline nounwind optnone
define dso_local i32 @test22a(i32 noundef %c) #0 {
entry:
  %c.addr = alloca i32, align 4
  %i = alloca i32, align 4
  store i32 %c, ptr %c.addr, align 4
  %0 = load ptr, ptr @test, align 4
  %1 = load i32, ptr %c.addr, align 4
  %arrayidx = getelementptr inbounds i8, ptr %0, i32 %1
  %2 = load i8, ptr %arrayidx, align 1
  %conv = sext i8 %2 to i32
  store i32 %conv, ptr %i, align 4
  %3 = load i32, ptr %i, align 4
  ret i32 %3
; CHECK-LABEL: test22a:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldc test
; CHECK: ldnl 0
; CHECK: ldl 3
; CHECK: add
; CHECK: lb
; CHECK: ldc 128
; CHECK: xword
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: ajw 4
; CHECK: ldl 0
; CHECK: gcall
}

; Function Attrs: noinline nounwind optnone
define dso_local i32 @test22b(i32 noundef %d) #0 {
entry:
  %d.addr = alloca i32, align 4
  %c = alloca i32, align 4
  store i32 %d, ptr %d.addr, align 4
  %0 = load i32, ptr %d.addr, align 4
  %call = call i32 @test22a(i32 noundef %0)
  store i32 %call, ptr %c, align 4
  %1 = load i32, ptr %c, align 4
  %mul = mul nsw i32 5, %1
  ret i32 %mul
; CHECK-LABEL: test22b:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldl 3
; CHECK: ldlp 4294967295
; CHECK: stnl 0
; CHECK: ldc test22a
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

