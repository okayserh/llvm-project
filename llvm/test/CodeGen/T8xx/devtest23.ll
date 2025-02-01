; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test23(i32 noundef %c) #0 {
entry:
  %c.addr = alloca i32, align 4
  %i = alloca i32, align 4
  store i32 %c, ptr %c.addr, align 4
  %0 = load i32, ptr %c.addr, align 4
  %tobool = icmp ne i32 %0, 0
  %1 = zext i1 %tobool to i64
  %cond = select i1 %tobool, i32 10, i32 20
  store i32 %cond, ptr %i, align 4
  %2 = load i32, ptr %i, align 4
  ret i32 %2
; CHECK-LABEL: test23:
}
