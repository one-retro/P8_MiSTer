// DIV / DIVU / REM / REMU on pseudo-random operands of every size: for
// each pair, q * b + r == a with q * b in 32 bits, |r| < |b|, and r has
// the sign of a (0 or a's); plus the RISC-V special cases. On the first
// failure the program exits with the failing check number (gp; 1000 + the
// loop count for the random pairs); on success it prints "div ok\n" and
// exits with 0.
#include "macros.h"

.macro CHK reg, exp
  addi gp, gp, 1
  li a3, \exp
  bne \reg, a3, fail
.endm

.section .text.start
.globl _start
_start:
  li gp, 0

  // ---- special cases
  li t0, 7
  div t1, t0, x0
  CHK t1, -1
  rem t1, t0, x0
  CHK t1, 7
  divu t1, t0, x0
  CHK t1, -1
  remu t1, t0, x0
  CHK t1, 7
  li t0, 0x80000000
  li t2, -1
  div t1, t0, t2
  CHK t1, 0x80000000
  rem t1, t0, t2
  CHK t1, 0
  li t0, -7
  li t2, 2
  div t1, t0, t2
  CHK t1, -3
  rem t1, t0, t2
  CHK t1, -1
  li t0, 0xffffffff
  li t2, 0xfffffffe
  divu t1, t0, t2
  CHK t1, 1
  remu t1, t0, t2
  CHK t1, 1
  li t2, 3
  divu t1, t0, t2
  CHK t1, 0x55555555

  // ---- random pairs: x = xorshift32; a = x >> (x & 31), b likewise
  li s0, 0x2545f491         // state
  li s1, 4000               // pairs
  li s2, 0
1:
  // a
  slli t0, s0, 13
  xor s0, s0, t0
  srli t0, s0, 17
  xor s0, s0, t0
  slli t0, s0, 5
  xor s0, s0, t0
  andi t1, s0, 31
  srl a0, s0, t1            // a (unsigned size)
  andi t1, s0, 0x100
  beqz t1, 2f
  neg a0, a0                // negative half the time
2:
  // b
  slli t0, s0, 13
  xor s0, s0, t0
  srli t0, s0, 17
  xor s0, s0, t0
  slli t0, s0, 5
  xor s0, s0, t0
  andi t1, s0, 31
  srl a1, s0, t1
  andi t1, s0, 0x100
  beqz t1, 3f
  neg a1, a1
3:
  beqz a1, 9f               // (division by 0: above)

  // unsigned: q * b + r == a (q * b fits in 32 bits), r < b
  divu t2, a0, a1
  remu t3, a0, a1
  mulhu t4, t2, a1
  bnez t4, 8f
  mul t4, t2, a1
  add t4, t4, t3
  bne t4, a0, 8f
  bgeu t3, a1, 8f

  // signed (not the overflow case)
  li t5, 0x80000000
  bne a0, t5, 4f
  li t5, -1
  beq a1, t5, 9f
4:
  div t2, a0, a1
  rem t3, a0, a1
  mul t4, t2, a1
  mulh t5, t2, a1           // q * b fits in 32 bits (signed)
  srai t6, t4, 31
  bne t5, t6, 8f
  add t4, t4, t3
  bne t4, a0, 8f
  // |r| < |b|
  mv t4, t3
  bgez t4, 5f
  neg t4, t4
5:
  mv t5, a1
  bgez t5, 6f
  neg t5, t5
6:
  bgeu t4, t5, 8f
  // r == 0 or sign(r) == sign(a)
  beqz t3, 9f
  xor t4, t3, a0
  bltz t4, 8f
9:
  addi s2, s2, 1
  blt s2, s1, 1b
  j 10f
8:
  addi gp, s2, 1000
  j fail
10:

  // ---- console
  la s0, msg
  li t6, MMIO
11:lbu a0, 0(s0)
  beqz a0, 12f
  sw a0, R_CONSOLE(t6)
  addi s0, s0, 1
  j 11b
12:
  EXIT_IMM 0

fail:
  MMIO_W R_EXIT, gp
  ebreak

.section .rodata
msg: .asciz "div ok\n"
