// p8.fdiv (custom-0 funct3 2, p8feat bit 8): the PICO-8 division of 16.16
// numbers. Checks p8feat, the special cases (y = 1.0, y = 0, overflow,
// the 2^31 boundary, rounding toward 0), then 4000 pseudo-random pairs of
// every size against a bit-serial reference (48-bit long division here),
// the result used by the next instruction. On the first failure the
// program exits with the failing check number (gp; 1000 + the loop count
// for the random pairs); on success it prints "fdiv ok\n" and exits with 0.
// (A copy of mister_p8 crates/rv32sim/tests/asm/fdiv.S.)
#include "macros.h"

// p8.fdiv rd, rs1, rs2
.macro FDIV rd, a, b
  .insn r 0x0b, 2, 0, \rd, \a, \b
.endm
// x / y == exp
.macro CHK x, y, exp
  addi gp, gp, 1
  li a0, \x
  li a1, \y
  FDIV t1, a0, a1
  li a3, \exp
  bne t1, a3, fail
.endm

.section .text.start
.globl _start
_start:
  li gp, 0
  li sp, 0x100000

  // ---- p8feat
  addi gp, gp, 1
  csrr a0, 0x7c1
  andi a0, a0, 8
  beqz a0, fail

  // ---- special cases
  CHK 0x10000, 0, 0x7fffffff          // 1 / 0
  CHK 0xffff0000, 0, 0x80000001       // -1 / 0
  CHK 0, 0, 0x7fffffff
  CHK 0x80000000, 0x10000, 0x80000000 // x / 1 = x, even -32768
  CHK 0x12345678, 0x10000, 0x12345678
  CHK 0x80000000, 0xffff0000, 0x7fffffff   // -32768 / -1 overflows
  CHK 0x12345678, 0xffff0000, 0xedcba988   // x / -1 = -x
  CHK 0x10000, 0x30000, 0x5555        // 1 / 3, toward 0
  CHK 0xffff0000, 0x30000, 0xffffaaab // -1 / 3
  CHK 0x10000, 0xfffd0000, 0xffffaaab // 1 / -3
  CHK 0xffff0000, 0xfffd0000, 0x5555  // -1 / -3
  CHK 0x7fffffff, 0x8000, 0x7fffffff  // overflow
  CHK 0x7fffffff, 0xffff8000, 0x80000001
  CHK 0x40000000, 0x8000, 0x7fffffff  // |x| = 2^15 |y|: q = 2^31
  CHK 0xc0000000, 0x8000, 0x80000001
  CHK 0x3fffffff, 0x8000, 0x7ffffffe  // just below
  CHK 0xc0000001, 0x8000, 0x80000002
  CHK 1, 0x7fffffff, 0
  CHK 0x7fffffff, 1, 0x7fffffff
  CHK 0x7fff, 1, 0x7fff0000
  CHK 0x8000, 1, 0x7fffffff
  CHK 0x7fffffff, 0x7fffffff, 0x10000
  CHK 0x80000000, 0x80000000, 0x10000
  CHK 0x80000001, 0x7fffffff, 0xffff0000
  CHK 0x00028000, 0x00008000, 0x50000 // 2.5 / 0.5
  // the result of a load, used at once
  addi gp, gp, 1
  la a2, words
  lw a0, 0(a2)
  lw a1, 4(a2)
  FDIV t1, a0, a1
  addi t1, t1, 1
  li a3, 0x20001
  bne t1, a3, fail

  // ---- random pairs: r = xorshift32; x = r >> (r & 31), y likewise,
  // each negated half the time
  li s0, 0x2545f491         // state
  li s1, 4000               // pairs
  li s2, 0
1:
  slli t0, s0, 13
  xor s0, s0, t0
  srli t0, s0, 17
  xor s0, s0, t0
  slli t0, s0, 5
  xor s0, s0, t0
  andi t1, s0, 31
  srl s3, s0, t1
  andi t1, s0, 0x100
  beqz t1, 2f
  neg s3, s3
2:
  slli t0, s0, 13
  xor s0, s0, t0
  srli t0, s0, 17
  xor s0, s0, t0
  slli t0, s0, 5
  xor s0, s0, t0
  andi t1, s0, 31
  srl s4, s0, t1
  andi t1, s0, 0x100
  beqz t1, 3f
  neg s4, s4
3:
  FDIV s5, s3, s4
  xor s5, s5, s0            // (used at once)
  mv a0, s3
  mv a1, s4
  jal ref
  xor a0, a0, s0
  bne a0, s5, 8f
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

// a0 = a0 / a1 the slow way: y = 1.0 gives x; the 48-bit |x| * 2^16 by
// |y| a bit at a time, the quotient in t4:t3; saturated when y = 0 or it
// is 2^31 or more. Clobbers t0-t5, a2, a3.
ref:
  li t0, 0x10000
  beq a1, t0, 9f
  xor t5, a0, a1            // the result's sign (y = 0: x's)
  beqz a1, 8f
  mv t1, a0
  bgez t1, 1f
  neg t1, t1
1:mv t2, a1
  bgez t2, 2f
  neg t2, t2
2:li t0, 0                  // remainder
  li t3, 0
  li t4, 0
  li a2, 48
3:addi a2, a2, -1
  slli t0, t0, 1
  li a3, 16
  blt a2, a3, 4f            // the dividend's low 16 bits are 0
  addi a3, a2, -16
  srl a3, t1, a3
  andi a3, a3, 1
  or t0, t0, a3
4:slli t4, t4, 1
  srli a3, t3, 31
  or t4, t4, a3
  slli t3, t3, 1
  bltu t0, t2, 5f
  sub t0, t0, t2
  ori t3, t3, 1
5:bnez a2, 3b
  bnez t4, 8f
  bltz t3, 8f
  mv a0, t3
  bgez t5, 9f
  neg a0, a0
  ret
8:li a0, 0x7fffffff
  bgez t5, 9f
  li a0, 0x80000001
9:ret

.section .rodata
.balign 4
words: .word 0x40000, 0x20000         // 4 / 2
msg: .asciz "fdiv ok\n"
