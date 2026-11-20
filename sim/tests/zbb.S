// Zbb self-test (build with -march=rv32im_zbb). Each check increments gp;
// on the first failure the program exits with gp (the failing check
// number); on success it prints "zbb ok\n" and exits with 0.
// The core computes Zbb results in M, like MUL: the hazard checks use a
// result in the next instruction (ALU, branch, store data, load address).
#include "macros.h"

.macro TRR op, a, b, exp
  addi gp, gp, 1
  li a0, \a
  li a1, \b
  \op a2, a0, a1
  li a3, \exp
  bne a2, a3, fail
.endm

.macro TR op, a, exp
  addi gp, gp, 1
  li a0, \a
  \op a2, a0
  li a3, \exp
  bne a2, a3, fail
.endm

.macro TRI op, a, imm, exp
  addi gp, gp, 1
  li a0, \a
  \op a2, a0, \imm
  li a3, \exp
  bne a2, a3, fail
.endm

.section .text.start
.globl _start
_start:
  li gp, 0

  // ---- logic with negate
  TRR andn, 0xff00ff00, 0x0ff00ff0, 0xf000f000
  TRR orn,  0x0000ff00, 0xffff0000, 0x0000ffff
  TRR xnor, 0xf0f0f0f0, 0xff00ff00, 0xf00ff00f

  // ---- counts
  TR clz, 0, 32
  TR clz, 1, 31
  TR clz, 0x80000000, 0
  TR clz, 0x00010000, 15
  TR ctz, 0, 32
  TR ctz, 0x80000000, 31
  TR ctz, 0x00000018, 3
  TR cpop, 0, 0
  TR cpop, 0xffffffff, 32
  TR cpop, 0x10203040, 5

  // ---- sign / zero extension
  TR sext.b, 0x00000080, 0xffffff80
  TR sext.b, 0x1234567f, 0x0000007f
  TR sext.h, 0x00008000, 0xffff8000
  TR sext.h, 0xabcd1234, 0x00001234
  TR zext.h, 0xabcd8765, 0x00008765

  // ---- min / max
  TRR min,  -5, 3, -5
  TRR min,  0x80000000, 0x7fffffff, 0x80000000
  TRR minu, -5, 3, 3
  TRR minu, 7, 7, 7
  TRR max,  -5, 3, 3
  TRR max,  0x80000000, 0x7fffffff, 0x7fffffff
  TRR maxu, -5, 3, -5
  TRR maxu, 0, 0, 0

  // ---- rotations, byte ops
  TRR rol, 0x80000001, 1, 0x00000003
  TRR rol, 0x12345678, 36, 0x23456781
  TRR ror, 0x80000001, 1, 0xc0000000
  TRR ror, 0x12345678, 0, 0x12345678
  TRI rori, 0x12345678, 8, 0x78123456
  TRI rori, 0x00000001, 31, 0x00000002
  TR orc.b, 0x00ff0100, 0x00ffff00
  TR orc.b, 0, 0
  TR rev8, 0x11223344, 0x44332211

  // ---- results used at once
  addi gp, gp, 1
  li a0, 10
  li a1, -3
  max a2, a0, a1
  addi a2, a2, 1            // ALU right after
  li a3, 11
  bne a2, a3, fail
  addi gp, gp, 1
  minu a2, a0, a1
  bne a2, a0, fail          // branch right after
  addi gp, gp, 1
  la s1, buf
  min a2, a0, a1
  sw a2, 0(s1)              // store data right after
  lw a3, 0(s1)
  li a4, -3
  bne a3, a4, fail
  addi gp, gp, 1
  li a5, 4
  clz a2, a5                // 29
  addi a2, a2, -29
  add a2, a2, s1            // load address from it (via one add)
  sw a0, 0(a2)
  li a2, 0
  minu a2, s1, s1           // load base straight from Zbb
  lw a3, 0(a2)
  bne a3, a0, fail
  addi gp, gp, 1
  li a0, 0x0f
  cpop a1, a0
  cpop a2, a1               // chained: 4 -> 1
  li a3, 1
  bne a2, a3, fail
  addi gp, gp, 1
  li a0, 5
  li a1, 9
  max a2, a0, a1
  mul a3, a2, a2            // MUL right after
  li a4, 81
  bne a3, a4, fail

  // ---- the predictor: a loop whose branch flips, nested loops
  addi gp, gp, 1
  li a0, 0
  li a1, 0
  li a2, 100
1:andi a3, a0, 1
  beqz a3, 2f               // taken every other time
  addi a1, a1, 1
2:addi a0, a0, 1
  blt a0, a2, 1b
  li a3, 50
  bne a1, a3, fail
  addi gp, gp, 1
  li a0, 0
  li a4, 0
3:li a1, 0
4:addi a4, a4, 1
  addi a1, a1, 1
  li a3, 7
  blt a1, a3, 4b
  addi a0, a0, 1
  li a3, 13
  blt a0, a3, 3b
  li a3, 91
  bne a4, a3, fail

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
msg: .asciz "zbb ok\n"

.data
.p2align 2
buf: .space 16
