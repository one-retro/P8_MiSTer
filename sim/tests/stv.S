// p8.stv (custom-1, S-type, funct3 = the tag; p8feat bit 16): the 8-byte
// value {tag, rs2} stored at an 8-byte aligned address in one D$ write.
// Checks p8feat, every tag with the words around untouched, loads right
// behind it (the D$ store bypass, both words, bytes of the tag word), its
// operands forwarded, a miss and a dirty victim, then 4000 pseudo-random
// stores over 64 KiB (twice the D$) against a copy written with sw. On
// the first failure the program exits with the failing check number (gp);
// on success it prints "stv ok\n" and exits with 0.
// (A copy of mister_p8 crates/rv32sim/tests/asm/stv.S.)
#include "macros.h"

// p8.stv rs2, off(rs1) with tag t
.macro STV t, v, off, base
  .insn s 0x2b, \t, \v, \off(\base)
.endm
.macro CHKW reg, off, exp
  addi gp, gp, 1
  lw t1, \off(\reg)
  li t2, \exp
  bne t1, t2, fail
.endm

#define BUF 0x200000
#define REF 0x300000
#define N 4000

.section .text.start
.globl _start
_start:
  li gp, 0
  li sp, 0x100000

  // ---- p8feat
  addi gp, gp, 1
  csrr a0, 0x7c1
  li a1, 16
  and a0, a0, a1
  beqz a0, fail

  // ---- every tag, the words around untouched
  li s0, BUF
  li a0, 0x5a5a5a5a
  sw a0, 0(s0)
  sw a0, 4(s0)
  sw a0, 8(s0)
  sw a0, 12(s0)
  sw a0, 16(s0)
  sw a0, 20(s0)
  li a1, 0x12345678
  STV 0, a1, 8, s0
  CHKW s0, 8, 0
  CHKW s0, 12, 0x12345678
  CHKW s0, 4, 0x5a5a5a5a
  CHKW s0, 16, 0x5a5a5a5a
  STV 3, a1, 8, s0
  CHKW s0, 8, 3
  STV 5, zero, 8, s0
  CHKW s0, 8, 5
  CHKW s0, 12, 0
  STV 7, a1, 8, s0
  CHKW s0, 8, 7
  STV 1, a1, 8, s0
  STV 2, a1, 8, s0
  STV 4, a1, 8, s0
  STV 6, a1, 8, s0
  CHKW s0, 8, 6
  CHKW s0, 12, 0x12345678
  CHKW s0, 0, 0x5a5a5a5a
  CHKW s0, 20, 0x5a5a5a5a
  // a negative offset
  addi s1, s0, 16
  li a1, -2
  STV 4, a1, -16, s1
  CHKW s0, 0, 4
  CHKW s0, 4, -2
  CHKW s0, 8, 6

  // ---- loads right behind it (store bypass)
  li a1, 0x0badf00d
  STV 3, a1, 0, s0
  lw t1, 0(s0)              // tag word, at once
  addi gp, gp, 1
  li t2, 3
  bne t1, t2, fail
  STV 5, a1, 0, s0
  lw t1, 4(s0)              // value word, at once
  addi gp, gp, 1
  bne t1, a1, fail
  STV 6, a1, 0, s0
  lbu t1, 1(s0)             // the tag word's upper bytes are 0
  addi gp, gp, 1
  bnez t1, fail
  STV 7, a1, 0, s0
  lbu t1, 0(s0)
  addi gp, gp, 1
  li t2, 7
  bne t1, t2, fail
  STV 2, a1, 0, s0
  lhu t1, 6(s0)
  addi gp, gp, 1
  li t2, 0x0bad
  bne t1, t2, fail
  // then a byte store into each word, loaded at once
  STV 4, a1, 0, s0
  li t3, 0x99
  sb t3, 1(s0)
  lw t1, 0(s0)
  addi gp, gp, 1
  li t2, 0x9904
  bne t1, t2, fail
  STV 4, a1, 0, s0
  sb t3, 7(s0)
  lw t1, 4(s0)
  addi gp, gp, 1
  li t2, 0x99adf00d
  bne t1, t2, fail

  // ---- operands forwarded: value from a load, base from the ALU
  li t3, 0xcafe0000
  sw t3, 32(s0)
  lw a2, 32(s0)
  STV 3, a2, 40, s0         // load-use on the value
  CHKW s0, 40, 3
  CHKW s0, 44, 0xcafe0000
  addi s1, s0, 48
  STV 5, a2, 8, s1          // base from the instruction before
  CHKW s0, 56, 5
  CHKW s0, 60, 0xcafe0000
  li a3, 0x77
  STV 1, a3, 64, s0         // value from the instruction before
  CHKW s0, 68, 0x77

  // ---- a miss, and a dirty victim: the D$ is 32 KiB 2-way (16 KiB a way)
  li s1, BUF + 0x10000
  sw a1, 0(s1)              // dirty, same set as BUF + 0x14000 / 0x18000
  li s2, BUF + 0x14000
  sw a1, 0(s2)
  li s3, BUF + 0x18000
  li a3, 0x31415926
  STV 6, a3, 8, s3          // misses, evicts a dirty line
  CHKW s3, 8, 6
  CHKW s3, 12, 0x31415926
  CHKW s1, 0, 0x0badf00d    // the victim came back from SDRAM
  li s3, BUF + 0x1c000
  STV 2, a3, 16, s3         // and once more, the other way
  CHKW s3, 16, 2
  CHKW s3, 20, 0x31415926
  CHKW s2, 0, 0x0badf00d

  // ---- pseudo-random stores against a copy written with sw
  li s0, BUF
  li s1, REF
  li a0, 0                  // zero both 64 KiB areas
  li a1, 0x10000
1:add t0, s0, a0
  sw zero, 0(t0)
  add t0, s1, a0
  sw zero, 0(t0)
  addi a0, a0, 4
  blt a0, a1, 1b
  li s2, 0x2545f491         // xorshift state
  li s3, 0                  // count
  li s4, N
2:slli t0, s2, 13
  xor s2, s2, t0
  srli t0, s2, 17
  xor s2, s2, t0
  slli t0, s2, 5
  xor s2, s2, t0
  li t0, 0xfff8             // offset in the area, 8-aligned
  and a0, s2, t0
  srli a1, s2, 16           // tag 0..7
  andi a1, a1, 7
  mul a2, s2, s2            // value
  add t0, s1, a0            // the copy
  sw a1, 0(t0)
  sw a2, 4(t0)
  add t0, s0, a0
  andi a3, a1, 7
  // one p8.stv per tag (funct3 is the tag)
  la t1, 3f
  slli a3, a3, 3
  add t1, t1, a3
  jr t1
3:STV 0, a2, 0, t0
  j 4f
  STV 1, a2, 0, t0
  j 4f
  STV 2, a2, 0, t0
  j 4f
  STV 3, a2, 0, t0
  j 4f
  STV 4, a2, 0, t0
  j 4f
  STV 5, a2, 0, t0
  j 4f
  STV 6, a2, 0, t0
  j 4f
  STV 7, a2, 0, t0
  j 4f
4:addi s3, s3, 1
  blt s3, s4, 2b
  // compare
  addi gp, gp, 1
  li a0, 0
  li a1, 0x10000
5:add t0, s0, a0
  lw t2, 0(t0)
  add t0, s1, a0
  lw t3, 0(t0)
  bne t2, t3, fail
  addi a0, a0, 4
  blt a0, a1, 5b

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
msg: .asciz "stv ok\n"
