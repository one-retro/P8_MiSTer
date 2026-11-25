// D$ write-back stress: stores and loads over 2x the D$ (dirty victims on
// most misses), a load of a line just evicted while its write-back is
// pending, byte stores, p8.zline over dirty lines, code written by
// stores then run after FENCE.I, and the lines FENCE.I wrote back. Each
// check increments gp; on the first failure the program exits with gp (the
// failing check number); on success it prints "mem ok\n" and exits with 0.
#include "macros.h"

#define REGION 0x00200000
#define NWORDS 16384          // 64 KiB
#define WAYSZ  0x4000         // 16 KiB: one way of the D$ (same set)

.macro CHK reg, exp
  addi gp, gp, 1
  li a3, \exp
  bne \reg, a3, fail
.endm

.macro ZLINE rs1
  .insn r 0x0b, 1, 0, x0, \rs1, x0
.endm

.section .text.start
.globl _start
_start:
  li gp, 0
  li s0, REGION
  li s1, NWORDS
  li s3, 2053               // strides (prime: every word once)
  li s4, 4099

  // ---- pass 1: word i = i * 0x9e3779b1, stored in the order
  // i = (i + 2053) mod N
  li t0, 0                  // i
  li t1, 0                  // count
1:
  slli t2, t0, 2
  add t2, t2, s0
  li t3, 0x9e3779b1
  mul t3, t3, t0
  sw t3, 0(t2)
  lw t4, 0(t2)              // read back at once
  bne t4, t3, fail_rb
  add t0, t0, s3
  blt t0, s1, 2f
  sub t0, t0, s1
2:
  addi t1, t1, 1
  blt t1, s1, 1b
  addi gp, gp, 1

  // ---- check in order, and rewrite as i ^ 0x5a5a5a5a with byte stores
  li t0, 0
3:
  slli t2, t0, 2
  add t2, t2, s0
  lw t4, 0(t2)
  li t3, 0x9e3779b1
  mul t3, t3, t0
  bne t4, t3, fail
  li t3, 0x5a5a5a5a
  xor t3, t3, t0
  sb t3, 0(t2)
  srli t5, t3, 8
  sb t5, 1(t2)
  srli t5, t3, 16
  sh t5, 2(t2)
  addi t0, t0, 1
  blt t0, s1, 3b
  addi gp, gp, 1

  // ---- check in another permuted order
  li t0, 0
  li t1, 0
4:
  slli t2, t0, 2
  add t2, t2, s0
  lw t4, 0(t2)
  li t3, 0x5a5a5a5a
  xor t3, t3, t0
  bne t4, t3, fail
  add t0, t0, s4
  blt t0, s1, 5f
  sub t0, t0, s1
5:
  addi t1, t1, 1
  blt t1, s1, 4b
  addi gp, gp, 1

  // ---- one set: dirty A, B, then C evicts A; A again at once (its
  // write-back may still be going on), then B, C
  li t0, 0x00300000
  li t1, 0x00300000 + WAYSZ
  li t2, 0x00300000 + 2 * WAYSZ
  li s2, 64                 // rounds
6:
  sw s2, 0(t0)
  sw s2, 4(t1)
  sw s2, 8(t2)              // evicts A (dirty)
  lw t3, 0(t0)              // evicts B (dirty)
  bne t3, s2, fail
  lw t3, 4(t1)
  bne t3, s2, fail
  lw t3, 8(t2)
  bne t3, s2, fail
  addi t0, t0, 32           // the next set
  addi t1, t1, 32
  addi t2, t2, 32
  addi s2, s2, -1
  bnez s2, 6b
  addi gp, gp, 1

  // ---- p8.zline: the line of a dirty victim's set; the victim keeps its
  // data, the zeroed line reads 0
  li t0, 0x00400000
  li t1, 0x00400000 + WAYSZ
  li t2, 0x00400000 + 2 * WAYSZ
  li t3, 77
  sw t3, 0(t0)
  sw t3, 28(t1)
  li t3, -1
  sw t3, 4(t2)
  ZLINE t2                  // hit: zeroed
  lw t4, 4(t2)
  CHK t4, 0
  li t5, 0x00400000 + 3 * WAYSZ
  ZLINE t5                  // miss, victim dirty
  lw t4, 0(t5)
  CHK t4, 0
  lw t4, 28(t5)
  CHK t4, 0
  lw t4, 0(t0)
  CHK t4, 77
  lw t4, 28(t1)
  CHK t4, 77

  // ---- code written by stores (with dirty lines around it), FENCE.I,
  // then run: li a0, 123; ret
  la t0, code
  li t1, 0x07b00513         // addi a0, x0, 123
  sw t1, 0(t0)
  li t1, 0x00008067         // jalr x0, 0(ra)
  sw t1, 4(t0)
  li t2, 0x00500000         // dirty lines in other sets
  sw t1, 0(t2)
  sw t1, 64(t2)
  fence.i
  li a0, 0
  jalr ra, 0(t0)
  CHK a0, 123
  // rewritten: li a0, 45
  li t1, 0x02d00513
  sw t1, 0(t0)
  fence.i
  jalr ra, 0(t0)
  CHK a0, 45

  // ---- FENCE.I writes back every dirty line and leaves it clean: fill
  // the D$ with dirty lines, flush, evict them with clean loads (no second
  // write-back), and read them back from DDR
  li t0, 0x00600000
  li t1, 8192               // 32 KiB
  li t2, 0
13:
  slli t3, t2, 2
  add t3, t3, t0
  xori t4, t2, 0x3c5
  sw t4, 0(t3)
  addi t2, t2, 1
  blt t2, t1, 13b
  fence.i
  li t3, 0x00700000         // 64 KiB of loads
  li t4, 0x00710000
14:
  lw t5, 0(t3)
  addi t3, t3, 32
  blt t3, t4, 14b
  li t2, 0
15:
  slli t3, t2, 2
  add t3, t3, t0
  lw t5, 0(t3)
  xori t4, t2, 0x3c5
  bne t5, t4, fail
  addi t2, t2, 1
  blt t2, t1, 15b
  addi gp, gp, 1

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

fail_rb:
  addi gp, gp, 100
fail:
  MMIO_W R_EXIT, gp
  ebreak

.section .rodata
msg: .asciz "mem ok\n"

.data
.p2align 5
code: .space 32
