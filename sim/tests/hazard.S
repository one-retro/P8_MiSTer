// Late results used at once (build with -march=rv32im_zbb). Each check
// increments gp; on the first failure the program exits with gp (the
// failing check number); on success it prints "hazard ok\n" and exits with
// 0. The result of a load (D$, D$ miss, PICO-8 RAM, MMIO), MUL, DIV, CSR
// read or Zbb op is read by the next instruction, which waits a cycle in E:
// in each operand, as a branch / JALR / load / store / divide / p8.zline /
// p8.hfind operand, with other operands forwarded from W during the wait,
// on a wrong path, and in chains.
#include "macros.h"

#define P8RAM 0x10000000

.macro CHK reg, exp
  addi gp, gp, 1
  li a3, \exp
  bne \reg, a3, fail
.endm

// p8.zline rs1 / p8.hfind rd, rs1, rs2 (custom-0)
.macro ZLINE rs1
  .insn r 0x0b, 1, 0, x0, \rs1, x0
.endm
.macro HFIND rd, rs1, rs2
  .insn r 0x0b, 0, 0, \rd, \rs1, \rs2
.endm

.section .text.start
.globl _start
_start:
  li gp, 0
  la s0, vals               // 7, 5, &vals[0], &chain, 0x40, -9
  la s1, buf

  // ---- ALU operands
  lw a0, 0(s0)
  add a1, a0, a0            // both
  CHK a1, 14
  li a2, 100
  lw a0, 4(s0)
  sub a1, a2, a0            // rs2
  CHK a1, 95
  lw a0, 0(s0)
  addi a1, a0, -8           // rs1 with an immediate
  CHK a1, -1
  lbu a0, 16(s0)
  slli a1, a0, 2            // shift by an immediate
  CHK a1, 0x100
  lw a0, 4(s0)
  li a2, 1
  sll a1, a2, a0            // shift amount
  CHK a1, 32
  lw a0, 20(s0)
  slt a1, a0, x0            // compare
  CHK a1, 1
  lw a0, 0(s0)
  add a0, a0, a0            // chain through M
  add a0, a0, a0
  CHK a0, 28

  // ---- two loads, the next instruction reads both (the first from W
  // while it waits for the second)
  lw a0, 0(s0)
  lw a1, 4(s0)
  sub a2, a0, a1
  CHK a2, 2
  lw a1, 4(s0)
  lw a0, 0(s0)
  sub a2, a0, a1
  CHK a2, 2

  // ---- branches
  addi gp, gp, 1
  li a1, 7
  lw a0, 0(s0)
  bne a0, a1, fail          // not taken
  addi gp, gp, 1
  lw a0, 4(s0)
  beq a0, a1, fail          // not taken
  addi gp, gp, 1
  lw a0, 20(s0)
  bge a0, x0, fail          // signed
  addi gp, gp, 1
  lw a0, 20(s0)
  bltu a0, a1, fail         // unsigned
  addi gp, gp, 1
  lw a0, 0(s0)
  beq a0, a1, 1f            // taken forward: mispredicted, a wrong-path
  lw a2, 0(s0)              // load and its user are dropped
  add a2, a2, a2
  j fail
1:addi gp, gp, 1
  lw a0, 4(s0)
  .insn b 0x63, 2, a0, x5, 3f   // beqi a0, 5: taken forward
  j fail
3:li a2, 3
  li a4, 0
4:addi a4, a4, 1            // backward loop branch on a load
  addi a2, a2, -1
  sw a2, 0(s1)
  lw a5, 0(s1)
  bne a5, x0, 4b
  CHK a4, 3

  // ---- loads and stores
  lw a0, 8(s0)
  lw a1, 0(a0)              // pointer chase
  CHK a1, 7
  lw a0, 12(s0)
  lw a0, 0(a0)
  lw a0, 0(a0)
  lw a0, 0(a0)              // chain of three
  CHK a0, 5
  lw a0, 0(s0)
  sw a0, 0(s1)              // store data
  lw a1, 0(s1)
  CHK a1, 7
  lw a0, 8(s0)
  sw s1, 4(a0)              // store address: vals[1] = buf
  lw a1, 4(s0)
  addi gp, gp, 1
  bne a1, s1, fail
  li a2, 5
  sw a2, 4(s0)              // (5 again)
  lw a0, 0(s0)
  sb a0, 3(s1)              // byte store data
  lw a1, 0(s1)
  CHK a1, 0x07000007

  // ---- a load right after a store to the same 64-bit row: only the
  // bytes the store wrote come from it
  la s4, sbuf
  li a0, 0x11223344
  li a1, 0x55667788
  sw a0, 0(s4)
  sw a1, 4(s4)
  sw a1, 0(s4)
  lw a2, 4(s4)              // the other word
  CHK a2, 0x55667788
  sb a0, 5(s4)
  lw a2, 4(s4)              // one byte of it
  CHK a2, 0x55664488
  sh a0, 0(s4)
  lhu a2, 2(s4)             // the other half
  CHK a2, 0x5566
  lw a2, 0(s4)
  CHK a2, 0x55663344
  sw a0, 4(s4)
  lb a2, 7(s4)
  CHK a2, 0x11
  sb a1, 2(s4)
  lw a2, 0(s4)
  CHK a2, 0x55883344

  // ---- D$ miss: the load is replayed, its user waits for the replay
  la s2, far
  li a0, 0x1234
  sw a0, 0(s2)
  li t0, 16384
  add t1, s2, t0            // same set, both other ways: s2's line is evicted
  lw x0, 0(t1)
  add t1, t1, t0
  lw x0, 0(t1)
  lw a0, 0(s2)              // miss (written back before)
  addi a1, a0, 1
  CHK a1, 0x1235
  add t1, s2, t0
  lw x0, 0(t1)
  add t1, t1, t0
  lw x0, 0(t1)
  lw a0, 0(s2)              // miss, its user a load
  lw a1, 0(s1)
  add a2, a0, a1
  CHK a2, 0x0700123b

  // ---- PICO-8 RAM and MMIO loads
  li s3, P8RAM
  li a0, 0x8badf00d
  sw a0, 0x200(s3)
  lw a1, 0x200(s3)
  addi a2, a1, 1
  CHK a2, 0x8badf00e
  lb a1, 0x203(s3)
  add a2, a1, a1
  CHK a2, -0xea
  li a0, 0x200
  add a0, a0, s3
  sw x0, 0x204(s3)
  lhu a1, 0(a0)
  sh a1, 0x204(s3)
  lw a2, 0x204(s3)
  CHK a2, 0xf00d
  li t6, MMIO
  lw a0, R_CYCLE_LO(t6)
  lw a1, R_CYCLE_LO(t6)
  sub a2, a1, a0            // MMIO loads (replayed once each)
  addi gp, gp, 1
  blez a2, fail

  // ---- JALR to a loaded address (predicted: offset 0; and with one)
  addi gp, gp, 1
  la a0, target
  sw a0, 0(s1)
  lw a1, 0(s1)
  jalr ra, 0(a1)
  bne a0, x0, fail          // target clears a0
  addi gp, gp, 1
  la a0, target
  addi a0, a0, -4
  sw a0, 0(s1)
  li a0, 1
  lw a1, 0(s1)
  jalr ra, 4(a1)
  bne a0, x0, fail

  // ---- MUL, DIV, CSR, Zbb results, and loads into them
  li a0, 6
  li a1, 7
  mul a2, a0, a1
  addi a2, a2, 1
  CHK a2, 43
  lw a0, 0(s0)
  mul a2, a0, a0
  CHK a2, 49
  lw a0, 0(s0)
  mulh a2, a0, a0
  CHK a2, 0
  // (the registers hold other values before: a divide that started
  // without waiting would use them)
  li a0, 100
  li a1, 3
  lw a1, 0(s0)
  div a2, a0, a1            // divisor from a load: 100 / 7
  CHK a2, 14
  li a1, 2
  li a0, 50
  lw a0, 0(s0)
  rem a2, a0, a1            // dividend from a load: 7 % 2
  CHK a2, 1
  li a0, 100
  li a1, 7
  li a2, 70
  divu a2, a0, a1
  add a2, a2, a2            // quotient used at once
  CHK a2, 28
  li a0, 63
  li a2, 70
  div a2, a0, a1
  div a2, a2, a1            // dividend from the divider: 63 / 7 / 7
  CHK a2, 1
  li a0, 9
  lw a0, 0(s0)
  divu a2, a0, a0           // both from a load
  CHK a2, 1
  lw a0, 0(s0)
  csrw mscratch, a0         // CSR write data from a load
  csrr a1, mscratch
  addi a1, a1, 1            // CSR read used at once
  CHK a1, 8
  lw a0, 20(s0)
  lw a1, 0(s0)
  min a2, a0, a1            // Zbb compare from loads
  CHK a2, -9
  lw a0, 20(s0)
  lw a1, 0(s0)
  maxu a2, a1, a0
  CHK a2, -9
  lw a0, 4(s0)
  max a2, a0, a0
  minu a2, a2, a0
  CHK a2, 5
  lw a0, 16(s0)
  cpop a1, a0
  CHK a1, 1

  // ---- p8.zline and p8.hfind (wait in M) on a loaded operand
  la a0, zbuf
  li a1, -1
  sw a1, 0(a0)
  sw a1, 28(a0)
  sw a0, 0(s1)
  lw a2, 0(s1)
  ZLINE a2
  lw a1, 0(a0)
  CHK a1, 0
  lw a1, 28(a0)
  CHK a1, 0
  // hfind of a table whose node count is 0 answers 0: heap base zbuf
  // (zeroed above), table 0, so the header at zbuf + 16 is read
  csrw 0x7c0, a0
  sw x0, 0(s1)
  li a1, 99
  lw a2, 0(s1)
  HFIND a1, a2, a2
  CHK a1, 0
  lw a2, 0(s1)
  HFIND a1, a2, a0
  addi a1, a1, 5            // hfind result used at once
  CHK a1, 5

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

.p2align 2
target:
  li a0, 0
  ret

.section .rodata
msg: .asciz "hazard ok\n"

.data
.p2align 5
vals:
  .word 7, 5, vals, chain, 0x40, -9
chain:
  .word chain + 4, chain + 8, 5
.p2align 5
zbuf: .space 32
buf: .space 16
.p2align 3
sbuf: .space 8

.bss
.p2align 5
far: .space 49152
