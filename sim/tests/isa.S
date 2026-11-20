// RV32IM semantics self-test. Each check increments gp; on the first
// failure the program exits with gp (the failing check number); on success
// it prints "isa ok\n" and exits with 0.
#include "macros.h"

.macro TRR op, a, b, exp
  addi gp, gp, 1
  li a0, \a
  li a1, \b
  \op a2, a0, a1
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

.macro CHECK reg, exp
  addi gp, gp, 1
  li a3, \exp
  bne \reg, a3, fail
.endm

// Branch must be taken (t) or not (n).
.macro TBR op, a, b, taken
  addi gp, gp, 1
  li a0, \a
  li a1, \b
  li a2, 0
  \op a0, a1, 1f
  li a2, 1
1:
  li a3, 1 - \taken
  bne a2, a3, fail
.endm

.section .text.start
.globl _start
_start:
  li gp, 0

  // ---- R-type ALU
  TRR add, 1, 2, 3
  TRR add, 0x7fffffff, 1, 0x80000000
  TRR add, 0xffffffff, 1, 0
  TRR sub, 1, 2, 0xffffffff
  TRR sub, 0x80000000, 1, 0x7fffffff
  TRR sll, 1, 31, 0x80000000
  TRR sll, 1, 33, 2
  TRR sll, 0x12345678, 4, 0x23456780
  TRR slt, -1, 1, 1
  TRR slt, 1, -1, 0
  TRR slt, 5, 5, 0
  TRR sltu, -1, 1, 0
  TRR sltu, 1, -1, 1
  TRR xor, 0xff00ff00, 0x0ff00ff0, 0xf0f0f0f0
  TRR srl, 0x80000000, 4, 0x08000000
  TRR srl, 0x80000000, 36, 0x08000000
  TRR sra, 0x80000000, 4, 0xf8000000
  TRR sra, 0x40000000, 4, 0x04000000
  TRR sra, 0x80000000, 31, 0xffffffff
  TRR or, 0xff00ff00, 0x0ff00ff0, 0xfff0fff0
  TRR and, 0xff00ff00, 0x0ff00ff0, 0x0f000f00

  // ---- I-type ALU
  TRI addi, 1, -2, 0xffffffff
  TRI addi, 0x7fffffff, 1, 0x80000000
  TRI addi, 0, 2047, 2047
  TRI addi, 0, -2048, 0xfffff800
  TRI slti, -5, -4, 1
  TRI slti, -4, -5, 0
  TRI sltiu, 0, -1, 1
  TRI sltiu, -1, -1, 0
  TRI sltiu, 5, 6, 1
  TRI xori, 0x00ff00ff, -1, 0xff00ff00
  TRI xori, 0x00ff00ff, 0x0f0, 0x00ff000f
  TRI ori, 0x00ff0000, -16, 0xfffffff0
  TRI andi, 0x12345678, -16, 0x12345670
  TRI andi, 0x12345678, 0x7ff, 0x678
  TRI slli, 1, 31, 0x80000000
  TRI srli, 0x80000000, 31, 1
  TRI srai, 0x80000000, 31, 0xffffffff
  TRI srai, 0x7fffffff, 30, 1

  // ---- LUI / AUIPC
  addi gp, gp, 1
  lui a0, 0xfffff
  li a3, 0xfffff000
  bne a0, a3, fail
  addi gp, gp, 1
1:auipc a0, 0
  la a1, 1b
  bne a0, a1, fail
  addi gp, gp, 1
2:auipc a0, 1
  la a1, 2b
  sub a0, a0, a1
  li a3, 0x1000
  bne a0, a3, fail

  // ---- M extension
  TRR mul, 6, 7, 42
  TRR mul, -6, 7, -42
  TRR mul, 0x12345678, 0x9abcdef0, 0x242d2080
  TRR mul, 0x80000000, -1, 0x80000000
  TRR mulh, 0x80000000, 0x80000000, 0x40000000
  TRR mulh, -1, -1, 0
  TRR mulh, -1, 1, 0xffffffff
  TRR mulh, 0x12345678, 0x9abcdef0, 0xf8cc93d6
  TRR mulhu, 0xffffffff, 0xffffffff, 0xfffffffe
  TRR mulhu, 0x12345678, 0x9abcdef0, 0x0b00ea4e
  TRR mulhsu, -1, 0xffffffff, 0xffffffff
  TRR mulhsu, 0x80000000, 0xffffffff, 0x80000000
  TRR mulhsu, 0x7fffffff, 0xffffffff, 0x7ffffffe
  TRR mulhsu, 0x12345678, 0x9abcdef0, 0x0b00ea4e
  TRR div, 20, 6, 3
  TRR div, -20, 6, -3
  TRR div, 20, -6, -3
  TRR div, -20, -6, 3
  TRR div, 20, 0, -1
  TRR div, 0x80000000, -1, 0x80000000
  TRR divu, 20, 6, 3
  TRR divu, -20, 6, 0x2aaaaaa7
  TRR divu, 20, 0, 0xffffffff
  TRR divu, 0x80000000, -1, 0
  TRR rem, 20, 6, 2
  TRR rem, -20, 6, -2
  TRR rem, 20, -6, 2
  TRR rem, -20, -6, -2
  TRR rem, 20, 0, 20
  TRR rem, -20, 0, -20
  TRR rem, 0x80000000, -1, 0
  TRR remu, 20, 6, 2
  TRR remu, -20, 6, 2
  TRR remu, 20, 0, 20
  TRR remu, -20, 0, -20

  // ---- branches
  TBR beq, 1, 1, 1
  TBR beq, 1, 2, 0
  TBR bne, 1, 2, 1
  TBR bne, 1, 1, 0
  TBR blt, -1, 1, 1
  TBR blt, 1, -1, 0
  TBR blt, 1, 1, 0
  TBR bge, 1, -1, 1
  TBR bge, 1, 1, 1
  TBR bge, -1, 1, 0
  TBR bltu, 1, -1, 1
  TBR bltu, -1, 1, 0
  TBR bltu, 1, 1, 0
  TBR bgeu, -1, 1, 1
  TBR bgeu, 1, 1, 1
  TBR bgeu, 1, -1, 0
  // backward branch
  addi gp, gp, 1
  li a0, 3
  li a1, 0
3:addi a1, a1, 1
  addi a0, a0, -1
  bnez a0, 3b
  li a3, 3
  bne a1, a3, fail

  // ---- JAL / JALR
  addi gp, gp, 1
  jal ra, 4f
5:j fail
4:la a3, 5b
  bne ra, a3, fail
  addi gp, gp, 1
  la a0, 6f
  addi a0, a0, 1       // bit 0 must be cleared by jalr
  jalr t0, 0(a0)
7:j fail
6:la a3, 7b
  bne t0, a3, fail
  addi gp, gp, 1
  la a0, 8f - 4
  jalr x0, 4(a0)       // positive offset, no link
  j fail
8:
  addi gp, gp, 1       // jalr with rd == rs1
  la a0, 9f
  jalr a0, 0(a0)
10:j fail
9:la a3, 10b
  bne a0, a3, fail

  // ---- x0 is hard-wired
  addi gp, gp, 1
  li x0, 123
  addi a0, x0, 0
  bnez a0, fail
  addi gp, gp, 1
  la a1, word
  lw x0, 0(a1)
  bnez x0, fail

  // ---- loads
  la s0, data
  lb a0, 0(s0)
  CHECK a0, 0xffffff80
  lbu a0, 0(s0)
  CHECK a0, 0x80
  lb a0, 1(s0)
  CHECK a0, 0x7f
  lh a0, 2(s0)
  CHECK a0, 0xffff8001
  lhu a0, 2(s0)
  CHECK a0, 0x8001
  lw a0, 4(s0)
  CHECK a0, 0xdeadbeef
  lh a0, 4(s0)
  CHECK a0, 0xffffbeef
  lhu a0, 6(s0)
  CHECK a0, 0xdead
  lb a0, 7(s0)
  CHECK a0, 0xffffffde
  addi s1, s0, 8
  lw a0, -4(s1)
  CHECK a0, 0xdeadbeef

  // ---- stores
  la s0, buf
  li a0, 0x11223344
  sw a0, 0(s0)
  li a0, 0xaabbccdd
  sb a0, 1(s0)
  lw a1, 0(s0)
  CHECK a1, 0x1122dd44
  sh a0, 2(s0)
  lw a1, 0(s0)
  CHECK a1, 0xccdddd44
  sw a0, 4(s0)
  lw a1, 4(s0)
  CHECK a1, 0xaabbccdd
  addi s1, s0, 16
  sw a0, -8(s1)
  lw a1, 8(s0)
  CHECK a1, 0xaabbccdd
  // bss is zero
  la s2, bss_word
  lw a1, 0(s2)
  CHECK a1, 0

  // ---- PICO-8 RAM (all widths)
  li s0, 0x10000000
  li a0, 0x8badf00d
  sw a0, 0x100(s0)
  lw a1, 0x100(s0)
  CHECK a1, 0x8badf00d
  lb a1, 0x103(s0)
  CHECK a1, 0xffffff8b
  lhu a1, 0x100(s0)
  CHECK a1, 0xf00d
  li s1, 0x1000ffff
  sb a0, 0(s1)
  lbu a1, 0(s1)
  CHECK a1, 0x0d
  // ---- self-modifying code: overwrite an instruction, fence.i, run it
  addi gp, gp, 1
  la a0, smc
  li a1, 0x02a00513     // li a0, 42
  jal ra, smc           // first run: original (li a0, 7)
  li a3, 7
  bne a0, a3, fail
  la a0, smc
  li a1, 0x02a00513
  sw a1, 0(a0)
  fence.i
  jal ra, smc
  li a3, 42
  bne a0, a3, fail

  // ---- CSRs
  addi gp, gp, 1
  rdinstret a0
  rdinstret a1
  sub a1, a1, a0
  li a3, 1
  bne a1, a3, fail
  addi gp, gp, 1
  rdcycle a0
  rdcycle a1
  sltu a2, a0, a1
  beqz a2, fail
  rdcycleh a0
  CHECK a0, 0
  rdtime a0
  rdinstreth a0
  CHECK a0, 0
  csrw mscratch, a1
  csrr a2, mscratch
  addi gp, gp, 1
  bne a1, a2, fail
  csrrwi a2, mscratch, 5
  csrrsi a2, mscratch, 2
  csrr a2, mscratch
  CHECK a2, 7
  csrrci a2, mscratch, 1
  csrr a2, mscratch
  CHECK a2, 6
  fence
  fence rw, rw

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
smc:
  li a0, 7
  ret

.section .rodata
msg: .asciz "isa ok\n"
.p2align 2
word: .word 0x55555555

.data
.p2align 2
data:
  .byte 0x80, 0x7f
  .half 0x8001
  .word 0xdeadbeef
buf: .space 16

.bss
.p2align 2
bss_word: .space 4
