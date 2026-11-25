// Performance counters (hpmcounter16-31, CSRs 0xc10-0xc1f): each check
// reads a counter, runs a snippet with a known number of events and reads
// it again. Each check increments gp; on the first failure the program
// exits with gp (the failing check number); on success it prints
// "perf ok\n" and exits with 0. Event counts are exact; cycle counts only
// bounded (they depend on the DDR timing). A counter shows an event 2
// cycles later: WAIT before reading.
#include "macros.h"

#define C_IFILL  0xc10
#define C_ICYC   0xc11
#define C_ISW    0xc12
#define C_DFILL  0xc13
#define C_DCYC   0xc14
#define C_DWB    0xc15
#define C_DBUSY  0xc16
#define C_HHIT   0xc17
#define C_REPLAY 0xc18
#define C_LDUSE  0xc1a
#define C_MWAIT  0xc1b
#define C_DIV    0xc1c
#define C_BRMISS 0xc1d
#define C_JALR   0xc1e
#define C_HFIND  0xc1f

// a set of the D$ (32 KiB, 2-way: 16 KiB apart), far from the program
#define DA 0x00100000
#define DB 0x00104000
#define DC 0x00108000

.macro CHK reg, exp
  addi gp, gp, 1
  li a3, \exp
  bne \reg, a3, fail
.endm

// \reg in [lo, hi]
.macro CHKR reg, lo, hi
  addi gp, gp, 1
  li a3, \lo
  blt \reg, a3, fail
  li a3, \hi
  blt a3, \reg, fail
.endm

// time for the counters to take the events in
.macro WAIT
  nop
  nop
  nop
  nop
.endm

// s3 = counter \c now - s2
.macro DELTA c
  WAIT
  csrr s3, \c
  sub s3, s3, s2
.endm

.macro HFIND rd, rs1, rs2
  .insn r 0x0b, 0, 0, \rd, \rs1, \rs2
.endm

.section .text.start
.globl _start
_start:
  li gp, 0
  la s1, buf
  lw t0, 0(s1)              // the D$ line of buf
  lw t0, 0(s1)

  // ---- load-use: 4 stall cycles (the pairs in one I$ line: a fill
  // between a load and its use would hide the bubble)
  WAIT
  csrr s2, C_LDUSE
.p2align 5
  lw t0, 0(s1)
  addi t1, t0, 1
  lw t0, 0(s1)
  addi t1, t0, 1
  lw t0, 4(s1)
  add t1, t1, t0
  lw t0, 0(s1)
  sw t0, 8(s1)              // store data
  DELTA C_LDUSE
  CHK s3, 4

  // ---- divider: one DIV
  li t1, 1000
  li t2, 7
  WAIT
  csrr s2, C_DIV
  div t0, t1, t2
  DELTA C_DIV
  CHKR s3, 8, 40
  CHK t0, 142

  // ---- a forward branch taken (predicted not taken), a backward one not
  // taken (predicted taken): 2 mispredicts
  WAIT
  csrr s2, C_BRMISS
  beq x0, x0, 1f
  nop
1:
  j 3f
2:
  nop
3:
  li t0, 1
  beqz t0, 2b
  DELTA C_BRMISS
  CHK s3, 2
  // the machine-mode alias reads the same counter
  WAIT
  csrr s2, C_BRMISS
  csrr s3, 0xb1d
  sub s3, s3, s2
  CHK s3, 0

  // ---- JALR: one with an offset (redirected from E), one with offset 0
  // whose target buffer entry is cold (redirected from M)
  la t0, jt1 - 4
  la t1, jt2
  WAIT
  csrr s2, C_JALR
  jalr x0, 4(t0)
  j fail
jt1:
  jalr x0, 0(t1)
  j fail
jt2:
  DELTA C_JALR
  CHK s3, 2

  // ---- D$: two stores to fresh lines of one set, then a load from a
  // third (its victim is dirty): 3 fills, 1 write-back
  li t0, DA
  li t1, DB
  li t2, DC
  WAIT
  csrr s2, C_DFILL
  csrr s4, C_DWB
  csrr s5, C_DCYC
  csrr s6, C_DBUSY
  csrr s7, C_REPLAY
  sw t0, 0(t0)
  sw t1, 0(t1)
  lw t3, 0(t2)
  // let a write-back that goes on after the fill finish
  li t4, 100
4:
  addi t4, t4, -1
  bnez t4, 4b
  DELTA C_DFILL
  CHK s3, 3
  mv s2, s4
  DELTA C_DWB
  CHK s3, 1
  mv s2, s5
  DELTA C_DCYC
  CHKR s3, 12, 100000
  mv s8, s3
  mv s2, s6
  DELTA C_DBUSY
  addi gp, gp, 1
  blt s3, s8, fail          // busy covers the fills
  mv s2, s7
  DELTA C_REPLAY
  CHKR s3, 3, 100000
  lw t3, 0(t0)              // the write-back kept the data
  CHK t3, DA
  lw t3, 0(t1)
  CHK t3, DB

  // ---- I$: a jump to a line not yet fetched: 1 fill
  j 5f
.p2align 5
5:
  WAIT
  csrr s2, C_IFILL
  csrr s4, C_ICYC
  j far
far_back:
  CHK s3, 1
  mv s2, s4
  DELTA C_ICYC
  CHKR s3, 4, 100000

  // ---- I$ way switch: blocks A and B 32 KiB apart share a set (64 KiB,
  // 2-way); A, B, then A again hits in the way not predicted: 1 switch
  // (and a line fill, of blk_c, is none)
  la t0, blk_a
  la t1, blk_b
  jalr ra, 0(t0)
  jalr ra, 0(t1)
  WAIT
  csrr s2, C_ISW
  jalr ra, 0(t0)
  call blk_c
  DELTA C_ISW
  CHK s3, 1

  // ---- p8.hfind: a table with no hash part (answered after its header)
  la t0, heap
  csrw 0x7c0, t0
  li t1, 0                  // table at offset 0
  WAIT
  csrr s2, C_HFIND
  csrr s4, C_MWAIT
  HFIND a0, t1, t1
  DELTA C_HFIND
  CHK s3, 1
  CHK a0, 0
  mv s2, s4
  DELTA C_MWAIT
  CHKR s3, 2, 100000

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

.p2align 5
far:
  WAIT
  csrr s3, C_IFILL
  sub s3, s3, s2
  j far_back

.p2align 5
blk_c:
  ret

// set 2 of the I$ (byte address 0x40 mod 32 KiB: only _start's second
// line shares it, and it runs once before)
.org 0x8040
blk_a:
  ret
.org 0x10040
blk_b:
  ret

.section .rodata
msg: .asciz "perf ok\n"

.data
.p2align 5
buf: .word 3, 4, 0, 0
.p2align 5
// a table header: nodes (+16) = 0
heap: .word 0, 0, 0, 0, 0, 0, 0, 0
