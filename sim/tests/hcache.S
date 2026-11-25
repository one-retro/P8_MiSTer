// The table unit's lookup cache: p8.hfind answers what a probe from the
// key's hash finds whatever the cache holds. Repeated lookups hit (counted
// in hpmcounter23, 0xc17); a cached node now holding another key, a node
// array rebuilt elsewhere, the same array with another size, an absent
// key, number keys, two lookups sharing an entry, a new arena base, and
// D$ misses on the cached node and the header. Each check increments gp;
// on the first failure the program exits with gp; on success it prints
// "hcache ok\n" and exits with 0.
#include "macros.h"

#define C_HHIT 0xc17

// arena offsets: tables (header: nodes at +16, nmask at +20), node arrays
// (node i at +8 + 16 i: key tag, key, value tag, value), strings (hash at
// +8)
#define T1 0
#define T2 64
#define T3 128
#define N1 256
#define N2 512
#define N3 1536
#define S1 1024
#define S2 1056
#define S4 1088
#define S6 1152
#define HEAPSZ 2048

.macro CHK reg, exp
  addi gp, gp, 1
  li a3, \exp
  bne \reg, a3, fail
.endm

// \reg == s4 + \off
.macro CHKA reg, off
  addi gp, gp, 1
  li a3, \off
  add a3, a3, s4
  bne \reg, a3, fail
.endm

// p8.hfind rd, table, key (funct7 1: a number key)
.macro HFIND rd, t, k
  .insn r 0x0b, 0, 0, \rd, \t, \k
.endm
.macro HFINDN rd, t, k
  .insn r 0x0b, 0, 1, \rd, \t, \k
.endm

// string-key lookup of (\t, \k) (arena offsets) into a0; s2 = hits before
.macro LOOK t, k
  csrr s2, C_HHIT
  li a1, \t
  li a2, \k
  HFIND a0, a1, a2
.endm

// cache hits since LOOK == \n
.macro HITS n
  nop
  nop
  nop
  csrr s3, C_HHIT
  sub s3, s3, s2
  CHK s3, \n
.endm

// node \i of array \n: tag \tg, key \k
.macro NODE n, i, tg, k
  li t0, \n + 8 + 16 * \i
  add t0, t0, s4
  li t1, \tg
  sw t1, 0(t0)
  li t1, \k
  sw t1, 4(t0)
.endm

// table \t: nodes \n, nmask \m
.macro TABLE t, n, m
  li t1, \n
  sw t1, \t + 16(s4)
  li t1, \m
  sw t1, \t + 20(s4)
.endm

.section .text.start
.globl _start
_start:
  li gp, 0
  la s4, heap
  csrw 0x7c0, s4

  // strings: hashes 2 (S1, S2, S4) and 9 (S6)
  li t1, 2
  sw t1, S1 + 8(s4)
  sw t1, S2 + 8(s4)
  sw t1, S4 + 8(s4)
  li t1, 9
  sw t1, S6 + 8(s4)

  // ---- T1 -> N1 (8 nodes): S1 at 2, S2 at 3 (probed)
  TABLE T1, N1, 7
  NODE N1, 2, 4, S1
  NODE N1, 3, 4, S2
  LOOK T1, S1
  CHKA a0, N1 + 8 + 32
  HITS 0
  LOOK T1, S1
  CHKA a0, N1 + 8 + 32
  HITS 1
  LOOK T1, S2
  CHKA a0, N1 + 8 + 48
  HITS 0
  LOOK T1, S2
  CHKA a0, N1 + 8 + 48
  HITS 1

  // ---- the cached node now holds another key: S4 at 2, S1 at 4
  NODE N1, 2, 4, S4
  NODE N1, 4, 4, S1
  LOOK T1, S1
  CHKA a0, N1 + 8 + 64
  HITS 0
  LOOK T1, S1
  CHKA a0, N1 + 8 + 64
  HITS 1
  // a number-tagged node with S1's payload is not S1
  NODE N1, 4, 3, S1
  NODE N1, 5, 4, S1
  LOOK T1, S1
  CHKA a0, N1 + 8 + 80
  HITS 0

  // ---- rebuilt: T1 -> N2 (16 nodes), S1 at 2
  TABLE T1, N2, 15
  NODE N2, 2, 4, S1
  LOOK T1, S1
  CHKA a0, N2 + 8 + 32
  HITS 0
  LOOK T1, S1
  CHKA a0, N2 + 8 + 32
  HITS 1

  // ---- the same array with another size (nmask 7): S1 at 2 again, found
  // by the probe (the entry's nmask differs)
  TABLE T1, N2, 7
  LOOK T1, S1
  CHKA a0, N2 + 8 + 32
  HITS 0
  // the cached node is outside the array now (nmask 1, S1 at 1 after S4
  // at 0): not taken
  TABLE T1, N2, 1
  NODE N2, 0, 4, S4
  NODE N2, 1, 4, S1
  LOOK T1, S1
  CHKA a0, N2 + 8 + 16
  HITS 0

  // ---- absent: S2 is not in N2 (its entry is for N1); then S2 at 3 of
  // N2 (nmask 7), cached, and its node emptied: not found
  TABLE T1, N2, 7
  NODE N2, 1, 0, 0
  LOOK T1, S2
  CHK a0, 0
  NODE N2, 3, 4, S2
  LOOK T1, S2
  CHKA a0, N2 + 8 + 48
  LOOK T1, S2
  CHKA a0, N2 + 8 + 48
  HITS 1
  NODE N2, 3, 0, 0
  LOOK T1, S2
  CHK a0, 0
  HITS 0

  // ---- number keys: T2 -> N3 (8 nodes); 1.0 hashes to slot 0, 2.5 to 1
  TABLE T2, N3, 7
  NODE N3, 0, 3, 0x00010000
  NODE N3, 1, 3, 0x00028000
  csrr s2, C_HHIT
  li a1, T2
  li a2, 0x00010000
  HFINDN a0, a1, a2
  CHKA a0, N3 + 8
  HFINDN a0, a1, a2
  CHKA a0, N3 + 8
  li a2, 0x00028000
  HFINDN a0, a1, a2
  CHKA a0, N3 + 8 + 16
  HFINDN a0, a1, a2
  CHKA a0, N3 + 8 + 16
  HITS 2
  // a string key with a number key's payload: not the number's node
  li a2, 0x00010000
  HFIND a0, a1, a2
  CHK a0, 0

  // ---- two lookups with one entry: (T1, S1) and (T3, S6) (bits 11-4:
  // 0x00 ^ 0x40 and 0x08 ^ 0x48)
  TABLE T3, N1, 7
  NODE N1, 1, 4, S6         // S6 hashes to 9 & 7 = 1
  LOOK T1, S1
  CHKA a0, N2 + 8 + 32
  LOOK T3, S6
  CHKA a0, N1 + 8 + 16
  LOOK T1, S1
  CHKA a0, N2 + 8 + 32
  HITS 0
  LOOK T3, S6
  CHKA a0, N1 + 8 + 16
  HITS 0
  LOOK T3, S6
  CHKA a0, N1 + 8 + 16
  HITS 1

  // ---- a new arena base: a copy of the heap; the cached offsets point
  // into it, and its own nodes count
  la s5, heap2
  li t2, 0
1:add t3, s4, t2
  lw t4, 0(t3)
  add t3, s5, t2
  sw t4, 0(t3)
  addi t2, t2, 4
  li t3, HEAPSZ
  blt t2, t3, 1b
  mv s4, s5
  csrw 0x7c0, s4
  LOOK T3, S6
  CHKA a0, N1 + 8 + 16
  HITS 1
  // S6 moved in the copy only (slot 1 -> 2; slot 1 another key)
  NODE N1, 1, 4, S4
  NODE N1, 2, 4, S6
  LOOK T3, S6
  CHKA a0, N1 + 8 + 32
  HITS 0

  // ---- D$ misses: evict the cached node's line (two other lines of its
  // set, 16 KiB apart), then the header's too
  LOOK T3, S6
  CHKA a0, N1 + 8 + 32
  li t0, N1 + 8 + 32
  add t0, t0, s4
  li t1, 0x4000
  add t2, t0, t1
  lw t3, 0(t2)
  add t2, t2, t1
  lw t3, 0(t2)
  LOOK T3, S6
  CHKA a0, N1 + 8 + 32
  HITS 1
  li t0, T3
  add t0, t0, s4
  add t2, t0, t1
  lw t3, 0(t2)
  add t2, t2, t1
  lw t3, 0(t2)
  li t0, N1 + 8 + 32
  add t0, t0, s4
  add t2, t0, t1
  lw t3, 0(t2)
  add t2, t2, t1
  lw t3, 0(t2)
  LOOK T3, S6
  CHKA a0, N1 + 8 + 32
  HITS 1

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
msg: .asciz "hcache ok\n"

.data
.p2align 5
heap: .space HEAPSZ
.p2align 5
heap2: .space HEAPSZ
