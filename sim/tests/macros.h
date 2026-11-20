// Shared helpers for the assembly tests.
#define MMIO 0x20000000
#define R_CONSOLE 0
#define R_EXIT 4
#define R_CATEGORY 8
#define R_FRAME 12
#define R_CYCLE_LO 16
#define R_CYCLE_HI 20
#define R_CART_SIZE 24
#define R_BUTTONS 28
#define R_FRAMES 32
#define R_MARK 36

// Write \val (register) to MMIO register \reg; clobbers t6.
.macro MMIO_W reg, val
  li t6, MMIO
  sw \val, \reg(t6)
.endm

.macro EXIT_IMM code
  li t6, MMIO
  li t5, \code
  sw t5, R_EXIT(t6)
.endm
