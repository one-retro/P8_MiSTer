# P8 hardware

The core's view from the CPU: memory map, MMIO registers, the
blitter and the audio unit. The runtime (`boot.rom`) is the only software that uses them;
they are documented here as the interface of the core. Where a row says
"simulator", it refers to the software model the runtime is developed on.

## Memory map

| range | size | what | timing |
|-------|------|------|--------|
| 0x0000_0000 - 0x01ff_ffff | 32 MiB | SDRAM (on the FPGA: HPS DDR3 at 0x3000_0000): code, data, heap, stack | through I$/D$ |
| 0x1000_0000 - 0x1000_ffff | 64 KiB | PICO-8 RAM (dual-port BRAM shared with video/gfx/audio) | uncached, fixed latency |
| 0x2000_0000 - 0x2000_0fff | 4 KiB  | MMIO | uncached |

The HPS downloads `boot.rom` (the BIOS) to address 0 and the OSD's carts
into the cart buffer at 0x0100_0000 (`rtl/p8_loader.sv`; an 8 MiB window,
see LOAD_CTRL); without the BIOS, a `.p8c` from *Load Compiled Cart* goes
to address 0. Every download holds the CPU in reset and restarts it at 0.

## MMIO registers (32-bit, word access)

### MMIO registers (32-bit, word access)

| offset | name | R/W | meaning |
|--------|------|-----|---------|
| 0x000 | CONSOLE   | W | write a byte to the debug console (stdout in the simulator) |
| 0x004 | EXIT      | W | stop the machine with this exit code |
| 0x008 | CATEGORY  | W | cycle accounting category for subsequent cycles (0=cpu/lua, 1=gfx (would be RTL), 2=system, 3=audio synthesis) |
| 0x00c | FRAME     | W | end of a PICO-8 frame: the screen is presented at the next vblank |
| 0x010 | CYCLE_LO  | R | cycle counter low |
| 0x014 | CYCLE_HI  | R | cycle counter high |
| 0x018 | CART_SIZE | R | byte size of the cart file at 0x0100_0000 |
| 0x01c | BUTTONS   | R | button bits for the current frame (player 0 in bits 0-5, player 1 in 8-13); bit 31 (FPGA): pause, the OSD is open and the "Pause when OSD open" option is on; bit 30: the OSD's "Cart menu" was chosen, bit 19 / 18: its "Save state" / "Load state" (each until BUTTONS is written), bits 17-16: the save state slot (OSD option); bits 29-28 (FPGA): the OSD mouse speed (0 normal, 1 slow, 2 fast, 3 faster) |
| 0x020 | FRAMES    | R | number of frames the firmware should run (0 = forever) |
| 0x024 | MARK      | W | debug marker (simulator logs value and cycle count) |
| 0x028 | VCOUNT    | R | vblank counter (FPGA: +1 at every 60 Hz vblank, after a requested present is copied; simulator: +1 per read) |
| 0x02c | AUDIO     | R/W | write: push one sample (low 16 bits, signed) to the 2047-entry FIFO played at 22050 Hz; read: samples queued |
| 0x030 | SPINNER   | W   | FPGA: bit 15 on, bits 14-8 y, 6-0 x of a 5x5 spinner the video hardware animates over the display until the next present (loading screen, while the cart's top-level code and `_init` run); the simulator ignores it |
| 0x034 | MOUSE     | R   | FPGA: mouse movement counters (wrapping): bits 11-0 x (right positive), 23-12 y (up positive), 31-27 wheel (down positive), and the buttons in bits 26-24 (1 left, 2 right, 4 middle); the firmware samples it once per frame for the dev-kit mouse (`stat(32..39)`). Simulator: 0 |
| 0x038 | KEY       | R/W | FPGA: oldest queued keyboard event (16-entry FIFO): bit 31 valid, bit 9 pressed, bit 8 E0-extended, bits 7-0 PS/2 set-2 code; write: drop it. The firmware turns them into typed characters (`stat(30)`/`stat(31)`) and held keys (`stat(28)`). Simulator: 0 |
| 0x040 | SAVE_CTRL | R/W | FPGA save file (hps_io virtual disk 0, which main mounts as `saves/P8/<cart>.sav` after each cart load): read: bit 31 a save file is mounted (or will be created on the first write) since the last download, bit 30 read-only, bit 0 busy; write: 1 reads block SAVE_LBA into the buffer, 2 writes the buffer to it (ignored while busy). Simulator: 0 |
| 0x044 | SAVE_LBA  | W   | block number (512 bytes) for SAVE_CTRL |
| 0x048 | SAVE_SIZE | R   | size of the save file in bytes (0: not created yet) |
| 0x04c | SAVE_ADDR | W   | word pointer (0-127) into the block buffers |
| 0x050 | SAVE_DATA | R/W | read: the word at SAVE_ADDR of the block read last (read twice after setting SAVE_ADDR: the RAM output follows a cycle later); write: the word at SAVE_ADDR of the block to write, then SAVE_ADDR + 1 |
| 0x060 | LOAD_CTRL | R/W | the OSD's cart downloads: read: bits 5-0 slot of the last one (1 *Load Cart*: `.p8` / `.p8.png`, 2 *Load Compiled Cart*: `.p8c`), bit 8 BIOS present (written; 0 after the core starts), bits 31-16 downloads into the cart buffer so far; write: bit 0 the BIOS is present (its carts then go to the cart buffer, also a `.p8c`). Simulator: the cart is download 1 |
| 0x064 | LOAD_SIZE | R   | size of the last download into the cart buffer |
| 0x068 | SS_ADDR   | W   | save state port (FPGA): the 64-bit beat address in the save state area (DDR 0x3400_0000, MiSTer Main's four slots of 16 MiB: beat = slot * 0x20_0000 + offset / 8); ignored while the port is busy |
| 0x06c | SS_LO     | R/W | write: the low word of the beat to write; read: the low word of the beat read last (the load waits until the port is free) |
| 0x070 | SS_HI     | R/W | write: writes the beat {SS_HI, SS_LO} at SS_ADDR, then SS_ADDR + 1 (ignored while the port is busy: read SS_LO first); read: the high word of the beat read last (waits likewise) |
| 0x074 | SS_READ   | W   | reads the beat at SS_ADDR (into SS_LO / SS_HI), then SS_ADDR + 1 (ignored while busy) |
| 0x100 | BLT_ID    | R   | blitter: `0xb17e01` in bits 31-8, feature bits in 7-0 (1 blit, 2 fill, 4 glyph, 8 sspr, 16 map, 32 tline, 64 circle, 128 oval); 0 = no blitter |
| 0x104 | BLT_CTRL  | R/W | blitter: write an operation (bits 3-0: 1 blit, 2 fill, 3 glyph, 4 sspr, 5 map row, 6 tline, 7 circle, 8 oval) to start it; read: bit 0 busy |
| 0x108 - 0x1bb | BLT_* | W | blitter operation registers (below) |
| 0x200 | APU_ID    | R   | audio unit: `0x415055` ("APU") in bits 31-8, version in 7-0; 0 = no audio unit |
| 0x204 - 0x21f | APU_* | R/W | audio unit registers (below) |

## Blitter (0x100-0x1bb)

`rtl/p8_blit.sv` draws into PICO-8 RAM in hardware: unscaled sprite
blits (`spr`), `map` a row of tiles at a time (the blitter reads the
tile numbers and, with layers, the sprite flags), scaled blits (`sspr`),
horizontal and vertical `tline`s over maps of up to 256 x 256 tiles,
filled circles (`circfill`: the midpoint walk and its spans), filled ovals
(`ovalfill` of boxes up to 256 pixels: the exact rounding walks of the
software path, kept as differences so they need only additions; the start
values and stop limits come from a few products of the half axes), fills (`cls`,
`rectfill`, spans of 48+ pixels, or 8+ with a pattern / bitplane mask, from
lines, circles, ovals) and font glyphs (`print`), with clipping, flips,
the draw palette and transparency, sheet / screen remapping (0x5f54 /
0x5f55), fill patterns of shapes and bitplane masks (0x5f5e). The runtime
keeps everything else and computes the registers: camera,
clipping, palettes, P8SCII parsing. It falls back to drawing in software
when the fill pattern applies to sprites (0x5f33 & 2), when the sprite
sheet overlaps the screen, for map rows whose tile numbers (or, with
layers, the sprite flags) are on the screen, for wide/tall/inverted/solid-background text,
diagonal `tline`s and those over larger maps, short spans, single pixels,
and out-of-range coordinates (huge `sspr` sizes, negative source sizes).

The CPU writes the registers and the operation to BLT_CTRL and goes on;
before writing the next operation's registers it polls BLT_CTRL until the
blitter is idle (one operation at a time, no queue). While busy the
blitter owns both ports of PICO-8 RAM: the SoC holds the CPU's loads and
stores to it, from the cycle after the CTRL store until the last write (a
load that missed the read port is replayed), and the vblank copy waits.
Code that does not touch PICO-8 RAM (Lua, the heap, audio) runs on. Throughput: a blit takes `rows * (1 + 2 * words) + 5` cycles
(8 pixels per destination word; an 8x8 sprite 29 or 45 cycles), a fill or
glyph `rows * words + 5`, a map row `tiles + 2` (+ `tiles + 2` with
layers) `+ 1`, then per pixel row 1 + per run of destination words that
show a drawn tile 1 + 2 per word, sspr one cycle per pixel plus one per row and the
source row's words when the row changes, a tline one cycle per pixel plus
one per destination word (per pixel when vertical) and 4 per tile change
(7 with layers), + 6, a circle one cycle per step of the walk (about
0.7 r) and one to end it, plus per visible span one cycle and one per
destination word; busy ends 2 cycles after the walk or 5 after the last
word read, whichever is later; an oval 3 cycles (the products) and 1 per
walk, per step of a walk (about 0.35 of the box's width plus height in
all) one cycle per move of the rounding and 5, the visible rows as for a
circle, and 3 between the walks.

| offset | name | meaning |
|--------|------|---------|
| 0x108 | DST   | 13-0 first destination word address (PICO-8 byte address / 4), 17-16 first row & 3 |
| 0x10c | SIZE  | 3-0 words per row - 1, 14-8 rows - 1, 23-16 / 31-24 pixel masks of the first / last word |
| 0x110 | SROW  | blit: 11-0 first sheet row (signed); 12 rows go up; 23-16 sheet base (0x5f54) |
| 0x114 | SCOL  | blit: 11-0 first sheet word (signed); 12 flip x; 18-16 window shift |
| 0x120 | PAL0-3   | write table: byte `c` = `keep << 4 \| val` for colour `c` (a pixel becomes `p & keep \| val`) |
| 0x130 | FKEEP0-3 | fill/glyph ink per row & 3: keep nibbles of 8 pixels |
| 0x140 | FVAL0-3  | fill/glyph ink: value nibbles |
| 0x150 | GMASK0-3 | glyph: 16-bit pixel mask per row (2 rows per word) |
| 0x160 | SXQ, SXS, SYQ, SYS, SDIV, SLW | sspr: column / row DDAs (start, step, divisor) and the sheet words to load per row |
| 0x178 | MAP0 | map row: 15-0 map pointer of the first tile (address = pointer & 0xffff for a map at 0x8000+, else 0x1000 + (pointer & 0x1fff)), 16 high map, 28-24 tiles - 1 (SROW 2-0: first pixel row in the tiles, SCOL: as a blit over the row's pixels) |
| 0x17c | MAP1 | map row: 7-0 layers (0 = all tiles), 8 draw tile 0 (0x5f36 & 8) |
| 0x180 | TL0-1 | tline: map x, y of the first pixel (16.16, masked); PAL is the write table, GMASK the one for pattern bits of 1, SROW 23-16 the sheet base |
| 0x188 | TL2-3 | tline: their steps per pixel |
| 0x190 | TL4-5 | tline: x, y masks (`(width << 16) - 1`) |
| 0x198 | TL6 | tline: 7-0 / 15-8 x / y map offset (0x5f3a-b), 23-16 / 31-24 map width / height - 1 (powers of two) |
| 0x19c | TL7 | tline: 15-0 map base (as MAP0), 16 high map, 17 draw tile 0, 23-20 log2 map width, 31-24 layers |
| 0x1a0 | TL8 | tline: 15-0 fill pattern, 23-16 pixels - 1, 24 vertical, 25 backwards, 30-28 first nibble |
| 0x1a4 | CR0 | circle: 15-0 / 31-16 centre x / y (signed, after the camera); FKEEP / FVAL are the ink, DST 13-0 the screen's first word |
| 0x1a8 | CR1 | circle: 10-0 radius |
| 0x1ac | CR2 | circle: clip rectangle, 7-0 x0, 15-8 x1, 23-16 y0, 31-24 y1 (x0 <= x < x1, likewise y) |
| 0x1b0 | OV0 | oval: 15-0 / 31-16 x0 + x1 / y0 + y1 (the doubled centre, signed, after the camera); FKEEP / FVAL / DST as for a circle |
| 0x1b4 | OV1 | oval: 7-0 / 15-8 max(x1 - x0, 2) / max(y1 - y0, 2) (< 256) |
| 0x1b8 | OV2 | oval: clip rectangle (as CR2) |

The runtime keeps a bit-exact software model of the blitter (including
the cycle counts above); the RTL regression compares the two.

## Audio unit (0x200-0x21f)

`rtl/p8_apu.sv` does the per-sample work of the audio: the oscillators (the
8 waveforms, their buzz variants, the detune filter's second wave), noise,
a channel's volume, its effects (reverb, dampen, distort) and the mix of the
4 channels. The runtime runs the sequencer (notes, note effects, custom
instruments, music) and, for each block of up to 32 samples that crosses
no note boundary, describes each sounding channel and queues a RUN, then a
PUSH of the block to the FIFO (0x02c, which it shares with the CPU: a CPU
write in the same cycle goes first).

| offset | name | meaning |
|---|---|---|
| 0x200 | ID    | R: `0x415055` << 8 \| version (1) |
| 0x204 | CTRL  | W: queue an operation: 4-0 samples - 1, 9-8 op (0 RUN, 1 PUSH, 2 READ, 3 RESET), with INC..FX as they are now; R: bit 0 busy (an operation queued, running, or being written), bit 1 the queue is full (253 entries waiting) |
| 0x208 | INC   | oscillator phase step per sample (one cycle = 1 << 24) |
| 0x20c | INC2  | the detune wave's phase step |
| 0x210 | VOICE | 2-0 wave, 5-3 detune wave, 6 buzz, 7 detune, 9-8 noise kind (0 pitched, 1 white, 2 brown), 18-10 volume (0-256), 19 the channel's second oscillator, 20 voice on |
| 0x214 | NOISE | pitched noise: 12-0 lowpass coefficient `a` (Q12), 27-16 gain `g` (Q8) |
| 0x218 | FX    | 5-0 effects (1 reverb 1, 2 reverb 2, 4 dampen 1, 8 dampen 2, 16 distort, 32 distort high), 6 effects on, 9-8 channel, 17-10 / 20-18 dampen 1 coefficient `a` (Q8) / shift `g`, 28-21 / 31-29 dampen 2 |
| 0x21c | ACC   | R: the next sample READ took; W: advance to the one after it (it is there 2 cycles later) |

Operations (all arithmetic in two's complement, `>>` arithmetic,
divisions rounding to 0):

- RUN n: channel FX 9-8 for n samples, added to accumulator samples
  0..n-1. With VOICE 20 the oscillator (each channel has two: VOICE 19
  picks the second; each keeps its phase, its detune wave's phase and its
  noise state `s`) makes a sample per step, with `t` = phase bits 23-8:
  - volume 0: the phase advances by INC, nothing else changes;
  - wave 6 (noise): a 32-bit xorshift (`r ^= r << 13; r ^= r >> 17; r ^=
    r << 5`, one generator for all channels) gives `white = r >> 15`
    (signed); pitched: `s += (white - s) * a >> 12`, sample `(s * g >> 8) *
    vol >> 9`; white: `s = white`, sample `white * vol >> 9`; brown: `s +=
    (white - s) >> 4`, sample `s * 4 * vol >> 9`; the phase advances by INC;
  - other waves: the wave at the phase, plus with detune half (`>> 1`) the
    detune wave at its phase (which advances by INC2), times the volume `>>
    9`, clamped to 16 bits; the phase advances by INC. The waves (the
    `_buzz` variant where it differs; `tilt(a, up, down)` is `t * up >> 14 -
    65536` below `a`, else `(65536 - t) * down - 65536`):
    - 0 triangle: `tri = |4t - 131072| - 65536`, `tri >> 1`; buzz `(3 tri -
      tilt(57344, 37449, 16)) >> 3`
    - 1 tilted saw: `tilt(58982, 36409, 20) >> 1`; buzz `tilt(63898, 33608,
      80) >> 1`
    - 2 saw: `r = t` as signed 16 bits, `r * 42795 >> 16`; buzz first `r = (r
      * 54395 >> 16) - (5571 if 32768 < phase bits 24-8 < 98304)`
    - 3 square: `16384` while `t < 32768` (buzz 26214), else `-16384`
    - 4 pulse: likewise with 21845 (buzz 16712)
    - 5 organ: `r = 196608 - |24t - 393216|` while `t < 32768`, else `65536
      - |16t - 786432|`; buzz: below 32768 `r = 2r + 196608`, then `r * 3277
      >> 14 - 65536` if `r > -122880`, else `r + 32768`; above, `r + 32768`;
      then `r * 7282 >> 16`
    - 6, 7 phaser: `k = |2 * (phase bits 30-15) - 65536|`, `u = (t + (k >> 1))
      & 0xffff`, `r = |4u - 131072| - |8t - 262144|`; buzz adds `24576 -
      |f1 - 32768| - |(phase bits 21-7) - 16384|` with `f1` = phase bits
      22-7 with bit 15 flipped; then `r * 5461 >> 15`

  With FX 6 the sample (0 without a voice) goes through the effects, per
  channel: `v = x`, plus half of the sample written 366 samples ago with
  reverb 1 and of the one written 732 ago with reverb 2 (a 732-sample delay
  line); clamped to 16 bits, written to the line; dampen 1: `lp += ((v +
  v1) >> 1 - lp) * a >> 8`, `v1 = v`, `v = lp`, plus `(v - lp) >> g` when
  `g` > 0; dampen 2 likewise with its own `a` and `g` (a dampen that is off
  is cleared at the start of the RUN); distort: `v / 4096 * 4681`, distort high (without distort)
  the same of `v - 4096` for negative `v`. Without FX 6 the delay line and
  its position stand still.
- PUSH n: accumulator samples 0..n-1, clamped to 16 bits, to the FIFO, and
  cleared.
- READ: all 32 accumulator samples, clamped, to the ACC output, and cleared.
- RESET: oscillators, generator (`0x2545f491`), effects, delay lines and
  accumulator to their initial state, as after the core's reset.

From its start to the start of the next queued operation, RUN n takes `5 +
n * (2 + osc + fx)` cycles (`osc`: 10 per wave, 7 for noise, plus 3; 0 when
the voice is off or its volume 0; `fx`: 14 with FX 6), PUSH n `2n + 2`
(more while the CPU writes the FIFO), READ 66, RESET 4098.

The runtime keeps a bit-exact software model of the audio unit (including
these cycle counts); `sim/apu_tb.cpp` replays its operation traces on the
RTL and checks every sample and the cycles.
