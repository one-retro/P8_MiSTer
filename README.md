# P8 for MiSTer

P8 is 1Retro's fantasy console core for
[MiSTer](https://github.com/MiSTer-devel/Wiki_MiSTer/wiki). It plays
PICO-8 carts: a soft RISC-V CPU at 100 MHz with a blitter and an audio unit runs carts that
its BIOS (`boot.rom`, distributed with each release as a binary) compiles
on the core.

PICO-8 is a fantasy console by [Lexaloffle Games](https://www.lexaloffle.com/pico-8.php);
see their site for PICO-8 itself, its manual and the carts on its BBS. 1Retro's
P8 is not affiliated with or endorsed by Lexaloffle Games. PICO-8 is a
trademark of Lexaloffle Games.

This repository is the FPGA core: the RTL, the MiSTer framework it is built
on, the Quartus project and a Verilator simulation.

## Install

Point the MiSTer Downloader at P8 once. Every `update_all` run from then
on installs and upgrades the core and the BIOS along with everything else on
the card. Over SSH, that is one line:

```sh
wget -O /media/fat/downloader_p8.ini \
  https://raw.githubusercontent.com/one-retro/P8_MiSTer/db/downloader_p8.ini
```

Or save [`downloader_p8.ini`](https://raw.githubusercontent.com/one-retro/P8_MiSTer/db/downloader_p8.ini)
onto the card as `/media/fat/downloader_p8.ini` from a desktop. Then run
**Update All** (or **Downloader**) from the Scripts menu: the core lands in
`_Console/` and the BIOS in `games/P8/boot.rom`. Your carts and saves are never touched by an update.

By hand instead: copy `P8_<date>.rbf` from a
[release](https://github.com/one-retro/P8_MiSTer/releases) to
`/media/fat/_Console/`, and its `boot.rom` to `/media/fat/games/P8/`.

## Using it

Start **P8** from the **Console** folder of the main menu. The BIOS starts and
says how to load a cart. Put carts in `/media/fat/games/P8/` and load
them from the OSD (F12):

- **Load Cart**: a `.p8` or `.p8.png`. The BIOS compiles it on the core
  (a few seconds, with a progress bar) and runs it.
- **Load Compiled Cart**: a `.p8c`, a cart compiled ahead of time, which
  runs at once. A `.p8c` can bundle the other carts a game loads.

**Reset** (or the USER button) starts the loaded cart again without
compiling it again. When a cart loads another cart that it does not
bundle, the BIOS shows which one to pick: load it from the OSD and the game
goes on with what the first cart passed to it.

On the card, cart data is saved in `saves/P8/` (one file per cart) and the
OSD settings in `config/P8.CFG`.

Controls: the joystick's d-pad, O, X and Pause buttons (Pause is Start by
default), or a keyboard laid out as in PICO-8 (player 2 too). A USB mouse
and keyboard work for carts that use them. Pause opens the cart's pause
menu (continue, the cart's own entries, reset cart).

OSD options:

- **Cart menu**: opens the same pause menu, for controllers without a
  Pause button mapped.
- **Save state** / **Load state** and **State slot** (1-4): the running
  cart as it is, in `savestates/P8/<cart>_<slot>.ss`. A state loads
  only into the cart and the core (and BIOS) version it was saved with.
  On a keyboard, Alt+F1 to Alt+F4 save into slots 1-4 and F1 to F4 load
  them.
- **Aspect ratio** and **Scale**: the framework's video scaling
  (V-Integer gives a sharp integer scale).
- **Scandoubler Fx**: HQ2x and scanlines for VGA output.
- **Show FPS**: the frames the cart presented in the last second, top left.
- **Pause when OSD open** (on by default): the cart, its clock and its
  sound stop while the OSD is shown.
- **Mouse speed**: how fast the mouse pointer moves.

No SDRAM module is needed: the core uses 32 MiB of the HPS DDR3, and
64 MiB more for the save state slots.

## Design (`rtl/`)

| file | what |
|---|---|
| `p8.sv` | MiSTer glue (`emu`): hps_io, PLL, joystick/keyboard mapping, video_mixer, audio |
| `rtl/p8_soc.sv` | CPU, caches, PICO-8 RAM, MMIO, reset/download sequencing |
| `rtl/rv32_cpu.sv` | RV32IM + Zbb pipeline (D/E/M/W), forwarding, branch prediction (static rule + 1K agree counters), Zbb in M, 1-cycle multiply (DSP), radix-4 divider (19 cycles; also `p8.fdiv`, the PICO-8 16.16 division), FENCE.I, counters + mscratch CSRs, performance counters (CSRs 0xc10-0xc1f, see `docs/hardware.md`), `p8.hfind` (custom-0) and its `p8heap` CSR (0x7c0), the `p8feat` CSR (0x7c1) listing the extensions |
| `rtl/p8_icache.sv` | 64 KiB 2-way (LRU) I$, 32-byte lines, predecoded branch targets |
| `rtl/p8_tunit.sv` | table unit: runs `p8.hfind` (hash-part lookup of a string or number key) through the D$, 64 bits per access, with a 256-entry lookup cache of where keys were found |
| `rtl/p8_dcache.sv` | 32 KiB 2-way (LRU) write-back D$, 32-byte lines (a dirty victim goes to a buffer and is written back after the fill), write-back-all for FENCE.I, line zeroing for `p8.zline` |
| `rtl/p8_ddr_arb.sv` | DDR3 (Avalon, 64-bit) arbiter: D$, I$, loader |
| `rtl/p8_loader.sv` | HPS downloads into DDR: `boot.rom` / `boot1.rom` (the BIOS) at address 0, the OSD's carts into the cart buffer (or a `.p8c` at 0 without the BIOS) |
| `rtl/p8_blit.sv` | blitter: sprite blits, map rows (reads the tile numbers and flags itself), sspr, fills and glyphs in PICO-8 RAM, 8 pixels per access (sspr 1 pixel per cycle); the CPU waits for each operation (MMIO 0x100-0x1bb) |
| `rtl/p8_video.sv` | vblank present (copies the draw state and screen into a display buffer), scanout, screen and raster modes; power-up screen (`rtl/splash.p8` -> `splash.hex`); FPS overlay |
| `rtl/p8_audio.sv` | 22050 Hz sample FIFO, fed by the audio unit (or the CPU) |
| `rtl/p8_apu.sv` | audio unit: oscillators, noise, the detune wave, volume, reverb / dampen / distort and the mix, per sample, from a queue of operations (MMIO 0x200-0x21f) |
| `rtl/p8_bram.sv` | simple dual-port block RAM with read-new-data bypass |

The register-level interface (memory map, MMIO registers, performance counters, blitter, audio unit) is in
[`docs/hardware.md`](docs/hardware.md). Clock: 100 MHz (`clk_sys`, also
the DDR3 port clock); the video is a 15 kHz / 60 Hz raster.

## Building

Quartus Prime Lite 17.0.2 and Verilator run in Docker:

```sh
make          # the core: output_files/p8.rbf (scripts/quartus.sh runs Quartus)
make test     # the CPU self-tests on the Verilator model
make clean
```

Timing at 100 MHz depends on the seed (`SEED` in `p8.qsf`): with little
margin, any change, even the contents of a RAM, can move the placement. See
[`docs/timing.md`](docs/timing.md) for what keeps the critical paths short.
The power-up screen (`rtl/splash.hex`, the display buffer's initial
contents, shown while no program runs) can be replaced without a new fit:
`make splash` puts it into the compiled project and reruns
only the assembler, keeping the placement and timing.

## Simulation (`sim/`)

The Verilator model covers `p8_soc` (everything except the MiSTer
framework) with a DDR3 model (latency and random wait states), the HPS
downloads (`--image` as `boot.rom`, `--download FRAME:INDEX:FILE` as the
OSD's), and video/audio capture. `sim/obj_dir/apu/Vp8_apu` runs the audio
unit alone on a trace of operations and checks its samples and cycles
(`sim/apu_tb.cpp`).

```sh
make sim      # the mister-p8-verilator image (Verilator 5, clang/lld), sim/obj_dir/Vp8_soc and apu/Vp8_apu
make test     # RV32IM and Zbb self-tests (sim/cpu-tests.sh)
sim/obj_dir/Vp8_soc --image boot.rom --download 3:1:cart.p8 --frames 300 --ppm out.ppm
```

## Releases and the Downloader database

A release carries the core as `P8_<date>.rbf` and the BIOS as `boot.rom`.
Publishing one runs `.github/workflows/db.yml`, which hashes both with
`scripts/generate_db.py` and pushes `db.json.zip` and `downloader_p8.ini` to the
orphan `db` branch. The database id (`p8`) and its URL never change:
Downloader tracks installed files by them.

## License

`sys/` is the MiSTer framework from
[Template_MiSTer](https://github.com/MiSTer-devel/Template_MiSTer)
(commit `e901a9f`), copied unmodified, and `rtl/pll*` is the template's
PLL with only the output frequency changed. Its files carry their own
licences (GPL-2.0-or-later, and GPL-3.0-or-later for `hps_io.sv`,
`ddr_svc.sv`, `sd_card.sv` and `scandoubler.v`); `p8.sv` is derived from
the template's glue. The built core (`.rbf`) is therefore distributed
under the GPL-3.0 ([`LICENSE-GPL-3.0`](LICENSE-GPL-3.0);
[`LICENSE-GPL-2.0`](LICENSE-GPL-2.0) for the files under it). The rest of
`rtl/`, `sim/`, `scripts/` and `docs/` is MIT ([`LICENSE-MIT`](LICENSE-MIT)).

`boot.rom`, the BIOS, is a separate program that the core loads at run
time. It is not part of this repository and not covered by the GPL: each
release distributes it as a binary under the Apache License 2.0
([`LICENSE-APACHE-2.0`](LICENSE-APACHE-2.0)).
