#!/usr/bin/env python3
"""Flatten an RV32 ELF into an image starting at address 0, as the FPGA
core runs it (boot.rom is built the same way). Used for the CPU
self-tests (sim/cpu-tests.sh).

usage: scripts/flatten-elf.py ELF OUT

Loadable segments are placed at their physical (load) address, which is
what the core uses.
"""
import struct
import sys

if len(sys.argv) != 3:
    sys.exit(__doc__)
elf, out = sys.argv[1], sys.argv[2]

b = open(elf, "rb").read()
if b[:4] != b"\x7fELF" or b[4] != 1 or b[5] != 1:
    sys.exit(f"{elf}: not a 32-bit little-endian ELF")
phoff, = struct.unpack_from("<I", b, 28)
phentsize, phnum = struct.unpack_from("<HH", b, 42)
img = bytearray()
for i in range(phnum):
    p_type, p_offset, _vaddr, p_paddr, p_filesz, _memsz, _flags, _align = struct.unpack_from("<8I", b, phoff + i * phentsize)
    if p_type != 1 or p_filesz == 0:
        continue
    if p_paddr + p_filesz > 0x0060_0000:
        sys.exit(f"segment at {p_paddr:#x} ({p_filesz} bytes) does not fit below 0x600000 (.data/.bss, stack)")
    if len(img) < p_paddr + p_filesz:
        img.extend(bytes(p_paddr + p_filesz - len(img)))
    img[p_paddr:p_paddr + p_filesz] = b[p_offset:p_offset + p_filesz]
open(out, "wb").write(img)
print(f"{out}: {len(img)} bytes")
