# Timing

The core clock is 100 MHz (Quartus 17.0.2 Lite, 5CSEBA6U23I7). Worst
setup slack of the core clock, in ns, over seeds 1-11: the RTL before the
timing work of 2026-11-23 (described below), after it, and with the
memory system of 2026-11-24 (D$ request with the miss and a write-back
buffer, radix-4 divider, the table unit's lookup cache, performance
counters; see the end):

| seed       | 1      | 2      | 3      | 4      | 5      | 6      | 7      | 8      | 9      | 10     | 11     |
|------------|--------|--------|--------|--------|--------|--------|--------|--------|--------|--------|--------|
| before     | +0.028 | +0.029 | -0.214 | +0.007 | -0.488 | -0.013 | -0.405 | +0.132 | -0.327 | -0.101 | -0.589 |
| 2026-11-23 | +0.179 | -0.383 | +0.164 | +0.290 | +0.271 | +0.174 | +0.209 | +0.322 | -0.379 | -0.013 | -0.674 |
| 2026-11-24 | -0.190 | -0.085 | -0.065 | +0.072 | -0.137 | +0.080 | -0.062 | -0.210 | +0.149 | -0.017 | -0.172 |

The timing work took the mean from -0.18 to +0.02 ns (7 seeds of 11
passing instead of 4); the memory system gave back 0.07 ns of it (mean
-0.06 ns, 3 of 11 passing: the spread is narrower, its centre just under
0). The project builds with seed 9 (+0.149 ns setup, +0.247 ns hold, HDMI
+0.478 ns). Check every clock in the timing summary: at seed 4 of the
2026-11-23 RTL the core clock passed but the framework's HDMI clock
missed by 0.018 ns.

Every change to the design moves the placement, so a single seed says
little: the worst path of one seed moves by about +-0.3 ns with any edit,
and compare a change over several seeds. A build is deterministic: a seed
gives the same result on any machine. Over these seeds the endpoints
within 0.5 ns of failing went from 23-375 (median 197) to 9-57 (median
16), within 1 ns from a median of about 1050 to about 460: what is left
near the edge is a few paths, a different one at each seed.

Register retiming (physical synthesis) is needed: without it the slack
falls by 1.1-1.6 ns (the multiply into W, the table unit, the blitter's
circle walk, the framework's scaler and Hq2x). It also moves logic across
the CPU's pipeline registers wherever one side looks longer: from D into E
(the immediate's way select) and from M into W (the end of the load data's
select), which lengthens the forwarding paths. So the RTL keeps every stage
short rather than relying on it. Quartus Lite has no LogicLock regions.

What keeps 100 MHz:

- No late signal drives a wide fanout. The I$ way prediction (a RAM output,
  which the fitter cannot duplicate) is kept in six copies, one per group
  of readers (fetch address, the immediate, the rs1 / rs2 read, `data`,
  the hit): one copy drove 168 loads, 2 ns of routing on the fetch loop.
  The I$ read enables come from registers: the load-use hazard is
  registered and the instruction waits in E (the same bubble), and a read
  on a miss is harmless (S_DONE re-reads d_pc); the I$ hit only enables
  q_addr and d_pc.
- The I$ RAMs are written 64 bits (two instructions) at a time and read
  32: the RAM selects the word by address bit 2, so the fetch loop, the
  decode and the register read have no word select after the RAM.
- The register file is four LUT RAMs (MLAB, one per read port: rs1 / rs2
  of each way): a read is a LUT's delay after the index from the I$ RAM,
  where registers needed a 32:1 mux whose selects drove 160 LUTs. Quartus
  infers an MLAB only with one asynchronous read port per array, and the
  indices are kept nodes (otherwise synthesis takes the I$ RAM's output
  register as the MLABs' address register, and the I$ RAMs cannot be
  inferred).
- The next fetch address: the early choices (reset, redirect, re-read,
  pc + 4) are made first, the predictions select last. `rd_pend` is one
  LUT after the branch compare (the terms from registers are combined
  first).
- E: the forwarded operands are copied (kept) for each kind of consumer:
  the D$ address add, the PICO-8 RAM address add, the shifter, the compare
  (`slt[u]`, min / max), the branch compare, the rest of the ALU. Shifts
  and compares enter the last select of the result; the order compares are
  a subtraction's carry chain (for `<` synthesis built a deeper LUT tree).
  A branch folds funct3[0] with the prediction and the static rule in D,
  so a mispredict is the selected compare ^ a register. CSR reads are
  decoded in D (which CSR, and whether the instruction ahead writes it).
  Zbb min / max take their compare from the ALU; the rotates enter the Zbb
  select last.
- M: the D$ store bypass is merged with the word select (one LUT after the
  RAM; the byte lanes are read raw, and the store's row compare is
  registered in its own cycle); the load alignment uses selects decoded in
  E; PICO-8 RAM and MMIO data have a select of their own in front of the
  last one, so the D$ data meets one select into w_val.
- The D$ has a tag compare for the CPU's request and one for the table
  unit's: the CPU's comes straight from its address register, not through
  a requester select. Its read address select is kept per way.
- The PICO-8 RAM has two copies with the same writes: the CPU reads its
  own (its own address add, no owner select), the blitter or the vblank
  copy the other (the blitter without the write bypass). Each copy has its
  own write select.
- The blitter: the pixel merges (tline, sspr) select the nibble one-hot;
  the map row's tile slots are written with enables decoded a stage
  earlier. Its cycles are unchanged. The DDR write data is a register (the
  D$ write-back buffer is a shift register).

The memory system of 2026-11-24 (D$ request with the miss and a
write-back buffer, radix-4 divider, the table unit's lookup cache,
performance counters) costs slack in the same places, not on paths of its
own: the forwarding loops above are pushed around by placement. What
helped: counter events from registers only (replay, the branch compare and
the I$ hit are late; another load on them showed), the counter CSRs
decoded as a flag and the CSR's low bits (a decode after the I$ RAM missed
by 0.17 ns), and the counters' totals in LUT RAM, added to by one 5-bit
count per cycle in turn (16 32-bit counters in registers and their 16:1
read mux into M cost the forwarding loop 0.15-0.4 ns). What did not:
going back to the radix-2 divider (every seed of 1-6 worse, by 0.5 ns on
average: the change moved the whole placement) and one PICO-8 RAM address
add per byte lane. A 128 KiB I$ (`SB = 11`: twice the RAM blocks on the
fetch address, 525 of 553 in all) failed the fetch loop on every seed of
1-6, by 0.17-0.63 ns.

Tried and dropped: retiming off (above); register copies of `rd_pend`
kept out of retiming, and the map row's tile table in LUT RAM (both worse
over the seeds); a result select for bit 0 of its own and a JALR target
add of its own (better on the seeds this RTL misses, worse on others).
What limits this RTL now is mostly the forwarding loop through the ALU
into bit 0 of the result, a different path at each seed within +-0.4 ns.

Faster clocks: at 110 MHz a few hundred endpoints of this RTL would fail,
about two thirds of them the I$ fetch loop (a RAM output, a LUT or two,
the address of about 125 RAM blocks); at 125 MHz thousands. The fetch
loop, the single-cycle forwarding loop through the ALU and the load data
loop (D$ RAM, W, forwarding, D$ address) would need more pipeline stages
(a registered I$ output, registered load data), at a cost in cycles per
instruction, and the framework's scaler input runs on the same clock.
