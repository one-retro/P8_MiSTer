# Timing

Timing closes at 100 MHz in Quartus 17.0.2 with seed 4 (+0.007 ns setup,
+0.246 ns hold, HDMI +0.358 ns), but not with every seed: of the ten
tried on this RTL, 4 and 64 (+0.003 ns) pass; the others missed by
0.16-0.78 ns on the CPU's paths below, not on the audio unit. Any change
to the design moves the placement, even the core's name in its OSD
string: seed 22, which passed before that change, missed by 0.16 ns
after it.
The audio unit keeps its state (oscillators, effects, READ's output) in
RAM rather than registers: as registers it took twice the logic and the
CPU's paths missed by more. One
cause of failing seeds was register retiming: when the blitter's `run`
waited on a 35-bit compare (the end of an oval), Quartus moved the register
back across it, and the compare then sat in front of `hold` (the CPU's
stall and the PICO-8 RAM port select), on the CPU's paths; no seed of ten
passed. The compare is now registered a cycle ahead. The margin moves with every
RTL change. Quartus Lite has no LogicLock regions (they need a subscription
license), and more fitter effort did not change the placement, so the
critical paths are kept short in the RTL instead. The tight paths are in
the CPU's D, E and M stages (the register read, forwarding into the ALU and
the address adds, the load data into W) and in the PICO-8 RAM read address.
What keeps 100 MHz: the register file is read for all four words of the
fetched line (2 ways x 2 words) and the word and way select last; the E
result mux has two inputs (ALU, pc + 4 from D): divides, CSR reads, MUL,
Zbb and p8.hfind results reach W from M through one kept node, so load data
meets a single 2:1 select into W; MMIO loads take a replay instead of the
register decode on the load path; the PICO-8 RAM read port goes to a load
in E without waiting for redirects; no
late signal (I$/D$ hit, replay) drives a RAM address or a register
enable; the stalls that do gate the I$ read enables come from registers
two LUTs deep (`dm_done` is an OR of registered pulses); the I$ stores
predecoded branch targets, computed from registered DDR data; logic that
waits on a cache way (register read, immediate, load alignment,
table-unit compares) runs for both ways and the way selects last; the
table unit reads the D$ without the RAMs' write bypass. JALR prediction
sits on the fetch loop: the target buffer's entry is an input of its own
to the fetch-address mux, so the predecoded branch target keeps a single
way select in front of it, and the prediction is checked in M from
registers (a check in E, on the forwarding paths, missed by 0.1 to 0.9 ns
over four seeds). A return-address stack adds a select to the target and
missed by 0.66 ns; it is in the RTL behind `RAS`, off. The branch
predictor's update registers only the outcome behind the forwarded
compare (the new counter is computed at the RAM's write port): with the
counter arithmetic in front of the register, the forwarding paths into it
failed by 0.5-0.8 ns after an unrelated change moved the placement.
