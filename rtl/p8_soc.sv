// P8 SoC: RV32IM CPU, caches on DDR3, PICO-8 RAM, MMIO, video,
// audio FIFO, the audio unit and the HPS loader. Memory map: docs/hardware.md.
//
//   0x0000_0000 - 0x01ff_ffff  DDR3 (cached), 32 MiB at DDR 0x3000_0000
//   0x1000_0000 - 0x1000_ffff  PICO-8 RAM (64 KiB block RAM)
//   0x2000_0000 - 0x2000_0fff  MMIO
//
// The save state port (SS_* registers) reaches MiSTer Main's save state
// area, DDR 0x3400_0000 (four 16 MiB slots), a 64-bit beat at a time.
//
// The CPU is held in reset until a program (the BIOS, or a p8c image:
// runtime + compiled cart) has been downloaded to address 0, and
// while a download is in progress; every download restarts it, and so
// does `reset` (OSD Reset, USER button).

module p8_soc #(
	parameter CLK_HZ = 100000000
) (
	input             clk,
	input             reset,

	// HPS download
	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input      [26:0] ioctl_addr,
	input       [7:0] ioctl_dout,
	output            ioctl_wait,

	input      [15:0] buttons,      // {player 1, player 0}, PICO-8 bit order
	input             pause,        // BUTTONS bit 31: the firmware waits
	input             menu,         // a pulse: BUTTONS bit 30 until BUTTONS is written (OSD "Cart menu")
	input             ss_save,      // pulses: BUTTONS bit 19 / 18 until BUTTONS is written
	input             ss_load,      // (OSD "Save state" / "Load state")
	input       [1:0] ss_slot,      // BUTTONS bits 17:16 (OSD option)
	input       [1:0] mouse_speed,  // BUTTONS bits 29:28 (OSD option)
	// mouse and keyboard as hps_io delivers them (MOUSE and KEY registers)
	input      [24:0] ps2_mouse,
	input       [7:0] mouse_wheel,  // ps2_mouse_ext[7:0]
	input      [10:0] ps2_key,
	input      [31:0] frames,       // FRAMES register (0 = run forever)
	input             show_fps,     // fps overlay

	// save file (hps_io virtual disk 0, 512-byte blocks; SAVE_* registers)
	input             img_mounted,
	input             img_readonly,
	input      [31:0] img_size,
	output reg [31:0] sd_lba = 32'd0,
	output reg        sd_rd = 1'b0,
	output reg        sd_wr = 1'b0,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din,
	input             sd_buff_wr,

	// DDR3
	output     [28:0] DDRAM_ADDR,
	output      [7:0] DDRAM_BURSTCNT,
	output            DDRAM_RD,
	output            DDRAM_WE,
	output     [63:0] DDRAM_DIN,
	output      [7:0] DDRAM_BE,
	input             DDRAM_BUSY,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY,

	// video
	output            ce_pix,
	output      [7:0] r,
	output      [7:0] g,
	output      [7:0] b,
	output            hs,
	output            vs,
	output            hblank,
	output            vblank,

	output     [15:0] audio,

	// status / debug
	output            running,
	output reg        con_wr,
	output reg  [7:0] con_data,
	output reg        exit_wr,
	output reg [31:0] exit_code,
	output reg        frame_wr
);

// ---------------------------------------------------------------- reset control

wire        loaded, ld_busy, arb_idle;
wire [31:0] cart_size, buf_size;
wire        ld_bios, ld_bios_wr;
wire [15:0] cart_seq;
wire  [5:0] cart_slot;
// The CPU and caches stop only between DDR transactions (`hold` lets the
// current one finish): the DDR controller does not see a reset and would
// still deliver the rest of a read burst, or wait for the rest of a write
// burst. A reset pulse is remembered until the CPU has stopped.
reg         cpu_reset = 1'b1;
reg         rst_req = 1'b1;
wire        run_req = loaded && !ld_busy && !rst_req;

always @(posedge clk) begin
	if (reset)                      rst_req <= 1'b1;
	else if (cpu_reset)             rst_req <= 1'b0;
	if (run_req)                    cpu_reset <= 1'b0;
	else if (arb_idle)              cpu_reset <= 1'b1;
end

assign running = !cpu_reset;

// ---------------------------------------------------------------- CPU

wire [31:0] if_addr, if_data, if_data0, if_data1;
wire  [3:0] if_way;
wire        if_valid, if_adv, if_ren, if_reread;
wire [28:0] if_pre0, if_pre1;
wire [13:0] de_addr;
wire [31:0] dm_addr, dm_wdata;
wire        dm_rd, dm_wr, dm_fencei, dm_hfind, dm_hnum, dm_zline;
wire [31:0] dm_hkey, p8heap;
wire  [3:0] dm_be;
wire [31:0] dm_rdata0, dm_rdata1;
wire        dm_rsel1;
reg         dm_ready, dm_done;

// PICO-8 RAM read ports (see below): the CPU's copy, and the blitter's
// (while it owns the RAM) or the vblank copy's
wire        blt_own;
wire [13:0] cp_addr, blt_raddr, cpu_raddr;

// table unit (p8.hfind), see below
wire        tu_active, tu_rd, tu_done;
wire [31:0] tu_addr, tu_result;
// performance events (rv32_cpu's counters 0-7)
wire  [2:0] ic_pm;
wire  [3:0] dc_pm;
wire        tu_pm;

rv32_cpu cpu (
	.clk(clk), .reset(cpu_reset),
	.if_addr(if_addr), .if_adv(if_adv), .if_ren(if_ren), .if_reread(if_reread), .if_data(if_data), .if_valid(if_valid),
	.if_data0(if_data0), .if_data1(if_data1), .if_pre0(if_pre0), .if_pre1(if_pre1), .if_way(if_way),
	.de_addr(de_addr), .ra_addr(cpu_raddr),
	.dm_addr(dm_addr), .dm_rd(dm_rd), .dm_wr(dm_wr), .dm_be(dm_be),
	.dm_wdata(dm_wdata), .dm_fencei(dm_fencei),
	.dm_hfind(dm_hfind), .dm_hnum(dm_hnum), .dm_zline(dm_zline), .dm_hkey(dm_hkey), .p8heap(p8heap),
	.dm_rdata0(dm_rdata0), .dm_rdata1(dm_rdata1), .dm_rsel1(dm_rsel1),
	.dm_pdata(p8_qc), .dm_mdata(mmio_q_r), .dm_p8(m_p8), .dm_mmio(!m_ddr && !m_p8),
	.dm_ready(dm_ready), .dm_done(dm_done),
	.dm_hres(tu_result),
	.pm_ev({tu_pm, dc_pm, ic_pm})
);

wire [3:0] m_region = dm_addr[31:28];
wire       m_ddr  = m_region == 4'h0;
wire       m_p8   = m_region == 4'h1;
wire       m_mmio = m_region == 4'h2;

// ---------------------------------------------------------------- caches + DDR

wire        ic_req, ic_gnt, ic_rd, ic_inval_done;
wire [21:0] ic_maddr;
wire        dc_req, dc_gnt, dc_rd, dc_we, dc_ready, dc_flush_done, dc_zl_done;
wire [21:0] dc_maddr;
wire [63:0] dc_mwdata;
wire [31:0] dc_rdata0, dc_rdata1;
wire        dc_rhit1, dc_t_rhit1, dc_t_ready;
wire [63:0] dc_rdata64_0, dc_rdata64_1;
wire        ld_req, ld_gnt, ld_we;
wire [21:0] ld_addr;
wire [63:0] ld_wdata;
wire  [7:0] ld_be;
wire        mem_busy, mem_rvalid;
wire [63:0] mem_rdata;
reg         ss_req = 1'b0, ss_rd = 1'b0, ss_we = 1'b0;
reg  [22:0] ss_addr = 23'd0;
reg  [63:0] ss_wdata;
wire        ss_gnt;
reg   [1:0] fence_st;

p8_icache #(.WAYS(4)) icache (
	.clk(clk), .reset(cpu_reset),
	.addr(if_addr), .adv(if_adv), .ren(if_ren), .reread(if_reread), .data(if_data), .valid(if_valid),
	.data0(if_data0), .data1(if_data1), .pre0(if_pre0), .pre1(if_pre1), .way_q(if_way),
	.inval(fence_st == 2'd2), .inval_done(ic_inval_done), .pm(ic_pm),
	.mem_req(ic_req), .mem_gnt(ic_gnt), .mem_rd(ic_rd), .mem_addr(ic_maddr),
	.mem_busy(mem_busy), .mem_rdata(mem_rdata), .mem_rvalid(mem_rvalid)
);

// (while the table unit runs, the CPU waits in M with p8.hfind: no load,
// store or p8.zline of its own)
p8_dcache dcache (
	.clk(clk), .reset(cpu_reset),
	.e_addr(de_addr), .m_addr(dm_addr),
	// requesters that wait in M with a fixed address (registered signals)
	.m_hold(tu_active || (dm_zline && m_ddr && !dc_zl_done)),
	.m_rd(dm_rd && m_ddr), .m_wr(dm_wr && m_ddr), .m_zl(dm_zline && m_ddr),
	.m_be(dm_be), .m_wdata(dm_wdata),
	.t_act(tu_active), .t_addr(tu_addr), .t_rd(tu_rd),
	.rdata0(dc_rdata0), .rdata1(dc_rdata1), .rhit1(dc_rhit1), .rdata64_0(dc_rdata64_0), .rdata64_1(dc_rdata64_1),
	.t_rhit1(dc_t_rhit1), .ready(dc_ready), .t_ready(dc_t_ready), .zl_done(dc_zl_done),
	.flush(fence_st == 2'd1), .flush_done(dc_flush_done), .pm(dc_pm),
	.mem_req(dc_req), .mem_gnt(dc_gnt), .mem_rd(dc_rd), .mem_we(dc_we),
	.mem_addr(dc_maddr), .mem_wdata(dc_mwdata),
	.mem_busy(mem_busy), .mem_rdata(mem_rdata), .mem_rvalid(mem_rvalid)
);

p8_loader loader (
	.clk(clk),
	.ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout), .ioctl_wait(ioctl_wait),
	.cpu_stopped(cpu_reset), .busy(ld_busy), .loaded(loaded), .cart_size(cart_size),
	.bios_wr(ld_bios_wr), .bios_data(dm_wdata[0]), .bios(ld_bios),
	.cart_seq(cart_seq), .cart_slot(cart_slot), .buf_size(buf_size),
	.mem_req(ld_req), .mem_gnt(ld_gnt), .mem_we(ld_we), .mem_addr(ld_addr),
	.mem_wdata(ld_wdata), .mem_be(ld_be), .mem_busy(mem_busy)
);

p8_ddr_arb arb (
	.clk(clk), .hold(!run_req), .idle_caches(arb_idle),
	.c0_req(dc_req), .c0_rd(dc_rd), .c0_we(dc_we), .c0_addr(dc_maddr), .c0_wdata(dc_mwdata), .c0_gnt(dc_gnt),
	.c1_req(ic_req), .c1_rd(ic_rd), .c1_addr(ic_maddr), .c1_gnt(ic_gnt),
	.c2_req(ld_req), .c2_we(ld_we), .c2_addr(ld_addr), .c2_wdata(ld_wdata), .c2_be(ld_be), .c2_gnt(ld_gnt),
	.c3_req(ss_req), .c3_rd(ss_rd), .c3_we(ss_we), .c3_addr(ss_addr), .c3_wdata(ss_wdata), .c3_gnt(ss_gnt),
	.busy(mem_busy), .rdata(mem_rdata), .rvalid(mem_rvalid),
	.DDRAM_ADDR(DDRAM_ADDR), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_RD(DDRAM_RD), .DDRAM_WE(DDRAM_WE),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_BUSY(DDRAM_BUSY),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY)
);

// p8.hfind
p8_tunit tunit (
	.clk(clk), .reset(cpu_reset),
	.start(dm_hfind), .num(dm_hnum), .base(p8heap), .tab(dm_addr), .key(dm_hkey),
	.done(tu_done), .result(tu_result), .pm_hit(tu_pm),
	.active(tu_active), .addr(tu_addr), .rd(tu_rd), .ready(dc_t_ready),
	.rdata0(dc_rdata64_0), .rdata1(dc_rdata64_1), .rsel1(dc_t_rhit1)
);

// FENCE.I: write back the D$, then invalidate the I$
always @(posedge clk) begin
	if (cpu_reset) fence_st <= 2'd0;
	else case (fence_st)
		2'd0: if (dm_fencei) fence_st <= 2'd1;
		2'd1: if (dc_flush_done) fence_st <= 2'd2;
		2'd2: if (ic_inval_done) fence_st <= 2'd3;
		default: fence_st <= 2'd0;
	endcase
end

// ---------------------------------------------------------------- PICO-8 RAM

wire        cp_req;
wire [31:0] p8_q, p8_qr, p8_qc;
// Two copies with the same writes (the blitter's, or the CPU's stores):
// the CPU reads its own every cycle with E's address (cpu_raddr, its own
// add: the late E-stage address has no mux after it, and the copy's 64 RAM
// blocks serve no other reader); M decides from the registered address
// whether p8_qc is used. The blitter (while it owns the RAM, blt_own, a
// register) reads the other, else the vblank copy. While the blitter runs
// the CPU runs on: its loads and stores to PICO-8 RAM are replayed
// (dm_ready) from the cycle after its CTRL store until the blitter is done
// (blt_hold: own lags the start by two cycles), so software needs no
// fences. A load that read its copy in the cycle of a blitter write saw it
// (the RAM's write bypass) or is replayed (own is high in the next cycle).
// The blitter takes its data without the bypass (p8_qr): the CPU does not
// write while it owns the RAM, and it never reads a word it writes in the
// same cycle.
wire        blt_we;
wire [13:0] blt_waddr;
wire [31:0] blt_wdata;
wire        p8_we     = dm_wr && m_p8 && !blt_hold;
wire [13:0] p8_raddr  = blt_own ? blt_raddr : cp_addr;

genvar gi;
// (the write port's select is kept once per copy: one would drive the
// address of 128 RAM blocks)
generate for (gi = 0; gi < 4; gi = gi + 1) begin : p8lane
	(* keep = 1 *) wire        we_b = blt_we || (p8_we && dm_be[gi]);
	(* keep = 1 *) wire [13:0] wa_b = blt_we ? blt_waddr : dm_addr[15:2];
	(* keep = 1 *) wire        we_c = blt_we || (p8_we && dm_be[gi]);
	(* keep = 1 *) wire [13:0] wa_c = blt_we ? blt_waddr : dm_addr[15:2];
	wire  [7:0] wd = blt_we ? blt_wdata[gi*8 +: 8] : dm_wdata[gi*8 +: 8];
	p8_bram #(.AW(14), .DW(8)) ram (
		.clk(clk), .re(1'b1), .raddr(p8_raddr), .rdata(p8_q[gi*8 +: 8]), .rdata_raw(p8_qr[gi*8 +: 8]),
		.we(we_b), .waddr(wa_b), .wdata(wd)
	);
	p8_bram #(.AW(14), .DW(8)) cram (
		.clk(clk), .re(1'b1), .raddr(cpu_raddr), .rdata(p8_qc[gi*8 +: 8]), .rdata_raw(),
		.we(we_c), .waddr(wa_c), .wdata(wd)
	);
end endgenerate

// ---------------------------------------------------------------- MMIO

reg  [63:0] cycles;
wire [31:0] vcount;
wire [11:0] au_level;
wire        mmio_wr = dm_wr && m_mmio;
wire  [9:0] mreg    = dm_addr[11:2];
assign ld_bios_wr = mmio_wr && mreg == 10'h018;

always @(posedge clk) begin
	cycles   <= cpu_reset ? 64'd0 : cycles + 64'd1;
	con_wr   <= mmio_wr && mreg == 10'h000;
	exit_wr  <= mmio_wr && mreg == 10'h001;
	frame_wr <= mmio_wr && mreg == 10'h003;
	con_data <= dm_wdata[7:0];
	if (mmio_wr && mreg == 10'h001) exit_code <= dm_wdata;
end

wire [31:0] blt_id;
wire        blt_busy, blt_hold;

p8_blit blit (
	.clk(clk), .reset(cpu_reset),
	.mmio_wr_in(mmio_wr), .mreg_in(mreg), .wdata_in(dm_wdata), .id(blt_id), .status(blt_busy), .hold(blt_hold),
	.own(blt_own), .raddr(blt_raddr), .rdata(p8_qr),
	.we(blt_we), .waddr(blt_waddr), .wdat(blt_wdata)
);

// Mouse: packet deltas accumulated into wrapping counters, which the
// firmware samples once per frame (MOUSE).
reg [11:0] mouse_x = 12'd0, mouse_y = 12'd0;
reg  [4:0] mouse_w = 5'd0;
reg        mouse_tog = 1'b0;
always @(posedge clk) begin
	mouse_tog <= ps2_mouse[24];
	if (mouse_tog != ps2_mouse[24]) begin
		mouse_x <= mouse_x + {{4{ps2_mouse[4]}}, ps2_mouse[15:8]};
		mouse_y <= mouse_y + {{4{ps2_mouse[5]}}, ps2_mouse[23:16]};
		mouse_w <= mouse_w + mouse_wheel[4:0];
	end
end

// Keyboard: a FIFO of hps_io key events {pressed, extended, code}. KEY
// reads the oldest ({valid, 21'd0, event}); writing KEY drops it.
reg  [9:0] kq [0:15];
reg  [4:0] kq_wp = 5'd0, kq_rp = 5'd0;
reg        key_tog = 1'b0;
wire       kq_empty = kq_wp == kq_rp;
always @(posedge clk) begin
	key_tog <= ps2_key[10];
	if (key_tog != ps2_key[10] && kq_wp - kq_rp != 5'd16) begin
		kq[kq_wp[3:0]] <= ps2_key[9:0];
		kq_wp <= kq_wp + 5'd1;
	end
	if (mmio_wr && mreg == 10'h00e && !kq_empty) kq_rp <= kq_rp + 5'd1;
	if (cpu_reset) kq_rp <= kq_wp;
end

// Save file: one 512-byte block at a time between the HPS and two buffers
// (HPS -> CPU, CPU -> HPS) of byte lanes. The CPU sets the word pointer
// (SAVE_ADDR), then reads SAVE_DATA twice (the RAM output follows the
// pointer a cycle later) or writes words (the pointer advances).
// `sv_present`: a save file was mounted after the last download (main
// mounts it, or will create it on the first write, for each loaded cart).
reg  [6:0] sv_ptr = 7'd0;
reg        sv_present = 1'b0, sv_ro = 1'b0, sv_busy = 1'b0, sv_acked = 1'b0;
reg        menu_req = 1'b0;
reg  [1:0] ss_ask = 2'd0;
reg [31:0] sv_size = 32'd0;
reg        sv_dl = 1'b0;
reg  [1:0] sv_lane;
wire [31:0] sv_rx;
wire  [7:0] sv_tx [0:3];
always @(posedge clk) begin
	sv_dl <= ioctl_download;
	if (ioctl_download && !sv_dl) sv_present <= 1'b0;
	if (img_mounted) begin
		sv_present <= 1'b1;
		sv_ro      <= img_readonly;
		sv_size    <= img_size;
	end
	if (mmio_wr && mreg == 10'h011) sd_lba <= dm_wdata;
	// the pause menu asked for from the OSD, until the firmware takes it
	if (menu)                                     menu_req <= 1'b1;
	else if (reset || (mmio_wr && mreg == 10'h007)) menu_req <= 1'b0;
	// a save state to save / load, until the firmware takes it
	if (reset || (mmio_wr && mreg == 10'h007))      ss_ask <= 2'd0;
	else if (ss_save)                             ss_ask <= 2'b10;
	else if (ss_load)                             ss_ask <= 2'b01;
	if (mmio_wr && mreg == 10'h010 && dm_wdata[1:0] != 2'd0 && !sv_busy) begin
		sd_rd   <= dm_wdata[0];
		sd_wr   <= dm_wdata[1];
		sv_busy <= 1'b1;
	end
	sv_acked <= sd_ack;
	if (sd_ack) begin sd_rd <= 1'b0; sd_wr <= 1'b0; end
	if (sv_acked && !sd_ack) sv_busy <= 1'b0;
	if (mmio_wr && mreg == 10'h013) sv_ptr <= dm_wdata[6:0];
	if (mmio_wr && mreg == 10'h014) sv_ptr <= sv_ptr + 7'd1;
	sv_lane <= sd_buff_addr[1:0];
end
assign sd_buff_din = sv_tx[sv_lane];

genvar sv_i;
generate for (sv_i = 0; sv_i < 4; sv_i = sv_i + 1) begin : sv_lanes
	p8_bram #(.AW(7), .DW(8), .BYPASS(0)) rx (
		.clk(clk), .re(1'b1), .raddr(sv_ptr), .rdata(sv_rx[8*sv_i +: 8]), .rdata_raw(),
		.we(sd_buff_wr && sd_ack && sd_buff_addr[1:0] == sv_i), .waddr(sd_buff_addr[8:2]), .wdata(sd_buff_dout));
	p8_bram #(.AW(7), .DW(8), .BYPASS(0)) tx (
		.clk(clk), .re(1'b1), .raddr(sd_buff_addr[8:2]), .rdata(sv_tx[sv_i]), .rdata_raw(),
		.we(mmio_wr && mreg == 10'h014), .waddr(sv_ptr), .wdata(dm_wdata[8*sv_i +: 8]));
end endgenerate

// Save state port: SS_ADDR (0x068) the beat address in the save state
// area; SS_LO (0x06c) the low word of the beat to write; writing SS_HI
// (0x070) writes the beat {SS_HI, SS_LO}, writing SS_READ (0x074) reads
// one into a register; either takes the port (ignored while it is busy)
// and goes on to the next beat. Loads of SS_LO / SS_HI (the beat read
// last) wait until the port is free. The transaction always completes
// (a CPU reset does not stop it: the DDR controller does not see one).
reg [63:0] ss_q;
reg [31:0] ss_lo;
reg        ss_op_wr = 1'b0, ss_sent = 1'b0;
always @(posedge clk) begin
	if (mmio_wr && mreg == 10'h01a && !ss_req) ss_addr <= dm_wdata[22:0];
	if (mmio_wr && mreg == 10'h01b) ss_lo <= dm_wdata;
	if (mmio_wr && mreg == 10'h01c && !ss_req) begin
		ss_wdata <= {dm_wdata, ss_lo};
		ss_req   <= 1'b1;
		ss_op_wr <= 1'b1;
	end
	if (mmio_wr && mreg == 10'h01d && !ss_req) begin
		ss_req   <= 1'b1;
		ss_op_wr <= 1'b0;
	end
	if (ss_req && ss_gnt) begin
		if (ss_op_wr) begin
			ss_we <= 1'b1;
			if (ss_we && !mem_busy) begin
				ss_we   <= 1'b0;
				ss_req  <= 1'b0;
				ss_addr <= ss_addr + 23'd1;
			end
		end else if (!ss_sent) begin
			ss_rd <= 1'b1;
			if (ss_rd && !mem_busy) begin
				ss_rd   <= 1'b0;
				ss_sent <= 1'b1;
			end
		end else if (mem_rvalid) begin
			ss_q    <= mem_rdata;
			ss_sent <= 1'b0;
			ss_req  <= 1'b0;
			ss_addr <= ss_addr + 23'd1;
		end
	end
end

reg [31:0] mmio_q;
always @* begin
	case (mreg)
		10'h004: mmio_q = cycles[31:0];
		10'h005: mmio_q = cycles[63:32];
		10'h006: mmio_q = cart_size;
		10'h007: mmio_q = {pause, menu_req, mouse_speed, 8'd0, ss_ask, ss_slot, buttons};
		10'h008: mmio_q = frames;
		10'h00a: mmio_q = vcount;
		10'h00b: mmio_q = {20'd0, au_level};
		10'h00d: mmio_q = {mouse_w, ps2_mouse[2:0], mouse_y, mouse_x};
		10'h00e: mmio_q = {!kq_empty, 21'd0, kq[kq_rp[3:0]]};
		10'h010: mmio_q = {sv_present, sv_ro, 29'd0, sv_busy};
		10'h012: mmio_q = sv_size;
		10'h014: mmio_q = sv_rx;
		10'h018: mmio_q = {cart_seq, 7'd0, ld_bios, 2'd0, cart_slot};
		10'h019: mmio_q = buf_size;
		10'h01b: mmio_q = ss_q[31:0];
		10'h01c: mmio_q = ss_q[63:32];
		10'h040: mmio_q = blt_id;
		10'h041: mmio_q = {31'd0, blt_busy};
		10'h080: mmio_q = 32'h41505501;            // audio unit: "APU", version 1
		10'h081: mmio_q = {30'd0, apu_status};
		10'h087: mmio_q = {16'd0, apu_acc};
		default: mmio_q = 32'd0;
	endcase
end

// dm_done: hfind / zline / fence.i, an OR of registered pulses that each
// come only for their own operation (it stalls the whole pipeline, so no
// mux by the operation). A p8.zline outside DDR does nothing and takes a
// cycle. dm_ready: loads and stores (late: D$ hit).
reg zl_skip;
always @(posedge clk) zl_skip <= !cpu_reset && dm_zline && !m_ddr && !zl_skip;
always @* dm_done = tu_done || dc_zl_done || zl_skip || fence_st == 2'd3;

// MMIO loads: the register decode (mreg, a 10-bit compare) and its 32-bit
// mux are kept off the load path into the CPU's W value. The first time a
// load is in M its value is registered and it is replayed; it is the next
// MMIO load to reach M (in order: the replay restarts at it), which takes
// the registered value. Its value is from a few cycles before it completes
// (the blitter's busy bit, counters: harmless). A load of SS_LO / SS_HI is
// replayed until the save state port is free.
reg [31:0] mmio_q_r;
reg        mmio_pend;
wire       mmio_wait = ss_req && (mreg == 10'h01b || mreg == 10'h01c);
always @(posedge clk) begin
	if (cpu_reset) mmio_pend <= 1'b0;
	else if (dm_rd && m_mmio) begin
		if (!mmio_pend) mmio_q_r <= mmio_q;
		mmio_pend <= !mmio_pend && !mmio_wait;
	end
end

// Load data: the word from each D$ way and the D$ hit way, which is late;
// outside DDR the PICO-8 RAM's word or the MMIO value (dm_pdata,
// dm_mdata, with dm_p8 / dm_mmio). The CPU aligns each and selects last,
// so the D$ data has no source select in front of it. (p8.hfind's result
// goes to the CPU on its own: dm_hres.)
assign dm_rdata0 = dc_rdata0;
assign dm_rdata1 = dc_rdata1;
assign dm_rsel1  = dc_rhit1;
always @* begin
	if (m_ddr)
		dm_ready = dc_ready;
	else if (m_p8)
		// replayed while the blitter runs (from its start)
		dm_ready = !blt_hold;
	else
		// MMIO: a load is replayed once and takes the registered value
		dm_ready = !dm_rd || mmio_pend;
end


// ---------------------------------------------------------------- video, audio

p8_video #(.CLK_HZ(CLK_HZ)) video (
	.clk(clk), .reset(reset),
	.cp_addr(cp_addr), .cp_req(cp_req), .cp_gnt(cp_req && !blt_own), .cp_rdata(p8_q),
	.present(mmio_wr && mreg == 10'h003), .show_fps(show_fps), .vcount(vcount),
	.spin_wr(mmio_wr && mreg == 10'h00c), .spin_data(dm_wdata[15:0]),
	.ce_pix(ce_pix), .r(r), .g(g), .b(b), .hs(hs), .vs(vs), .hblank(hblank), .vblank(vblank)
);

// the audio unit pushes its samples into the FIFO when the CPU does not
// write it in the same cycle
wire        au_cpu_wr = mmio_wr && mreg == 10'h00b;
wire  [1:0] apu_status;
wire [15:0] apu_acc, apu_data;
wire        apu_push;

p8_apu apu (
	.clk(clk), .reset(cpu_reset),
	.wr(mmio_wr && mreg[9:3] == 7'h10), .wreg(mreg[2:0]), .wdata(dm_wdata),
	.status(apu_status), .acc_q(apu_acc),
	.push(apu_push), .push_data(apu_data), .push_ok(!au_cpu_wr)
);

p8_audio #(.CLK_HZ(CLK_HZ)) snd (
	.clk(clk), .reset(cpu_reset),
	.wr(au_cpu_wr || apu_push), .wdata(au_cpu_wr ? dm_wdata[15:0] : apu_data),
	.level(au_level), .out(audio)
);

endmodule
