// Blitter: sprite blits, rectangle fills and textured lines on the PICO-8
// RAM, one 32-bit word (8 pixels) per RAM access. Registers: docs/hardware.md.
// The runtime's bit-exact software model of the blitter is kept in sync with
// this file.
//
// The CPU writes the registers and starts an operation with a CTRL write;
// it polls CTRL before writing the next one. While busy, the blitter owns
// the read and write ports of the PICO-8 RAM (the SoC holds the CPU's
// accesses to it; the vblank copy waits).
//
// Pipeline: each cycle the sequencer issues one read (registered address):
//   blit, per row: prime source word, then (source word, destination word)
//                  per destination word: 1 + 2 * words cycles
//   fill / glyph, per row: one destination word per cycle
//   sspr, per row: one cycle, the sheet words into the row buffer (a
//                  16 x 32 RAM; one cycle each + 1, skipped when it holds
//                  the row), then one pixel per cycle (column DDA -> row
//                  buffer -> write table, merged into the destination
//                  word, which is read at its first pixel and written
//                  after its last)
//   circle, per step of the midpoint walk: one cycle (the step's up to
//                  four spans, clipped), then per visible span one cycle
//                  and one destination word per cycle (as a fill)
//   oval, three cycles (the products: walk start values and stop limits),
//                  then per walk one cycle and per step one cycle per
//                  move of the rounding walk and one (the point), three
//                  (its rows: sort, ranges, clip), per visible row one
//                  cycle and one destination word per cycle, and one (the
//                  next step); three for the column walk's last rows
//   tline, per pixel: when the map tile changes, find it (1), read its
//                  map byte (1), wait (2) [with layers: read the flags (1),
//                  wait (2)]; at the first pixel of a destination word,
//                  read the word (1); read the sheet word (1). The pixel
//                  merges into the word being assembled, which is written
//                  after its last pixel.
// The data arrives in the next cycle (d1): a source word shifts into the
// 2-word window; for a destination word the 8 (keep, val) nibbles are
// computed from the window and the write table (or the fill ink) and
// registered with the old word (d2), which is merged and written in the
// following cycle. A destination word is read once and written two cycles
// later; nothing reads it again in the same operation (rows are distinct
// and software never lets the sprite sheet overlap the screen).
//
// busy (CTRL bit 0) is set from the cycle after the CTRL write until the
// cycle after the last write: rows * (1 + 2 * words) + 5 cycles for a
// blit, rows * words + 5 for a fill or glyph, see blitter::cycles for
// sspr.

module p8_blit (
	input             clk,
	input             reset,

	// MMIO: word register index (dm_addr[11:2]) and data of a write
	input             mmio_wr_in,
	input       [9:0] mreg_in,
	input      [31:0] wdata_in,
	output     [31:0] id,
	output            status,      // CTRL read: busy
	output            hold,        // running or about to (registers only): the SoC holds the CPU's RAM accesses

	// PICO-8 RAM ports (owned while `own`)
	output reg        own,
	output reg [13:0] raddr,
	input      [31:0] rdata,       // data of the read issued last cycle
	output reg        we,
	output reg [13:0] waddr,
	output     [31:0] wdat
);

localparam OP_BLIT = 4'd1, OP_FILL = 4'd2, OP_GLYPH = 4'd3, OP_SSPR = 4'd4, OP_MAP = 4'd5, OP_TLINE = 4'd6, OP_CIRC = 4'd7, OP_OVAL = 4'd8;

assign id = {24'hb17e01, 8'hff};   // blit, fill, glyph, sspr, map, tline, circle, oval

// ---------------------------------------------------------------- registers

reg  [17:0] r_dst;
reg  [31:0] r_size;
reg  [23:0] r_srow;
reg  [18:0] r_scol;
reg [127:0] r_pal, r_fkeep, r_fval, r_gmask;
reg  [31:0] r_ss [0:5];       // sspr: SXQ, SXS, SYQ, SYS, SDIV, SLW
reg  [28:0] r_map0;           // map row: pointer, high, tiles - 1
reg   [8:0] r_map1;           // map row: layers, draw tile 0
reg  [31:0] r_tl [0:8];       // tline: TL0-8
reg  [31:0] r_cr [0:2];       // circle: CR0-2
reg  [31:0] r_ov [0:2];       // oval: OV0-2

// MMIO writes are registered first: the CPU's M-stage registers only see
// one load each (start and register writes happen a cycle later; status
// covers that cycle)
reg        mmio_wr, ctrl_wr;
reg  [9:0] mreg;
reg [31:0] wdata;
always @(posedge clk) begin
	mmio_wr <= !reset && mmio_wr_in && mreg_in[9:6] == 4'd1;
	ctrl_wr <= !reset && mmio_wr_in && mreg_in == 10'h041;
	mreg    <= mreg_in;
	wdata   <= wdata_in;
end

wire       start = mmio_wr && mreg == 10'h041 && wdata[3:0] != 4'd0 && wdata[3:0] <= OP_OVAL;

integer ri;

always @(posedge clk) begin
	if (reset) begin
		r_dst   <= 18'd0;
		r_size  <= 32'd0;
		r_srow  <= 24'd0;
		r_scol  <= 19'd0;
		r_pal   <= 128'd0;
		r_fkeep <= 128'd0;
		r_fval  <= 128'd0;
		r_gmask <= 128'd0;
		for (ri = 0; ri < 6; ri = ri + 1) r_ss[ri] <= 32'd0;
		r_map0  <= 29'd0;
		r_map1  <= 9'd0;
		for (ri = 0; ri < 9; ri = ri + 1) r_tl[ri] <= 32'd0;
		for (ri = 0; ri < 3; ri = ri + 1) r_cr[ri] <= 32'd0;
		for (ri = 0; ri < 3; ri = ri + 1) r_ov[ri] <= 32'd0;
	end else if (mmio_wr) begin
		if (mreg >= 10'h060 && mreg <= 10'h068) r_tl[mreg - 10'h060] <= wdata;
		if (mreg >= 10'h069 && mreg <= 10'h06b) r_cr[mreg - 10'h069] <= wdata;
		if (mreg >= 10'h06c && mreg <= 10'h06e) r_ov[mreg - 10'h06c] <= wdata;
		case (mreg)
			10'h042: r_dst  <= wdata[17:0];
			10'h043: r_size <= wdata;
			10'h044: r_srow <= wdata[23:0];
			10'h045: r_scol <= wdata[18:0];
			10'h048: r_pal[31:0]     <= wdata;
			10'h049: r_pal[63:32]    <= wdata;
			10'h04a: r_pal[95:64]    <= wdata;
			10'h04b: r_pal[127:96]   <= wdata;
			10'h04c: r_fkeep[31:0]   <= wdata;
			10'h04d: r_fkeep[63:32]  <= wdata;
			10'h04e: r_fkeep[95:64]  <= wdata;
			10'h04f: r_fkeep[127:96] <= wdata;
			10'h050: r_fval[31:0]    <= wdata;
			10'h051: r_fval[63:32]   <= wdata;
			10'h052: r_fval[95:64]   <= wdata;
			10'h053: r_fval[127:96]  <= wdata;
			10'h054: r_gmask[31:0]   <= wdata;
			10'h055: r_gmask[63:32]  <= wdata;
			10'h056: r_gmask[95:64]  <= wdata;
			10'h057: r_gmask[127:96] <= wdata;
			10'h058: r_ss[0] <= wdata;
			10'h059: r_ss[1] <= wdata;
			10'h05a: r_ss[2] <= wdata;
			10'h05b: r_ss[3] <= wdata;
			10'h05c: r_ss[4] <= wdata;
			10'h05d: r_ss[5] <= wdata;
			10'h05e: r_map0  <= wdata[28:0];
			10'h05f: r_map1  <= wdata[8:0];
			default: ;
		endcase
	end
end

wire  [3:0] nk_m1   = r_size[3:0];
wire  [6:0] rows_m1 = r_size[14:8];
wire  [7:0] lmask   = r_size[23:16];
wire  [7:0] rmask   = r_size[31:24];
wire        rstep_n = r_srow[12];
wire  [7:0] gbpage  = r_srow[23:16];
wire        flip    = r_scol[12];
wire  [2:0] sh      = r_scol[18:16];

// ---------------------------------------------------------------- sequencer

localparam S_PRIME = 4'd0, S_SRC = 4'd1, S_DST = 4'd2, S_ROW = 4'd3, S_LOAD = 4'd4, S_BUB = 4'd5, S_PIX = 4'd6,
           S_TILE = 4'd7, S_FLAG = 4'd8, S_WAIT = 4'd9, S_MASK = 4'd10, S_MROW = 4'd11;
// kind of an issued read (tline: K_TB map/flag byte, K_TD destination
// word, K_TS sheet word of a pixel)
localparam K_SRC = 3'd0, K_DST = 3'd1, K_TILE = 3'd2, K_FLAG = 3'd3, K_NONE = 3'd4, K_TB = 3'd5, K_TD = 3'd6, K_TS = 3'd7;

reg        run;
reg  [3:0] op;
reg  [3:0] sub;
reg  [6:0] rows_left;
reg  [3:0] k;
reg [11:0] sw, srow;
reg [13:0] drow, daddr;
reg  [1:0] phase;

// issued request (drives the RAM read address)
reg        i_v, i_ok, i_first, i_second, i_last;
reg  [2:0] i_kind;
reg  [1:0] i_phase, i_bsel;
reg  [7:0] i_lm, i_rm;        // edge masks of a destination word (first, last)
reg  [2:0] i_row;
reg  [4:0] i_slot;

// map row: tile numbers and whether each is drawn, map pointer, tile
// counter, bubble counter, destination words showing a drawn tile
wire        mapop   = op == OP_MAP;
// (tb in registers: in a RAM block its write data had a long way from the
// PICO-8 RAM's)
(* ramstyle = "logic" *) reg [7:0] tb [0:31];
reg  [31:0] tv;
reg  [15:0] mp;
reg   [4:0] ti;
reg   [1:0] bub;
reg         bub_flags;      // after the bubble: the flag pass (else the mask)
reg  [15:0] mneed;
wire  [4:0] n_m1    = r_map0[28:24];
wire  [7:0] layers  = r_map1[7:0];
wire [15:0] mp_a    = r_map0[16] ? mp : 16'h1000 + {3'd0, mp[12:0]};
wire  [7:0] tcur    = tb[ti];
wire  [7:0] tsw     = tb[sw[4:0]];
wire [11:0] sw0     = r_scol[11:0];

wire        row_end = sub == S_DST && k == nk_m1;
wire [11:0] sw_step = flip ? sw - 12'd1 : sw + 12'd1;
wire        src_ok  = mapop ? sw[11:5] == 7'd0 && tv[sw[4:0]] : srow[11:7] == 5'd0 && sw[11:4] == 8'd0;
wire [13:0] src_a   = mapop ? {gbpage, 6'd0} + {3'd0, tsw[7:4], 7'd0} + {7'd0, srow[2:0], 4'd0} + {10'd0, tsw[3:0]}
                            : {gbpage, 6'd0} + {srow[6:0], 4'd0} + {10'd0, sw[3:0]};

// destination words (relative to the first) whose pixels come from a
// drawn tile: word k shows tiles sw0 + k and, unless the shift is 0, the
// next one
wire [32:0] tv_x    = {tv, 1'b0} >> (sw0[5:0] + 6'd1);
wire [15:0] kmask   = 16'hffff >> (4'd15 - nk_m1);
wire [15:0] mneed_c = (tv_x[15:0] | (sh != 3'd0 ? tv_x[16:1] : 16'd0)) & kmask;
// first needed word, and the first after k
function [4:0] first1(input [15:0] m);   // {found, index}
	integer j;
	begin
		first1 = 5'd0;
		for (j = 15; j >= 0; j = j - 1) if (m[j]) first1 = {1'b1, j[3:0]};
	end
endfunction
wire  [4:0] nk_first = first1(mneed);
wire  [4:0] nk_next  = first1(mneed & (16'hfffe << k));

// sspr state: column DDA (c + cr / dw), row DDA (sr + rr / dh), row
// buffer (the loaded sheet words of row lrow), pixel position
wire [15:0] c0 = r_ss[0][15:0], cr0 = r_ss[0][31:16], cqs = r_ss[1][15:0], crs = r_ss[1][31:16];
wire [15:0] sy0 = r_ss[2][15:0], rr0 = r_ss[2][31:16], rqs = r_ss[3][15:0], rrs = r_ss[3][31:16];
wire [15:0] dw = r_ss[4][15:0], dh = r_ss[4][31:16];
wire  [3:0] lw0 = r_ss[5][3:0], lw1 = r_ss[5][7:4];
wire        noload = r_ss[5][8];
wire  [2:0] pi0 = r_ss[5][18:16];
wire  [6:0] np_m1 = r_ss[5][30:24];

reg  [15:0] c, cr, sr, rr, lrow;
reg   [3:0] lw;
reg         loaded, pfirst;
reg   [2:0] pi;
reg   [6:0] pcnt;

// one DDA step: q + r/d by qs + rs/d, backwards when `back`
function [31:0] dda(input [15:0] q, input [15:0] r, input [15:0] qs, input [15:0] rs, input [15:0] d, input back);
	reg [16:0] t;
	reg        cy;
	begin
		if (back) begin
			t  = {1'b0, r} - {1'b0, rs};
			cy = t[16];
			dda = {cy ? t[15:0] + d : t[15:0], q - qs - {15'd0, cy}};
		end else begin
			t  = {1'b0, r} + {1'b0, rs};
			cy = t >= {1'b0, d};
			dda = {cy ? t[15:0] - d : t[15:0], q + qs + {15'd0, cy}};
		end
	end
endfunction

wire [31:0] col_next = dda(c, cr, cqs, crs, dw, flip);
wire [31:0] row_next = dda(sr, rr, rqs, rrs, dh, rstep_n);
wire        row_ok   = sr[15:7] == 9'd0;
wire        need     = row_ok && !noload && (!loaded || lrow != sr);
wire [13:0] ld_a     = {gbpage, 6'd0} + {sr[6:0], 4'd0} + {10'd0, lw};
wire [13:0] ld_a0    = {gbpage, 6'd0} + {sr[6:0], 4'd0} + {10'd0, lw0};
wire [13:0] ld_a1    = {gbpage, 6'd0} + {sr[6:0], 4'd0} + {10'd0, lw + 4'd1};

// pixel stage registers (sspr)
reg         ld_v, p1_v, p1_first, p1_last;
reg   [3:0] ld_w;
reg   [2:0] p1_c;
reg   [7:0] p1_oh;         // the pixel's nibble, one-hot (each merged bit is one LUT)
reg         p1_cok;
reg  [13:0] p1_addr;
wire [31:0] rb_q;

// row buffer: 16 sheet words, read at the pixel's column (data in P1)
p8_bram #(.AW(4), .DW(32), .BYPASS(0)) rowbuf (
	.clk(clk), .re(1'b1), .raddr(c[6:3]), .rdata(rb_q), .rdata_raw(),
	.we(ld_v), .waddr(ld_w), .wdata(rdata)
);
wire  [3:0] p1_pix = p1_cok ? rb_q[4 * p1_c +: 4] : 4'd0;

// tline state: map position (16.16, masked), pixels left, destination word
// and nibble, pattern row, the tile of the last lookup and whether it is
// drawn, the destination word read for this pixel
localparam T_PIX = 3'd0, T_MA = 3'd1, T_W1 = 3'd2, T_W2 = 3'd3, T_FL = 3'd4, T_FW1 = 3'd5, T_FW2 = 3'd6;
reg   [2:0] tsub;
reg  [31:0] tmx, tmy;
reg   [7:0] tcnt, ttx, tty, ttile;
reg  [13:0] twa;
reg   [2:0] tnib;
reg   [1:0] trow;
reg         tfirst, tdread, tcell_v, tdrawn;
reg   [1:0] tb_sel;          // byte of the map / flag word read
wire  [7:0] tb_byte = rdata[8 * tb_sel +: 8];
reg  [15:0] tcell;
wire  [7:0] t_offx = r_tl[6][7:0], t_offy = r_tl[6][15:8], t_wm = r_tl[6][23:16], t_hm = r_tl[6][31:24];
wire [15:0] t_base = r_tl[7][15:0];
wire        t_high = r_tl[7][16], t_zero = r_tl[7][17];
wire  [3:0] t_wsh  = r_tl[7][23:20];
wire  [7:0] t_layers = r_tl[7][31:24];
wire [15:0] t_pat  = r_tl[8][15:0];
wire        t_vert = r_tl[8][24], t_back = r_tl[8][25];
wire  [7:0] t_tx   = (t_offx + tmx[23:16]) & t_wm;
wire  [7:0] t_ty   = (t_offy + tmy[23:16]) & t_hm;
wire        t_hit  = tcell_v && tcell == {t_ty, t_tx};
wire        t_wfirst = t_vert || tfirst || tnib == (t_back ? 3'd7 : 3'd0);
wire        t_wlast  = t_vert || tcnt == 8'd0 || tnib == (t_back ? 3'd0 : 3'd7);
// map address of tile (ttx, tty), registered when it was found
wire [15:0] t_p    = t_base + ({8'd0, tty} << t_wsh) + {8'd0, ttx};
wire [15:0] t_ma   = t_high ? t_p : 16'h1000 + {3'd0, t_p[12:0]};
// sheet word of the pixel: row (y >> 13) & 7 of sprite ttile
wire [13:0] t_sa   = {gbpage, 6'd0} + {3'd0, ttile[7:4], 7'd0} + {7'd0, tmy[15:13], 4'd0} + {10'd0, ttile[3:0]};
wire  [3:0] t_pidx = 4'd15 - {2'd0, tnib[1:0]} - {trow, 2'b00};
// circle state: the walk (dx, dy, err), the step's visible spans (A, B:
// rows cy -+ dy over cx -+ dx; C, D: rows cy -+ dx over cx -+ dy), their
// rows and clipped ranges, and the span being filled
localparam C_STEP = 2'd0, C_SETUP = 2'd1, C_WORD = 2'd2;
reg   [1:0] csub;
reg  signed [11:0] cdx, cdy, crm1;
reg  signed [15:0] cerr;
reg   [3:0] cvis;
reg   [6:0] cy_a, cy_b, cy_c, cy_d, ca_x, cb_x, ca_y, cb_y;
reg   [3:0] ck, ck1;
reg  [13:0] caddr;
reg         cfirst;
reg   [7:0] clm, crm;
reg   [1:0] cph;
wire signed [17:0] c_cx = {{2{r_cr[0][15]}}, r_cr[0][15:0]};
wire signed [17:0] c_cy = {{2{r_cr[0][31]}}, r_cr[0][31:16]};
wire signed [17:0] c_x0 = {10'd0, r_cr[2][7:0]},   c_x1 = {10'd0, r_cr[2][15:8]} - 18'sd1;
wire signed [17:0] c_y0 = {10'd0, r_cr[2][23:16]}, c_y1 = {10'd0, r_cr[2][31:24]};
wire signed [17:0] c_dx = {{6{cdx[11]}}, cdx}, c_dy = {{6{cdy[11]}}, cdy};
wire signed [17:0] cya = c_cy - c_dy, cyb = c_cy + c_dy, cyc = c_cy - c_dx, cyd = c_cy + c_dx;
wire signed [17:0] cxa_x = c_cx - c_dx, cxb_x = c_cx + c_dx, cxa_y = c_cx - c_dy, cxb_y = c_cx + c_dy;
wire signed [17:0] cca_x = cxa_x > c_x0 ? cxa_x : c_x0, ccb_x = cxb_x < c_x1 ? cxb_x : c_x1;
wire signed [17:0] cca_y = cxa_y > c_x0 ? cxa_y : c_x0, ccb_y = cxb_y < c_x1 ? cxb_y : c_x1;
wire        c_okx  = cca_x <= ccb_x, c_oky = cca_y <= ccb_y;
wire        c_in_a = cya >= c_y0 && cya < c_y1, c_in_b = cyb >= c_y0 && cyb < c_y1;
wire        c_in_c = cyc >= c_y0 && cyc < c_y1, c_in_d = cyd >= c_y0 && cyd < c_y1;
// dx moves on (err >= r - 1) or the walk ends after this step
wire        c_mv   = !(cerr < $signed({{4{crm1[11]}}, crm1}));
wire        c_cd   = c_mv || cdx <= cdy;
wire  [3:0] c_vis  = {c_cd && cdx != 12'sd0 && c_in_d && c_oky, c_cd && c_in_c && c_oky,
                      cdy != 12'sd0 && c_in_b && c_okx, c_in_a && c_okx};
wire signed [15:0] c_ndy = {{4{cdy[11]}}, cdy} + 16'sd1;
// the next visible span
wire  [1:0] c_j    = cvis[0] ? 2'd0 : cvis[1] ? 2'd1 : cvis[2] ? 2'd2 : 2'd3;
wire  [6:0] c_sy   = c_j == 2'd0 ? cy_a : c_j == 2'd1 ? cy_b : c_j == 2'd2 ? cy_c : cy_d;
wire  [6:0] c_sa   = c_j[1] ? ca_y : ca_x;
wire  [6:0] c_sb   = c_j[1] ? cb_y : cb_x;

// oval state: the products, the walk (r, v, w = v - d; differences e1 =
// v² - m, e2 = w² - m; kup = dv + dd, kdn = dv - 3dd; dm and its step; the
// stop test u and its step), the column / row of the step, the previous
// column (rows lo..hi), the step's two row ranges (clipped) and x range
localparam O_M1 = 4'd0, O_M2 = 4'd1, O_M3 = 4'd2, O_INIT = 4'd3, O_ADJ = 4'd4, O_E1 = 4'd5, O_E2 = 4'd6,
           O_E3 = 4'd7, O_SETUP = 4'd8, O_WORD = 4'd9, O_NEXT = 4'd10;
reg   [3:0] osub;
reg         owalk, ofinal, ohave, oj;
reg  [15:0] m_a2, m_b2, m_ab;
reg  [23:0] m_a2b, m_b2a;
reg  [31:0] m_a4, m_b4;
reg  [16:0] m_s;
reg  signed [17:0] wr, wv, ww;
reg   [8:0] wd;
reg  signed [31:0] we1, we2, wkup, wkdn, wdm;
reg  [21:0] wdd2, wdm2, ws8;
reg  signed [34:0] wu, wdu;
reg  signed [17:0] ores, ox, oy18, oyb18, ol, oh, opx, oplo, ophi, oxa, oxb, pa, pb;
reg  signed [17:0] rya [0:1], ryb [0:1];
reg   [1:0] rv;
reg   [6:0] oy, oya [0:1], oyb [0:1];
reg   [1:0] ovis;
wire  [7:0] o_a2 = r_ov[1][7:0], o_b2 = r_ov[1][15:8];
wire signed [17:0] o_xx = {{2{r_ov[0][15]}}, r_ov[0][15:0]};
wire signed [17:0] o_yy = {{2{r_ov[0][31]}}, r_ov[0][31:16]};
wire signed [17:0] o_x0 = {10'd0, r_ov[2][7:0]},   o_x1 = {10'd0, r_ov[2][15:8]} - 18'sd1;
wire signed [17:0] o_y0 = {10'd0, r_ov[2][23:16]}, o_y1 = {10'd0, r_ov[2][31:24]} - 18'sd1;
// a walk's start (columns: c = y0 + y1, p = a2, o = b2; rows: x0 + x1, b2,
// a2): at step 0 the root is exact (p * o), see blitter::Walk::start
wire signed [17:0] ws_c   = owalk ? o_xx : o_yy;
wire  [7:0] ws_p   = owalk ? o_b2 : o_a2;
wire  [7:0] ws_o   = owalk ? o_a2 : o_b2;
wire [31:0] ws_p2  = {16'd0, owalk ? m_b2 : m_a2};
wire [31:0] ws_o2  = {16'd0, owalk ? m_a2 : m_b2};
wire [31:0] ws_p2o = {8'd0, owalk ? m_b2a : m_a2b};
wire [31:0] ws_lim = owalk ? m_b4 : m_a4;
wire        ws_odd = ws_c[0] ^ ws_o[0];
wire signed [17:0] ws_cmo = ws_c - $signed({10'd0, ws_o});
wire o_vpos  = !wv[17] && wv != 18'sd0;
wire o_wpos  = !ww[17] && ww != 18'sd0;
wire o_e1pos = !we1[31] && we1 != 32'sd0;
wire o_e2pos = !we2[31] && we2 != 32'sd0;
wire signed [34:0] o_unext = wu - wdu;
// the walk ends at this NEXT: registered a cycle ahead (wu and wdu change
// only in NEXT and INIT, at least four cycles apart), so `run` (and with
// it `own` and `hold`, the CPU's stall and RAM port select) does not wait
// for the 35-bit subtraction; Quartus retimed `run` across it otherwise
reg         o_end;
always @(posedge clk) o_end <= o_unext[34];
// E1: the point's rows (columns) or columns (rows), sorted
wire signed [17:0] o_nres = (owalk ? o_xx : o_yy) - ores;
wire        o_rle  = ores <= o_nres;
// E1, columns: the previous column's x range, clipped
wire signed [17:0] o_pxa  = o_xx - opx;
// E3: the ranges clipped to the rows
wire signed [17:0] o_cya0 = rya[0] > o_y0 ? rya[0] : o_y0, o_cyb0 = ryb[0] < o_y1 ? ryb[0] : o_y1;
wire signed [17:0] o_cya1 = rya[1] > o_y0 ? rya[1] : o_y0, o_cyb1 = ryb[1] < o_y1 ? ryb[1] : o_y1;
wire        o_xv   = pa <= pb;
wire        o_v0   = rv[0] && o_xv && o_cya0 <= o_cyb0;
wire        o_v1   = rv[1] && o_xv && o_cya1 <= o_cyb1;
wire  [6:0] o_yend = oj ? oyb[1] : oyb[0];

// the issued sheet read's pixel
reg   [2:0] ti_fx, ti_nib;
reg         ti_pbit, ti_drawn, ti_first, ti_last;
reg  [13:0] ti_wa;

always @(posedge clk) begin
	if (reset) begin
		run  <= 1'b0;
		i_v  <= 1'b0;
		ld_v <= 1'b0;
		p1_v <= 1'b0;
	end else if (start) begin
		run       <= 1'b1;
		op        <= wdata[3:0];
		sub       <= wdata[3:0] == OP_BLIT ? S_PRIME : wdata[3:0] == OP_SSPR ? S_ROW : S_DST;
		rows_left <= rows_m1;
		k         <= 4'd0;
		sw        <= r_scol[11:0];
		srow      <= r_srow[11:0];
		drow      <= r_dst[13:0];
		daddr     <= r_dst[13:0];
		phase     <= r_dst[17:16];
		sr        <= sy0;
		rr        <= rr0;
		loaded    <= 1'b0;
		mp        <= r_map0[15:0];
		ti        <= 5'd0;
		tsub      <= T_PIX;
		tmx       <= r_tl[0];
		tmy       <= r_tl[1];
		tcnt      <= r_tl[8][23:16];
		tnib      <= r_tl[8][30:28];
		twa       <= r_dst[13:0];
		trow      <= r_dst[17:16];
		tfirst    <= 1'b1;
		tdread    <= 1'b0;
		tcell_v   <= 1'b0;
		if (wdata[3:0] == OP_MAP) sub <= S_TILE;
		osub      <= O_M1;
		owalk     <= 1'b0;
		ofinal    <= 1'b0;
		csub      <= C_STEP;
		cdx       <= {1'b0, r_cr[1][10:0]};
		cdy       <= 12'sd0;
		cerr      <= 16'sd0;
		crm1      <= {1'b0, r_cr[1][10:0]} - 12'sd1;
		i_v       <= 1'b0;
		ld_v      <= 1'b0;
		p1_v      <= 1'b0;
	end else if (run && op == OP_SSPR) begin
		i_v  <= 1'b0;
		ld_v <= 1'b0;
		p1_v <= 1'b0;
		case (sub)
			S_ROW: begin
				c      <= c0;
				cr     <= cr0;
				pi     <= pi0;
				pcnt   <= np_m1;
				pfirst <= 1'b1;
				daddr  <= drow;
				if (need) begin
					sub    <= S_LOAD;
					lw     <= lw0;
					raddr  <= ld_a0;
					loaded <= 1'b1;
					lrow   <= sr;
				end else begin
					sub    <= S_PIX;
					raddr  <= drow;
				end
			end
			S_LOAD: begin
				// reading sheet word lw (raddr)
				ld_v <= 1'b1;
				ld_w <= lw;
				lw   <= lw + 4'd1;
				if (lw == lw1) sub <= S_BUB;
				raddr <= ld_a1;
			end
			S_BUB: begin
				sub   <= S_PIX;
				raddr <= daddr;
			end
			default: begin
				// S_PIX: pixel at column c; its destination word is read
				// now when it is the first pixel of the word
				p1_v     <= 1'b1;
				p1_cok   <= row_ok && c[15:7] == 9'd0;
				p1_c     <= c[2:0];
				p1_oh    <= 8'd1 << pi;
				p1_first <= pfirst;
				p1_last  <= pi == 3'd7 || pcnt == 7'd0;
				p1_addr  <= daddr;
				{cr, c}  <= col_next;
				pi       <= pi + 3'd1;
				pfirst   <= pi == 3'd7;
				pcnt     <= pcnt - 7'd1;
				if (pi == 3'd7) begin
					daddr <= daddr + 14'd1;
					raddr <= daddr + 14'd1;
				end
				if (pcnt == 7'd0) begin
					sub       <= S_ROW;
					{rr, sr}  <= row_next;
					drow      <= drow + 14'd16;
					rows_left <= rows_left - 7'd1;
					if (rows_left == 7'd0) run <= 1'b0;
				end
			end
		endcase
	end else if (run && op == OP_TLINE) begin
		ld_v   <= 1'b0;
		p1_v   <= 1'b0;
		i_v    <= 1'b0;
		i_kind <= K_NONE;
		case (tsub)
			T_PIX: begin
				if (!t_hit) begin
					// another tile: find it now, read its map byte next
					ttx     <= t_tx;
					tty     <= t_ty;
					tcell   <= {t_ty, t_tx};
					tcell_v <= 1'b1;
					tsub    <= T_MA;
				end else if (t_wfirst && !tdread) begin
					i_v    <= 1'b1;
					i_kind <= K_TD;
					raddr  <= twa;
					tdread <= 1'b1;
				end else begin
					i_v      <= 1'b1;
					i_kind   <= K_TS;
					raddr    <= t_sa;
					ti_fx    <= tmx[15:13];
					ti_nib   <= tnib;
					ti_pbit  <= t_pat[t_pidx];
					ti_drawn <= tdrawn;
					ti_first <= t_wfirst;
					ti_last  <= t_wlast;
					ti_wa    <= twa;
					tmx      <= (tmx + r_tl[2]) & r_tl[4];
					tmy      <= (tmy + r_tl[3]) & r_tl[5];
					tfirst   <= 1'b0;
					tdread   <= 1'b0;
					tcnt     <= tcnt - 8'd1;
					if (t_vert) begin
						twa  <= t_back ? twa - 14'd16 : twa + 14'd16;
						trow <= t_back ? trow - 2'd1 : trow + 2'd1;
					end else if (t_back) begin
						if (tnib == 3'd0) twa <= twa - 14'd1;
						tnib <= tnib - 3'd1;
					end else begin
						if (tnib == 3'd7) twa <= twa + 14'd1;
						tnib <= tnib + 3'd1;
					end
					if (tcnt == 8'd0) run <= 1'b0;
				end
			end
			T_MA: begin
				i_v    <= 1'b1;
				i_kind <= K_TB;
				tb_sel <= t_ma[1:0];
				raddr  <= t_ma[15:2];
				tsub   <= T_W1;
			end
			T_W1: tsub <= T_W2;
			T_W2: begin
				// the map byte (d1)
				ttile  <= tb_byte;
				tdrawn <= tb_byte != 8'd0 || t_zero;
				tsub   <= t_layers != 8'd0 ? T_FL : T_PIX;
			end
			T_FL: begin
				i_v    <= 1'b1;
				i_kind <= K_TB;
				tb_sel <= ttile[1:0];
				raddr  <= 14'h0c00 + {8'd0, ttile[7:2]};
				tsub   <= T_FW1;
			end
			T_FW1: tsub <= T_FW2;
			default: begin
				// T_FW2: the flag byte (d1)
				if ((tb_byte & t_layers) == 8'd0) tdrawn <= 1'b0;
				tsub <= T_PIX;
			end
		endcase
	end else if (run && op == OP_OVAL) begin
		ld_v   <= 1'b0;
		p1_v   <= 1'b0;
		i_v    <= 1'b0;
		i_kind <= K_DST;
		case (osub)
			O_M1: begin
				m_a2 <= o_a2 * o_a2;
				m_b2 <= o_b2 * o_b2;
				m_ab <= o_a2 * o_b2;
				osub <= O_M2;
			end
			O_M2: begin
				m_a2b <= m_a2 * o_b2;
				m_b2a <= m_b2 * o_a2;
				m_a4  <= m_a2 * m_a2;
				m_b4  <= m_b2 * m_b2;
				m_s   <= {1'b0, m_a2} + {1'b0, m_b2};
				osub  <= O_M3;
			end
			O_M3: osub <= O_INIT;
			O_INIT: begin
				wr   <= ws_cmo >>> 1;
				wv   <= $signed({2'd0, m_ab}) + (ws_odd ? $signed({9'd0, ws_p, 1'b0}) : $signed({10'd0, ws_p}));
				ww   <= ws_odd ? $signed({2'd0, m_ab}) : $signed({2'd0, m_ab}) - $signed({10'd0, ws_p});
				wd   <= {ws_p, 1'b0};
				we1  <= ws_odd ? (ws_p2o << 2) + (ws_p2 << 2) : (ws_p2o << 1) + ws_p2;
				we2  <= ws_odd ? 32'sd0 : ws_p2 - (ws_p2o << 1);
				wkup <= ws_odd ? (ws_p2o << 2) + (ws_p2 << 3) + (ws_p2 << 2) : (ws_p2o << 2) + (ws_p2 << 3);
				wkdn <= ws_odd ? (ws_p2o << 2) - (ws_p2 << 2) : (ws_p2o << 2) - (ws_p2 << 3);
				wdd2 <= ws_p2[18:0] << 3;
				wdm  <= ws_o2 << 2;
				wdm2 <= ws_o2[18:0] << 3;
				wu   <= {3'd0, ws_lim};
				wdu  <= {16'd0, m_s, 2'd0};
				ws8  <= {2'd0, m_s, 3'd0};
				ox   <= (o_xx + 18'sd1) >>> 1;
				oy18 <= (o_yy + 18'sd1) >>> 1;
				oyb18 <= o_yy - ((o_yy + 18'sd1) >>> 1);
				ohave <= 1'b0;
				osub <= O_ADJ;
			end
			O_ADJ: begin
				// one move of v, or the point
				if (!(o_vpos && o_e1pos)) begin
					wr   <= wr - 18'sd1;
					we2  <= we1;
					we1  <= we1 + wkup;
					wkup <= wkup + $signed({10'd0, wdd2});
					wkdn <= wkdn + $signed({10'd0, wdd2});
					ww   <= wv;
					wv   <= wv + $signed({9'd0, wd});
				end else if (o_wpos && o_e2pos) begin
					wr   <= wr + 18'sd1;
					we1  <= we2;
					we2  <= we2 - wkdn;
					wkup <= wkup - $signed({10'd0, wdd2});
					wkdn <= wkdn - $signed({10'd0, wdd2});
					wv   <= ww;
					ww   <= ww - $signed({9'd0, wd});
				end else begin
					ores <= (!ww[17] && we2 == 32'sd0 && !wr[17]) ? wr + 18'sd1 : wr;
					osub <= O_E1;
				end
			end
			O_E1: begin
				// columns: the point's rows lo..hi and the previous column's
				// x range; rows: the point's x range
				ol  <= o_rle ? ores : o_nres;
				oh  <= o_rle ? o_nres : ores;
				oxa <= o_rle ? ores : o_nres;
				oxb <= o_rle ? o_nres : ores;
				pa  <= o_pxa > o_x0 ? o_pxa : o_x0;
				pb  <= opx < o_x1 ? opx : o_x1;
				osub <= O_E2;
			end
			O_E2: begin
				if (owalk) begin
					pa     <= oxa > o_x0 ? oxa : o_x0;
					pb     <= oxb < o_x1 ? oxb : o_x1;
					rya[0] <= oy18;
					ryb[0] <= oy18;
					rya[1] <= oyb18;
					ryb[1] <= oyb18;
					rv     <= 2'b11;
				end else if (ofinal || !(ol >= oplo && oh <= ophi)) begin
					rya[0] <= oplo;
					ryb[0] <= ophi;
					rv     <= {1'b0, ohave};
				end else begin
					rya[0] <= oplo;
					ryb[0] <= ol - 18'sd1;
					rya[1] <= oh + 18'sd1;
					ryb[1] <= ophi;
					rv     <= {ohave, ohave};
				end
				osub <= O_E3;
			end
			O_E3: begin
				oya[0] <= o_cya0[6:0];
				oyb[0] <= o_cyb0[6:0];
				oya[1] <= o_cya1[6:0];
				oyb[1] <= o_cyb1[6:0];
				ovis   <= {o_v1, o_v0};
				if (!owalk && !ofinal) begin
					opx   <= ox;
					oplo  <= ol;
					ophi  <= oh;
					ohave <= 1'b1;
				end
				oj <= !o_v0;
				oy <= o_v0 ? o_cya0[6:0] : o_cya1[6:0];
				osub <= o_v0 || o_v1 ? O_SETUP : ofinal ? O_INIT : O_NEXT;
				if (ofinal && !(o_v0 || o_v1)) begin
					owalk  <= 1'b1;
					ofinal <= 1'b0;
				end
			end
			O_SETUP: begin
				ck     <= pa[6:3];
				ck1    <= pb[6:3];
				caddr  <= r_dst[13:0] + {3'd0, oy, 4'd0} + {10'd0, pa[6:3]};
				cfirst <= 1'b1;
				clm    <= 8'hff << pa[2:0];
				crm    <= 8'hff >> (3'd7 - pb[2:0]);
				cph    <= oy[1:0];
				osub   <= O_WORD;
			end
			O_WORD: begin
				i_v      <= 1'b1;
				raddr    <= caddr;
				i_first  <= cfirst;
				i_second <= 1'b0;
				i_last   <= ck == ck1;
				i_lm     <= clm;
				i_rm     <= crm;
				i_phase  <= cph;
				caddr    <= caddr + 14'd1;
				ck       <= ck + 4'd1;
				cfirst   <= 1'b0;
				if (ck == ck1) begin
					if (oy != o_yend) begin
						oy   <= oy + 7'd1;
						osub <= O_SETUP;
					end else if (!oj && ovis[1]) begin
						oj   <= 1'b1;
						oy   <= oya[1];
						osub <= O_SETUP;
					end else if (ofinal) begin
						owalk  <= 1'b1;
						ofinal <= 1'b0;
						osub   <= O_INIT;
					end else begin
						osub <= O_NEXT;
					end
				end
			end
			default: begin
				// O_NEXT: the next step, or the end of the walk
				we1   <= we1 + wdm;
				we2   <= we2 + wdm;
				wdm   <= wdm + $signed({10'd0, wdm2});
				wu    <= o_unext;
				wdu   <= wdu + $signed({13'd0, ws8});
				ox    <= ox + 18'sd1;
				oy18  <= oy18 + 18'sd1;
				oyb18 <= oyb18 - 18'sd1;
				if (!o_end) osub <= O_ADJ;
				else if (!owalk) begin
					ofinal <= 1'b1;
					osub   <= O_E1;
				end else run <= 1'b0;
			end
		endcase
	end else if (run && op == OP_CIRC) begin
		ld_v   <= 1'b0;
		p1_v   <= 1'b0;
		i_v    <= 1'b0;
		i_kind <= K_DST;
		case (csub)
			C_STEP: begin
				if (cdx < cdy) begin
					run <= 1'b0;
				end else begin
					cvis <= c_vis;
					cy_a <= cya[6:0];
					cy_b <= cyb[6:0];
					cy_c <= cyc[6:0];
					cy_d <= cyd[6:0];
					ca_x <= cca_x[6:0];
					cb_x <= ccb_x[6:0];
					ca_y <= cca_y[6:0];
					cb_y <= ccb_y[6:0];
					cdy  <= cdy + 12'sd1;
					if (c_mv) begin
						cdx  <= cdx - 12'sd1;
						cerr <= cerr + 16'sd1 + 16'sd2 * (c_ndy - {{4{cdx[11]}}, cdx} + 16'sd1);
					end else begin
						cerr <= cerr + 16'sd1 + 16'sd2 * c_ndy;
					end
					if (c_vis != 4'd0) csub <= C_SETUP;
				end
			end
			C_SETUP: begin
				cvis   <= cvis & ~(4'd1 << c_j);
				ck     <= c_sa[6:3];
				ck1    <= c_sb[6:3];
				caddr  <= r_dst[13:0] + {3'd0, c_sy, 4'd0} + {10'd0, c_sa[6:3]};
				cfirst <= 1'b1;
				clm    <= 8'hff << c_sa[2:0];
				crm    <= 8'hff >> (3'd7 - c_sb[2:0]);
				cph    <= c_sy[1:0];
				csub   <= C_WORD;
			end
			default: begin
				// C_WORD: one destination word of the span
				i_v      <= 1'b1;
				raddr    <= caddr;
				i_first  <= cfirst;
				i_second <= 1'b0;
				i_last   <= ck == ck1;
				i_lm     <= clm;
				i_rm     <= crm;
				i_phase  <= cph;
				caddr    <= caddr + 14'd1;
				ck       <= ck + 4'd1;
				cfirst   <= 1'b0;
				if (ck == ck1) csub <= cvis != 4'd0 ? C_SETUP : C_STEP;
			end
		endcase
	end else if (run && mapop && sub >= S_TILE) begin
		// map row set-up: tile numbers, flags, needed words
		ld_v    <= 1'b0;
		p1_v    <= 1'b0;
		i_v     <= 1'b1;
		i_kind  <= K_NONE;
		i_slot  <= ti;
		case (sub)
			S_TILE: begin
				i_kind <= K_TILE;
				i_bsel <= mp_a[1:0];
				raddr  <= mp_a[15:2];
				mp     <= mp + 16'd1;
				ti     <= ti + 5'd1;
				if (ti == n_m1) begin
					sub       <= S_WAIT;
					bub       <= 2'd0;
					bub_flags <= layers != 8'd0;
					ti        <= 5'd0;
				end
			end
			S_FLAG: begin
				i_kind <= K_FLAG;
				i_bsel <= tcur[1:0];
				raddr  <= 14'h0c00 + {8'd0, tcur[7:2]};
				ti     <= ti + 5'd1;
				if (ti == n_m1) begin
					sub       <= S_WAIT;
					bub       <= 2'd0;
					bub_flags <= 1'b0;
				end
			end
			S_WAIT: begin
				// two cycles until the last read's data is stored
				bub <= bub + 2'd1;
				if (bub == 2'd1) sub <= bub_flags ? S_FLAG : S_MASK;
			end
			S_MASK: begin
				mneed <= mneed_c;
				sub  <= S_MROW;
			end
			default: begin
				// S_MROW: start a row at its first needed word
				if (!nk_first[4]) begin
					run <= 1'b0;
				end else begin
					k     <= nk_first[3:0];
					sw    <= sw0 + {8'd0, nk_first[3:0]};
					daddr <= drow + {10'd0, nk_first[3:0]};
					sub   <= S_PRIME;
				end
			end
		endcase
	end else if (run) begin
		ld_v    <= 1'b0;
		p1_v    <= 1'b0;
		i_v     <= 1'b1;
		i_kind  <= sub == S_DST ? K_DST : K_SRC;
		i_ok    <= src_ok;
		i_first <= k == 4'd0;
		i_second <= k == 4'd1;
		i_last  <= k == nk_m1;
		i_lm    <= lmask;
		i_rm    <= rmask;
		i_row   <= rows_m1[2:0] - rows_left[2:0];
		i_phase <= phase;
		raddr   <= sub == S_DST ? daddr : src_a;
		case (sub)
			S_PRIME: begin sw <= sw_step; sub <= S_SRC; end
			S_SRC:   begin sw <= sw_step; sub <= S_DST; end
			default: begin
				k     <= k + 4'd1;
				daddr <= daddr + 14'd1;
				sub   <= op == OP_BLIT || mapop ? S_SRC : S_DST;
				if (mapop && (k == 4'd15 || !mneed[k + 4'd1]) && nk_next[4]) begin
					// skip words without a drawn tile
					k     <= nk_next[3:0];
					sw    <= sw0 + {8'd0, nk_next[3:0]};
					daddr <= drow + {10'd0, nk_next[3:0]};
					sub   <= S_PRIME;
				end
			end
		endcase
		if (mapop && sub == S_DST && !nk_next[4]) begin
			srow  <= srow + 12'd1;
			drow  <= drow + 14'd16;
			sub   <= S_MROW;
			rows_left <= rows_left - 7'd1;
			if (rows_left == 7'd0) run <= 1'b0;
		end else if (row_end) begin
			k     <= 4'd0;
			sw    <= r_scol[11:0];
			srow  <= rstep_n ? srow - 12'd1 : srow + 12'd1;
			drow  <= drow + 14'd16;
			daddr <= drow + 14'd16;
			phase <= phase + 2'd1;
			sub   <= op == OP_BLIT ? S_PRIME : S_DST;
			rows_left <= rows_left - 7'd1;
			if (rows_left == 7'd0) run <= 1'b0;
		end
	end else begin
		i_v  <= 1'b0;
		ld_v <= 1'b0;
		p1_v <= 1'b0;
	end
end

// sspr: row buffer fill, pixel merge into the word being assembled
reg  [31:0] asm_w, wbuf;
reg         w_v;
reg  [13:0] w_addr;
wire [31:0] p1_base = p1_first ? rdata : asm_w;
wire  [7:0] p1_e    = r_pal[8 * p1_pix +: 8];
reg  [31:0] p1_word;
integer pk;
always @* begin
	for (pk = 0; pk < 8; pk = pk + 1)
		p1_word[4*pk +: 4] = p1_oh[pk] ? (p1_base[4*pk +: 4] & p1_e[7:4]) | p1_e[3:0] : p1_base[4*pk +: 4];
end


// ---------------------------------------------------------------- data

function [31:0] nrev(input [31:0] w);
	integer j;
	for (j = 0; j < 8; j = j + 1) nrev[4*j +: 4] = w[4*(7-j) +: 4];
endfunction

reg        d1_v, d1_ok, d1_first, d1_second, d1_last;
reg  [2:0] d1_kind;
reg  [1:0] d1_phase, d1_bsel;
reg  [7:0] d1_lm, d1_rm;
reg [31:0] d1_wt, d1_wf;      // map row: d1 holds the tile / flag byte of slot k (one-hot)
reg        wok_a, wok_b;      // map row: window words come from drawn tiles
wire       d1_dst  = d1_kind == K_DST;
wire [7:0] d1_byte = rdata[8 * d1_bsel +: 8];
reg  [2:0] d1_row;
reg [13:0] d1_addr;
reg [31:0] win_a, win_b;      // window: {b, a}, a fetched first

// tline data stage (d1): the destination word (K_TD) is held; a sheet word
// (K_TS) gives the pixel, which merges into the word being assembled
reg   [2:0] td_fx, td_nib;
reg         td_pbit, td_drawn, td_first, td_last;
reg  [13:0] td_wa;
reg  [31:0] tdreg, tasm;
always @(posedge clk) begin
	td_fx    <= ti_fx;
	td_nib   <= ti_nib;
	td_pbit  <= ti_pbit;
	td_drawn <= ti_drawn;
	td_first <= ti_first;
	td_last  <= ti_last;
	td_wa    <= ti_wa;
end
wire        td_v    = d1_v && d1_kind == K_TS;
// the pixel's colour is registered first (tp: RAM data and the nibble
// select, then the write table and the merge, in two cycles)
reg   [3:0] tp_c;
reg   [7:0] tp_oh;         // the pixel's nibble, one-hot (each merged bit is one LUT)
reg         tp_v, tp_pbit, tp_drawn, tp_first, tp_last;
reg  [13:0] tp_wa;
always @(posedge clk) begin
	tp_v     <= !reset && td_v;
	tp_c     <= rdata[4 * td_fx +: 4];
	tp_oh    <= 8'd1 << td_nib;
	tp_pbit  <= td_pbit;
	tp_drawn <= td_drawn;
	tp_first <= td_first;
	tp_last  <= td_last;
	tp_wa    <= td_wa;
end
wire  [7:0] tp_t    = tp_pbit ? r_gmask[8 * tp_c +: 8] : r_pal[8 * tp_c +: 8];
wire  [7:0] tp_e    = tp_drawn ? tp_t : 8'hf0;
wire [31:0] tp_base = tp_first ? tdreg : tasm;
reg  [31:0] tp_word;
integer tk;
always @* begin
	for (tk = 0; tk < 8; tk = tk + 1)
		tp_word[4*tk +: 4] = tp_oh[tk] ? (tp_base[4*tk +: 4] & tp_e[7:4]) | tp_e[3:0] : tp_base[4*tk +: 4];
end
always @(posedge clk) begin
	if (d1_v && d1_kind == K_TD) tdreg <= rdata;
	if (tp_v) tasm <= tp_word;
end

// (sspr and tline share the registered write)
always @(posedge clk) begin
	if (p1_v) asm_w <= p1_word;
	if (op == OP_TLINE) begin
		wbuf   <= tp_word;
		w_addr <= tp_wa;
		w_v    <= !reset && tp_v && tp_last;
	end else begin
		wbuf   <= p1_word;
		w_addr <= p1_addr;
		w_v    <= !reset && p1_v && p1_last;
	end
end


wire [63:0] win = {win_b, win_a};
wire [15:0] gm  = r_gmask[16 * d1_row +: 16];
wire  [7:0] gw  = op != OP_GLYPH ? 8'hff : d1_first ? gm[7:0] : d1_second ? gm[15:8] : 8'h00;
wire  [7:0] pm  = (d1_first ? d1_lm : 8'hff) & (d1_last ? d1_rm : 8'hff) & gw & vm;
// map row: pixel i comes from window word b when sh + i >= 8
wire [15:0] vw  = {{8{wok_b}}, {8{wok_a}}} >> sh;
wire  [7:0] vm  = mapop ? vw[7:0] : 8'hff;
wire        pal_op = op == OP_BLIT || mapop;

wire [31:0] kv_keep, kv_val;
genvar gi;
generate for (gi = 0; gi < 8; gi = gi + 1) begin : px
	wire [3:0] c = win[4 * ({1'b0, sh} + gi) +: 4];
	wire [7:0] e = r_pal[8 * c +: 8];
	wire [3:0] fk = r_fkeep[32 * d1_phase + 4 * gi +: 4];
	wire [3:0] fv = r_fval[32 * d1_phase + 4 * gi +: 4];
	assign kv_keep[4*gi +: 4] = !pm[gi] ? 4'hf : pal_op ? e[7:4] : fk;
	assign kv_val[4*gi +: 4]  = !pm[gi] ? 4'h0 : pal_op ? e[3:0] : fv;
end endgenerate

reg        d2_v;
reg [13:0] d2_addr;
reg [31:0] d2_old, d2_keep, d2_val;

always @(posedge clk) begin
	if (reset) begin
		d1_v  <= 1'b0;
		d2_v  <= 1'b0;
		own   <= 1'b0;
		d1_wt <= 32'd0;
		d1_wf <= 32'd0;
	end else begin
		d1_v     <= i_v;
		d1_wt    <= (i_v && i_kind == K_TILE) ? 32'd1 << i_slot : 32'd0;
		d1_wf    <= (i_v && i_kind == K_FLAG) ? 32'd1 << i_slot : 32'd0;
		d1_kind  <= i_kind;
		d1_bsel  <= i_bsel;
		d1_ok    <= i_ok;
		d1_first <= i_first;
		d1_second <= i_second;
		d1_last  <= i_last;
		d1_row   <= i_row;
		d1_phase <= i_phase;
		d1_lm    <= i_lm;
		d1_rm    <= i_rm;
		d1_addr  <= raddr;
		if (d1_v && d1_kind == K_SRC) begin
			win_a <= win_b;
			win_b <= !d1_ok ? 32'd0 : flip ? nrev(rdata) : rdata;
			wok_a <= wok_b;
			wok_b <= d1_ok;
		end
		d2_v    <= d1_v && d1_dst;
		d2_addr <= d1_addr;
		d2_old  <= rdata;
		d2_keep <= kv_keep;
		d2_val  <= kv_val;
		own     <= run || i_v || d1_v || d2_v || ld_v || p1_v || w_v || tp_v;
	end
end

// map row: tile numbers and drawn flags (the slot's enable decoded a stage
// earlier: d1_wt / d1_wf)
integer mk;
always @(posedge clk) begin
	if (start) tv <= 32'd0;
	for (mk = 0; mk < 32; mk = mk + 1) begin
		if (d1_wt[mk]) begin
			tb[mk] <= d1_byte;
			tv[mk] <= d1_byte != 8'd0 || r_map1[8];
		end
		if (d1_wf[mk] && (d1_byte & layers) == 8'd0) tv[mk] <= 1'b0;
	end
end

always @* begin
	we    = d2_v || w_v;
	waddr = w_v ? w_addr : d2_addr;
end
assign wdat   = w_v ? wbuf : (d2_old & d2_keep) | d2_val;
assign status = start || run || own;
// three registers (ctrl_wr: the CTRL write, as start): from the cycle
// after the CPU's CTRL store to the last write
assign hold   = ctrl_wr || run || own;

endmodule
