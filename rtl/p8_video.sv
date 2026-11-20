// PICO-8 display: 128x128 in a 15 kHz / 60 Hz raster.
//
// The CPU draws into PICO-8 RAM and then requests a present (`present`,
// the FRAME MMIO write). At the next vertical blank the copy engine reads
// the draw-state bytes (0x5f00-0x5f7f) and the screen (8 KiB at
// peek(0x5f55)<<8) from PICO-8 RAM into the display buffer; then
// `vcount` increments. `vcount` also increments on vblanks without a
// present, so it is a 60 Hz tick the firmware paces itself with.
//
// Scanout implements the screen palette (0x5f10), screen modes (0x5f2c:
// stretch, mirror, flip, rotate) and raster modes (0x5f5f with 0x5f60 /
// 0x5f70) like p8_gfx::render_rgba.
//
// Until the first present the display buffer shows the splash screen
// (splash.hex, made from splash.p8 by fpga/sim/make-splash.sh).
//
// With `show_fps` the top-left corner shows the number of frames
// presented in the last second (2 digits, 3x5 font).
//
// A spinner (SPINNER register: bit 15 on, bits 14-8 y, 6-0 x) draws a
// 5x5 ring of 8 dots over the display, a bright head with a fading tail
// going round once every 32 vblanks. The next completed present turns it
// off: the firmware starts it under the loading screen before the cart's
// top-level code and _init run, which may take seconds without a frame.
//
// Timing (DIV clocks per pixel): H_TOTAL x 262 pixels at ~60 Hz (398 at
// 100 MHz), active 128 x 128 centred in the visible area of a 240-line
// NTSC-like frame; 4.7 us horizontal sync.

module p8_video #(
	parameter DIV = 16,
	parameter CLK_HZ = 100000000
) (
	input             clk,
	input             reset,

	// read port into PICO-8 RAM (32-bit words)
	output reg [13:0] cp_addr,
	output            cp_req,
	input             cp_gnt,     // read issued this cycle, data next cycle
	input      [31:0] cp_rdata,

	input             present,
	input             show_fps,
	input             spin_wr,
	input      [15:0] spin_data,
	output reg [31:0] vcount,

	output reg        ce_pix,
	output reg  [7:0] r,
	output reg  [7:0] g,
	output reg  [7:0] b,
	output reg        hs,
	output reg        vs,
	output reg        hblank,
	output reg        vblank
);

localparam V_TOTAL = 262, V_SYNC = 187, V_SYNC_END = 190;
localparam PIX_HZ = CLK_HZ / DIV;
localparam H_TOTAL = (PIX_HZ + V_TOTAL * 30) / (V_TOTAL * 60);
localparam H_SYNC = H_TOTAL - 128, H_SYNC_END = H_SYNC + PIX_HZ / 1000 * 47 / 10000;

// ---------------------------------------------------------------- timing

reg  [4:0] div;
reg  [8:0] hc;
reg  [8:0] vc;
wire       tick = div == DIV - 1;

always @(posedge clk) begin
	if (reset) begin
		div <= 5'd0;
		hc  <= 9'd0;
		vc  <= 9'd0;
	end else begin
		div <= tick ? 5'd0 : div + 5'd1;
		if (tick) begin
			if (hc == H_TOTAL - 1) begin
				hc <= 9'd0;
				vc <= (vc == V_TOTAL - 1) ? 9'd0 : vc + 9'd1;
			end else begin
				hc <= hc + 9'd1;
			end
		end
	end
end

wire vbl_start = tick && hc == H_TOTAL - 1 && vc == 9'd127;

// ---------------------------------------------------------------- state

reg  [7:0] pal    [0:15];
reg  [7:0] secpal [0:15];
reg [127:0] rbits;
reg  [7:0] mode, raster, sbase;

// ---------------------------------------------------------------- copy

reg        pending, copying, done_req;
reg [11:0] idx;        // 0-31: 0x5f00 block, 32-2079: screen
reg        rd_v;
reg [11:0] rd_idx;
wire       last_req = idx == 12'd2079;

assign cp_req = copying && !done_req;

always @* begin
	if (idx < 12'd32) cp_addr = 14'h17c0 + {9'd0, idx[4:0]};
	else              cp_addr = {sbase, 6'd0} + {3'd0, idx[10:0] - 11'd32};
end

wire [31:0] w = cp_rdata;
wire        buf_we = rd_v && rd_idx >= 12'd32;
wire [10:0] buf_wa = rd_idx[10:0] - 11'd32;

integer i;
always @(posedge clk) begin
	if (reset) begin
		pending  <= 1'b0;
		copying  <= 1'b0;
		done_req <= 1'b0;
		rd_v     <= 1'b0;
		vcount   <= 32'd0;
		mode     <= 8'd0;
		raster   <= 8'd0;
		sbase    <= 8'h60;
		rbits    <= 128'd0;
		for (i = 0; i < 16; i = i + 1) begin
			pal[i]    <= i[7:0];
			secpal[i] <= i[7:0];
		end
	end else begin
		if (present) pending <= 1'b1;

		if (vbl_start) begin
			if (pending || present) begin
				pending  <= 1'b0;
				copying  <= 1'b1;
				done_req <= 1'b0;
				idx      <= 12'd0;
			end else begin
				vcount <= vcount + 32'd1;
			end
		end

		rd_v <= copying && cp_req && cp_gnt;
		rd_idx <= idx;
		if (copying && cp_req && cp_gnt) begin
			if (last_req) done_req <= 1'b1;
			else idx <= idx + 12'd1;
		end

		if (rd_v && rd_idx < 12'd32) begin
			case (rd_idx[4:0])
				5'd4, 5'd5, 5'd6, 5'd7: begin
					pal[{rd_idx[1:0], 2'd0}] <= w[7:0];
					pal[{rd_idx[1:0], 2'd1}] <= w[15:8];
					pal[{rd_idx[1:0], 2'd2}] <= w[23:16];
					pal[{rd_idx[1:0], 2'd3}] <= w[31:24];
				end
				5'd11: mode   <= w[7:0];      // 0x5f2c
				5'd21: sbase  <= w[15:8];     // 0x5f55
				5'd23: raster <= w[31:24];    // 0x5f5f
				5'd24, 5'd25, 5'd26, 5'd27: begin
					secpal[{rd_idx[1:0], 2'd0}] <= w[7:0];
					secpal[{rd_idx[1:0], 2'd1}] <= w[15:8];
					secpal[{rd_idx[1:0], 2'd2}] <= w[23:16];
					secpal[{rd_idx[1:0], 2'd3}] <= w[31:24];
				end
				5'd28: rbits[31:0]   <= w;
				5'd29: rbits[63:32]  <= w;
				5'd30: rbits[95:64]  <= w;
				5'd31: rbits[127:96] <= w;
				default: ;
			endcase
		end

		if (rd_v && rd_idx == 12'd2079) begin
			copying <= 1'b0;
			vcount  <= vcount + 32'd1;
		end
	end
end

// ---------------------------------------------------------------- fps

// presents per second, counted in BCD (saturates at 99)
reg  [7:0] fps_cnt, fps;
reg [26:0] sec;
wire       presented = rd_v && rd_idx == 12'd2079;

always @(posedge clk) begin
	if (reset) begin
		sec     <= 27'd0;
		fps_cnt <= 8'd0;
		fps     <= 8'd0;
	end else if (sec == CLK_HZ - 1) begin
		sec     <= 27'd0;
		fps     <= fps_cnt;
		fps_cnt <= {7'd0, presented};
	end else begin
		sec <= sec + 27'd1;
		if (presented && fps_cnt != 8'h99)
			fps_cnt <= fps_cnt[3:0] == 4'd9 ? {fps_cnt[7:4] + 4'd1, 4'd0} : fps_cnt + 8'd1;
	end
end

// ---------------------------------------------------------------- spinner

reg       spin_on;
reg [6:0] spin_x, spin_y;
reg [4:0] spin_t;      // vblanks; the head moves every 4

always @(posedge clk) begin
	if (reset) begin
		spin_on <= 1'b0;
		spin_t  <= 5'd0;
	end else begin
		if (spin_wr) begin
			spin_on <= spin_data[15];
			spin_y  <= spin_data[14:8];
			spin_x  <= spin_data[6:0];
		end else if (presented) begin
			spin_on <= 1'b0;
		end
		if (vbl_start) spin_t <= spin_t + 5'd1;
	end
end

// dots at the even offsets of the box's border, clockwise from the top
// middle: ring position 0-7
wire [8:0] sp_dx = hc - {2'd0, spin_x};
wire [8:0] sp_dy = vc - {2'd0, spin_y};
reg  [3:0] sp_k;      // 8: no dot here
always @* begin
	case ({sp_dx[2:0], sp_dy[2:0]})
		6'o20: sp_k = 4'd0;
		6'o40: sp_k = 4'd1;
		6'o42: sp_k = 4'd2;
		6'o44: sp_k = 4'd3;
		6'o24: sp_k = 4'd4;
		6'o04: sp_k = 4'd5;
		6'o02: sp_k = 4'd6;
		6'o00: sp_k = 4'd7;
		default: sp_k = 4'd8;
	endcase
end
wire       sp_dot = spin_on && sp_dx < 9'd5 && sp_dy < 9'd5 && !sp_k[3];
wire [2:0] sp_age = spin_t[4:2] - sp_k[2:0];
wire [23:0] sp_rgb = sp_age == 3'd0 ? 24'hfff1e8 : sp_age == 3'd1 ? 24'hc2c3c7 :
                     sp_age == 3'd2 ? 24'h5f574f : 24'h1d2b53;

// 3x5 digits, row 0 in the top bits, left column in the MSB of a row
function [14:0] glyph(input [3:0] d);
	case (d)
		4'd0: glyph = 15'b111_101_101_101_111;
		4'd1: glyph = 15'b110_010_010_010_111;
		4'd2: glyph = 15'b111_001_111_100_111;
		4'd3: glyph = 15'b111_001_011_001_111;
		4'd4: glyph = 15'b101_101_111_001_001;
		4'd5: glyph = 15'b111_100_111_001_111;
		4'd6: glyph = 15'b100_111_101_101_111;
		4'd7: glyph = 15'b111_001_001_001_001;
		4'd8: glyph = 15'b111_101_111_101_111;
		default: glyph = 15'b111_101_111_001_111;
	endcase
endfunction

// box at (0,0)-(8,6): digits at x 1 and 5, y 1
wire       ov_box   = show_fps && hc < 9'd9 && vc < 9'd7;
wire [3:0] ov_digit = hc < 9'd5 ? fps[7:4] : fps[3:0];
wire [2:0] ov_col   = hc < 9'd5 ? hc[2:0] - 3'd1 : hc[2:0] - 3'd5;
wire [2:0] ov_row   = vc[2:0] - 3'd1;
wire [3:0] ov_bit   = {1'b0, ov_row} + {ov_row, 1'b0} + {1'b0, ov_col};  // row * 3 + col
wire [14:0] ov_g    = glyph(ov_digit);
wire       ov_pix   = vc >= 9'd1 && vc <= 9'd5 && hc != 9'd0 && hc != 9'd4 && hc != 9'd8 &&
                      ov_g[4'd14 - ov_bit];

// ---------------------------------------------------------------- scanout

// screen mode mapping (as p8_gfx::render mode_map)
wire [6:0] x = hc[6:0];
wire [6:0] y = vc[6:0];
reg  [6:0] sx, sy, xx, yy;
wire [7:0] mode1 = mode + 8'd1;
always @* begin
	xx = x;
	yy = y;
	if ((mode & 8'hbc) == 8'h84) begin
		xx = mode[0] ? y : x;
		yy = mode[0] ? x : y;
		sx = mode[1] ? ~xx : xx;
		sy = mode1[1] ? ~yy : yy;
	end else begin
		case (mode & 8'hbd)
			8'h05:   sx = x[6] ? ~x : x;
			8'h01:   sx = {1'b0, x[6:1]};
			8'h81:   sx = ~x;
			default: sx = x;
		endcase
		case (mode & 8'hbe)
			8'h06:   sy = y[6] ? ~y : y;
			8'h02:   sy = {1'b0, y[6:1]};
			8'h82:   sy = ~y;
			default: sy = y;
		endcase
	end
end

wire [31:0] buf_q;
`ifndef P8_SPLASH
`define P8_SPLASH "rtl/splash.hex"
`endif
p8_bram #(.AW(11), .DW(32), .INIT(`P8_SPLASH)) dispbuf (
	.clk(clk), .re(1'b1), .raddr({sy, sx[6:3]}), .rdata(buf_q), .rdata_raw(),
	.we(buf_we), .waddr(buf_wa), .wdata(w)
);

reg  [2:0] s1_x;
reg  [6:0] s1_y, s2_y;
reg  [3:0] s2_c;
reg  [7:0] s3_p;
reg [23:0] rgb;

wire [3:0] c = buf_q[{s1_x, 2'b00} +: 4];
wire       rb = rbits[s2_y];
wire [3:0] c2 = s2_y[6:3] + {3'd0, rb};

function [23:0] p8rgb(input [4:0] i);
	case (i)
		5'd0:  p8rgb = 24'h000000; 5'd1:  p8rgb = 24'h1d2b53;
		5'd2:  p8rgb = 24'h7e2553; 5'd3:  p8rgb = 24'h008751;
		5'd4:  p8rgb = 24'hab5236; 5'd5:  p8rgb = 24'h5f574f;
		5'd6:  p8rgb = 24'hc2c3c7; 5'd7:  p8rgb = 24'hfff1e8;
		5'd8:  p8rgb = 24'hff004d; 5'd9:  p8rgb = 24'hffa300;
		5'd10: p8rgb = 24'hffec27; 5'd11: p8rgb = 24'h00e436;
		5'd12: p8rgb = 24'h29adff; 5'd13: p8rgb = 24'h83769c;
		5'd14: p8rgb = 24'hff77a8; 5'd15: p8rgb = 24'hffccaa;
		5'd16: p8rgb = 24'h291814; 5'd17: p8rgb = 24'h111d35;
		5'd18: p8rgb = 24'h422136; 5'd19: p8rgb = 24'h125359;
		5'd20: p8rgb = 24'h742f29; 5'd21: p8rgb = 24'h49333b;
		5'd22: p8rgb = 24'ha28879; 5'd23: p8rgb = 24'hf3ef7d;
		5'd24: p8rgb = 24'hbe1250; 5'd25: p8rgb = 24'hff6c24;
		5'd26: p8rgb = 24'ha8e72e; 5'd27: p8rgb = 24'h00b543;
		5'd28: p8rgb = 24'h065ab5; 5'd29: p8rgb = 24'h754665;
		5'd30: p8rgb = 24'hff6e59; default: p8rgb = 24'hff9d81;
	endcase
endfunction

always @(posedge clk) begin
	s1_x <= sx[2:0];
	s1_y <= sy;
	s2_c <= c;
	s2_y <= s1_y;
	if (raster == 8'h10 && rb)
		s3_p <= secpal[s2_c];
	else if ((raster & 8'h30) == 8'h30 && raster[3:0] == s2_c)
		s3_p <= secpal[c2];
	else
		s3_p <= pal[s2_c];
	rgb <= p8rgb({s3_p[7], s3_p[3:0]});
end

// outputs, one pixel behind the counters
always @(posedge clk) begin
	ce_pix <= tick;
	if (tick) begin
		hblank <= hc >= 9'd128;
		vblank <= vc >= 9'd128;
		hs     <= hc >= H_SYNC && hc < H_SYNC_END;
		vs     <= vc >= V_SYNC && vc < V_SYNC_END;
		{r, g, b} <= ov_box ? (ov_pix ? 24'hfff1e8 : 24'h000000) :
		             sp_dot ? sp_rgb :
		             (hc < 9'd128 && vc < 9'd128) ? rgb : 24'd0;
	end
end

endmodule
