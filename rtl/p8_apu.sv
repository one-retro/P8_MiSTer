// Audio unit: the per-sample work of the audio (oscillators, noise, the
// second detuned wave, volume, reverb / dampen / distort and the mix of the
// four channels). The firmware runs the sequencer and queues operations
// block by block; this module executes them one sample step at a time.
//
// Registers (MMIO 0x200-0x21f, word `wreg`):
//   0 ID    R: "APU" << 8 | version (decoded in p8_soc)
//   1 CTRL  W: queue an operation with the registers below: 4-0 samples
//              - 1, 9-8 op (0 RUN, 1 PUSH, 2 READ, 3 RESET)
//           R: bit 0 busy, bit 1 the queue is full
//   2 INC   oscillator phase step per sample (one cycle = 1 << 24)
//   3 INC2  the detune wave's phase step
//   4 VOICE 2-0 wave, 5-3 detune wave, 6 buzz, 7 detune, 9-8 noise kind
//           (0 pitched, 1 white, 2 brown), 18-10 volume (0-256), 19 the
//           channel's second oscillator, 20 voice on
//   5 NOISE pitched noise: 12-0 lowpass coefficient (Q12), 27-16 gain (Q8)
//   6 FX    5-0 effects (reverb 1, reverb 2, dampen 1, dampen 2, distort,
//           distort high), 6 effects on, 9-8 channel, 17-10 / 20-18 dampen
//           1 lowpass coefficient (Q8) / shift of the rest kept (0: none),
//           28-21 / 31-29 dampen 2
//   7 ACC   R: the next sample READ took; W: advance
//
// RUN n: the channel's oscillator and effects for n samples, added to the
// accumulator (32 samples). PUSH n: accumulator samples 0..n-1, clamped,
// to the audio FIFO, and cleared. READ: all 32, clamped, to the ACC
// output, and cleared. RESET: everything to its initial state (also after
// `reset`).
//
// The firmware's model, which defines every step and the cycles each
// operation takes (bit-exact), is the source of truth for the arithmetic:
// all values are two's complement, shifts of signed values are arithmetic,
// and divisions round towards 0.

module p8_apu (
	input             clk,
	input             reset,
	input             wr,          // MMIO write to 0x200-0x21f
	input       [2:0] wreg,
	input      [31:0] wdata,
	output      [1:0] status,      // {queue full, busy}
	output     [15:0] acc_q,
	output            push,        // a sample for the audio FIFO
	output     [15:0] push_data,
	input             push_ok      // the FIFO takes it this cycle
);

localparam [31:0] RNG_SEED = 32'h2545f491;
localparam [9:0]  RV1 = 10'd366, RV2 = 10'd732;

function automatic signed [15:0] clamp16(input signed [39:0] x);
	clamp16 = x > 40'sd32767 ? 16'sd32767 : x < -40'sd32768 ? -16'sd32768 : x[15:0];
endfunction

function automatic [31:0] xorshift(input [31:0] r0);
	reg [31:0] r1, r2;
	begin
		r1 = r0 ^ (r0 << 13);
		r2 = r1 ^ (r1 >> 17);
		xorshift = r2 ^ (r2 << 5);
	end
endfunction

// ---------------------------------------------------------------- registers

// writes are registered first (the CPU's M stage sees one store each)
reg        w_q;
reg  [2:0] wreg_q;
reg [31:0] wdata_q;
always @(posedge clk) begin
	w_q     <= wr && !reset;
	wreg_q  <= wreg;
	wdata_q <= wdata;
end
wire ctrl_wr = w_q && wreg_q == 3'd1;
wire acc_wr  = w_q && wreg_q == 3'd7;

reg [31:0] r_inc, r_inc2;
reg [20:0] r_voice;
reg [27:0] r_noise;
reg [31:0] r_fx;
always @(posedge clk) begin
	if (w_q) case (wreg_q)
		3'd2: r_inc   <= wdata_q;
		3'd3: r_inc2  <= wdata_q;
		3'd4: r_voice <= wdata_q[20:0];
		3'd5: r_noise <= wdata_q[27:0];
		3'd6: r_fx    <= wdata_q;
		default: ;
	endcase
end

// ---------------------------------------------------------------- queue

localparam QW = 10 + 32 + 32 + 21 + 28 + 32;
reg  [7:0] q_wp, q_rp;
wire [QW-1:0] q_rdata;
p8_bram #(.AW(8), .DW(QW), .BYPASS(0)) queue (
	.clk(clk), .re(1'b1), .raddr(q_rp), .rdata(), .rdata_raw(q_rdata),
	.we(ctrl_wr), .waddr(q_wp), .wdata({wdata_q[9:0], r_inc, r_inc2, r_voice, r_noise, r_fx})
);
wire [7:0] q_count = q_wp - q_rp;

// ---------------------------------------------------------------- state

localparam [5:0]
	S_IDLE = 6'd0,  S_FETCH = 6'd1, S_SETUP = 6'd2, S_SMP = 6'd3,
	S_E1 = 6'd4, S_E2 = 6'd5, S_E3 = 6'd6, S_E4 = 6'd7, S_E5 = 6'd8,
	S_E6 = 6'd9, S_E7 = 6'd10, S_E8 = 6'd11, S_E9 = 6'd12,
	S_N1 = 6'd13, S_N2 = 6'd14,
	S_MX1 = 6'd15, S_MX2 = 6'd16, S_MX3 = 6'd17,
	S_F1 = 6'd18, S_F2 = 6'd19, S_F3 = 6'd20, S_F4 = 6'd21, S_F5 = 6'd22, S_F6 = 6'd23,
	S_F7 = 6'd24, S_F8 = 6'd25, S_F9 = 6'd26, S_F10 = 6'd27, S_F11 = 6'd28, S_F12 = 6'd29,
	S_A1 = 6'd30, S_WB = 6'd31,
	S_P1 = 6'd32, S_P2 = 6'd33, S_R1 = 6'd34, S_R2 = 6'd35, S_RST = 6'd36, S_E6B = 6'd37, S_SETUP2 = 6'd38, S_F6B = 6'd39, S_F9B = 6'd40;

reg  [5:0] st;
reg [12:0] cnt;               // sample (k) / RESET counter

// the operation being executed
reg  [9:0] e_ctrl;
reg [31:0] e_inc, e_inc2;
reg [20:0] e_voice;
reg [27:0] e_noise;
reg [31:0] e_fx;
wire [4:0] e_last  = e_ctrl[4:0];
wire [2:0] v_wave  = e_voice[2:0];
wire [2:0] v_wave2 = e_voice[5:3];
wire       v_buzz  = e_voice[6];
wire       v_dt    = e_voice[7] && v_wave != 3'd6;
wire [1:0] v_noise = e_voice[9:8];
wire [8:0] v_vol   = e_voice[18:10];
wire       v_on    = e_voice[20];
wire       osc_on  = v_on && v_vol != 9'd0;
wire [2:0] slot    = {e_fx[9:8], e_voice[19]};
wire [1:0] ch      = e_fx[9:8];
wire [5:0] fx_f    = e_fx[5:0];
wire       fx_on   = e_fx[6];
wire [7:0] d1_a    = e_fx[17:10];
wire [2:0] d1_g    = e_fx[20:18];
wire [7:0] d2_a    = e_fx[28:21];
wire [2:0] d2_g    = e_fx[31:29];

reg        [31:0] rng;

// oscillators (two per channel: phase, detune phase, noise state) and the
// channels' effects (line position, the shelves), in RAM
reg         os_we, fs_we;
reg   [2:0] os_wa;
reg   [1:0] fs_wa;
reg  [83:0] os_wd;
reg  [89:0] fs_wd;
wire [83:0] os_q;
wire [89:0] fs_q;
p8_bram #(.AW(3), .DW(84), .BYPASS(0)) osc_ram (
	.clk(clk), .re(1'b1), .raddr(slot), .rdata(), .rdata_raw(os_q),
	.we(os_we), .waddr(os_wa), .wdata(os_wd)
);
p8_bram #(.AW(2), .DW(90), .BYPASS(0)) fx_ram (
	.clk(clk), .re(1'b1), .raddr(ch), .rdata(), .rdata_raw(fs_q),
	.we(fs_we), .waddr(fs_wa), .wdata(fs_wd)
);

// working copies for the RUN
reg        [31:0] ph, ph2;
reg signed [19:0] s, s_new;
reg         [9:0] pos;
reg signed [19:0] l1, x1, l2, x2;
reg        [31:0] rng_n;

// delay lines: 1024 samples per channel (732 used)
wire [11:0] l_ra;
wire [15:0] l_q;
reg         l_we;
reg  [11:0] l_wa;
reg  [15:0] l_wd;
p8_bram #(.AW(12), .DW(16), .BYPASS(0)) line (
	.clk(clk), .re(1'b1), .raddr(l_ra), .rdata(), .rdata_raw(l_q),
	.we(l_we), .waddr(l_wa), .wdata(l_wd)
);

// accumulator
wire [31:0] a_q;
reg         a_we;
reg   [4:0] a_wa;
reg  [31:0] a_wd;
p8_bram #(.AW(5), .DW(32), .BYPASS(1)) acc (
	.clk(clk), .re(1'b1), .raddr(cnt[4:0]), .rdata(a_q), .rdata_raw(),
	.we(a_we), .waddr(a_wa), .wdata(a_wd)
);

// READ's output and the ACC read pointer (32: past the end, reads 0)
reg         ob_we;
reg   [4:0] ob_wa;
reg  [15:0] ob_wd;
wire [15:0] ob_q;
reg   [5:0] orp;
wire  [5:0] orp_n = acc_wr && !orp[5] ? orp + 6'd1 : orp;
p8_bram #(.AW(5), .DW(16), .BYPASS(0)) out_ram (
	.clk(clk), .re(1'b1), .raddr(orp_n[4:0]), .rdata(), .rdata_raw(ob_q),
	.we(ob_we), .waddr(ob_wa), .wdata(ob_wd)
);
assign acc_q = orp[5] ? 16'd0 : ob_q;

assign status    = {q_count >= 8'd253, st != S_IDLE || q_count != 8'd0 || ctrl_wr};
assign push      = st == S_P2;
assign push_data = clamp16({{8{a_q[31]}}, a_q});

// ---------------------------------------------------------------- waveform pipeline

// E1: from the phase
reg        [31:0] ep;          // phase being evaluated
reg         [2:0] ew;          // its wave
reg               second;      // evaluating the detune wave
wire       [15:0] tw = ep[23:8];
reg        [15:0] et;
reg signed [20:0] a_tri, a_org, a_8t, a_k, a_f1, a_f2;
reg               sawoff, lt32, lt_a, lt_sq;
reg        [16:0] dn;
// E2
reg signed [20:0] w_tri, org_r, phb;
reg        [15:0] u;
reg signed [23:0] dnm;
// E3
reg signed [20:0] a_u;
reg signed [22:0] orb;
// E4
reg signed [22:0] ph_r;
reg               orb_gt;
// E5 / N2: first multiply's operands and what follows it
reg signed [23:0] m1;
reg        [16:0] c1;
reg         [4:0] s1;
reg         [2:0] pm;          // post-multiply step
// E6: product; E6B: shifted
reg signed [41:0] prod1;
reg signed [24:0] q1;
// E7: second multiply
reg signed [23:0] m2;
reg        [16:0] c2;
reg         [4:0] s2;
// E8, E9
reg signed [41:0] prod2;
reg signed [23:0] y, ya;
// MX
reg signed [24:0] yy;
reg signed [34:0] prodv;
reg signed [23:0] xs;          // the oscillator's sample
// effects
reg signed [24:0] fv;
reg signed [19:0] vc, vd, ve, d1, d2, dq;
reg signed [29:0] dp;          // dampen: difference times coefficient
reg signed [15:0] li;
reg signed [31:0] fout;        // to the accumulator

localparam [2:0] PM_Q = 3'd0, PM_TILT = 3'd1, PM_TILT3 = 3'd2, PM_SAW = 3'd3,
                 PM_SUB64 = 3'd4, PM_ADD32 = 3'd5, PM_NOISE = 3'd6;

function automatic signed [20:0] abs21(input signed [21:0] x);
	abs21 = x < 0 ? -x : x;
endfunction

wire [9:0]  jpos = pos >= RV1 ? pos - RV1 : pos + RV1;
assign l_ra = st == S_F1 ? {ch, pos} : {ch, jpos};

wire signed [16:0] white = $signed(rng_n) >>> 15;

always @(posedge clk) begin
	l_we  <= 1'b0;
	a_we  <= 1'b0;
	os_we <= 1'b0;
	fs_we <= 1'b0;
	ob_we <= 1'b0;
	orp   <= orp_n;
	if (ctrl_wr) q_wp <= q_wp + 8'd1;

	case (st)
	S_IDLE: if (q_count != 8'd0) st <= S_FETCH;
	S_FETCH: begin
		{e_ctrl, e_inc, e_inc2, e_voice, e_noise, e_fx} <= q_rdata;
		q_rp <= q_rp + 8'd1;
		cnt  <= 13'd0;
		case (q_rdata[QW-1 -: 10] >> 8)
			2'd0: st <= S_SETUP;
			2'd1: st <= S_P1;
			2'd2: begin st <= S_R1; orp <= 6'd0; end
			default: st <= S_RST;
		endcase
	end

	// ------------------------------------------------------------ RUN
	// the state RAMs read the slot and channel
	S_SETUP: st <= S_SETUP2;
	S_SETUP2: begin
		{ph, ph2, s} <= os_q;
		pos <= fs_q[89:80];
		// a shelf that is off starts from 0 when it comes on
		l1 <= fx_f[2] ? $signed(fs_q[79:60]) : 20'sd0;
		x1 <= fx_f[2] ? $signed(fs_q[59:40]) : 20'sd0;
		l2 <= fx_f[3] ? $signed(fs_q[39:20]) : 20'sd0;
		x2 <= fx_f[3] ? $signed(fs_q[19:0]) : 20'sd0;
		st <= S_SMP;
	end
	S_SMP: begin
		second <= 1'b0;
		ep <= ph;
		ew <= v_wave;
		xs <= 24'sd0;
		if (osc_on) st <= v_wave == 3'd6 ? S_N1 : S_E1;
		else begin
			if (v_on) ph <= ph + e_inc;
			st <= fx_on ? S_F1 : S_A1;
		end
	end
	S_E1: begin
		et     <= tw;
		a_tri  <= abs21($signed({4'd0, tw, 2'd0}) - 22'sd131072);
		a_org  <= abs21(tw[15] ? $signed({2'd0, tw, 4'd0}) - 22'sd786432
		                       : $signed({2'd0, tw, 4'd0}) + $signed({3'd0, tw, 3'd0}) - 22'sd393216);
		a_8t   <= abs21($signed({3'd0, tw, 3'd0}) - 22'sd262144);
		a_k    <= abs21($signed({5'd0, ep[30:15], 1'b0}) - 22'sd65536);
		a_f1   <= abs21($signed({6'd0, ep[22] ^ 1'b1, ep[21:7]}) - 22'sd32768);
		a_f2   <= abs21($signed({7'd0, ep[21:7]}) - 22'sd16384);
		sawoff <= ep[24:8] > 17'd32768 && ep[24:8] < 17'd98304;
		lt32   <= !tw[15];
		lt_a   <= tw < (ew == 3'd0 ? 16'd57344 : v_buzz ? 16'd63898 : 16'd58982);
		lt_sq  <= tw < (ew == 3'd3 ? (v_buzz ? 16'd26214 : 16'd32768) : (v_buzz ? 16'd16712 : 16'd21845));
		dn     <= 17'd65536 - {1'b0, tw};
		st <= S_E2;
	end
	S_E2: begin
		w_tri   <= a_tri - 21'sd65536;
		org_r <= (lt32 ? 21'sd196608 : 21'sd65536) - a_org;
		u     <= et + a_k[16:1];
		phb   <= 21'sd24576 - a_f1 - a_f2;
		dnm   <= ew == 3'd0 ? $signed({3'd0, dn, 4'd0})
		       : v_buzz ? $signed({1'd0, dn, 6'd0}) + $signed({3'd0, dn, 4'd0})
		       : $signed({3'd0, dn, 4'd0}) + $signed({5'd0, dn, 2'd0});
		st <= S_E3;
	end
	S_E3: begin
		a_u <= abs21($signed({4'd0, u, 2'd0}) - 22'sd131072);
		orb <= {org_r[20], org_r, 1'b0} + 23'sd196608;
		st <= S_E4;
	end
	S_E4: begin
		ph_r   <= a_u - a_8t + (v_buzz ? phb : 21'sd0);
		orb_gt <= orb > -23'sd122880;
		st <= S_E5;
	end
	S_E5: begin
		c1 <= 17'd1; s1 <= 5'd0; pm <= PM_Q;
		c2 <= 17'd1; s2 <= 5'd0;
		case (ew)
		3'd0: if (v_buzz) begin
			m1 <= lt_a ? $signed({8'd0, et}) : 24'sd0; c1 <= 17'd37449; s1 <= 5'd14; pm <= PM_TILT3; s2 <= 5'd3;
		end else begin
			m1 <= w_tri; s1 <= 5'd1;
		end
		3'd1: begin
			m1 <= lt_a ? $signed({8'd0, et}) : 24'sd0; c1 <= v_buzz ? 17'd33608 : 17'd36409; s1 <= 5'd14;
			pm <= PM_TILT; s2 <= 5'd1;
		end
		3'd2: begin
			m1 <= $signed(et); pm <= v_buzz ? PM_SAW : PM_Q; c2 <= 17'd42795; s2 <= 5'd16;
			if (v_buzz) begin c1 <= 17'd54395; s1 <= 5'd16; end
		end
		3'd3, 3'd4: m1 <= lt_sq ? 24'sd16384 : -24'sd16384;
		3'd5: begin
			c2 <= 17'd7282; s2 <= 5'd16;
			if (!v_buzz) m1 <= org_r;
			else if (!lt32) begin m1 <= org_r; pm <= PM_ADD32; end
			else if (orb_gt) begin m1 <= orb; c1 <= 17'd3277; s1 <= 5'd14; pm <= PM_SUB64; end
			else begin m1 <= orb; pm <= PM_ADD32; end
		end
		default: begin m1 <= ph_r; c2 <= 17'd5461; s2 <= 5'd15; end
		endcase
		st <= S_E6;
	end
	S_N1: begin
		rng_n <= xorshift(rng);
		st <= S_N2;
	end
	S_N2: begin
		pm <= PM_NOISE;
		c1 <= 17'd1; s1 <= 5'd0; s2 <= 5'd0;
		case (v_noise)
		2'd0: begin m1 <= white - s; c1 <= {4'd0, e_noise[12:0]}; s1 <= 5'd12; c2 <= {5'd0, e_noise[27:16]}; s2 <= 5'd8; end
		2'd1: begin m1 <= white; c2 <= 17'd1; end
		default: begin m1 <= white - s; s1 <= 5'd4; c2 <= 17'd4; end
		endcase
		st <= S_E6;
	end
	S_E6: begin
		prod1 <= m1 * $signed({1'b0, c1});
		st <= S_E6B;
	end
	S_E6B: begin
		q1 <= prod1 >>> s1;
		st <= S_E7;
	end
	S_E7: begin
		case (pm)
		PM_TILT:  m2 <= lt_a ? q1 - 25'sd65536 : dnm - 24'sd65536;
		PM_TILT3: m2 <= 3 * w_tri - (lt_a ? q1 - 25'sd65536 : dnm - 24'sd65536);
		PM_SAW:   m2 <= q1 - (sawoff ? 25'sd5571 : 25'sd0);
		PM_SUB64: m2 <= q1 - 25'sd65536;
		PM_ADD32: m2 <= q1 + 25'sd32768;
		PM_NOISE: begin
			m2    <= v_noise == 2'd1 ? q1 : s + q1;
			s_new <= v_noise == 2'd1 ? q1 : s + q1;
		end
		default:  m2 <= q1;
		endcase
		st <= S_E8;
	end
	S_E8: begin
		prod2 <= m2 * $signed({1'b0, c2});
		st <= S_E9;
	end
	S_E9: begin
		y <= prod2 >>> s2;
		if (v_dt && !second) begin
			ya <= prod2 >>> s2;
			second <= 1'b1;
			ep <= ph2;
			ew <= v_wave2;
			st <= S_E1;
		end else st <= S_MX1;
	end
	S_MX1: begin
		yy <= second ? ya + (y >>> 1) : y;
		st <= S_MX2;
	end
	S_MX2: begin
		prodv <= yy * $signed({1'b0, v_vol});
		st <= S_MX3;
	end
	S_MX3: begin
		xs <= v_wave == 3'd6 ? prodv >>> 9 : clamp16(prodv >>> 9);
		ph <= ph + e_inc;
		if (v_dt) ph2 <= ph2 + e_inc2;
		if (v_wave == 3'd6) begin s <= s_new; rng <= rng_n; end
		st <= fx_on ? S_F1 : S_A1;
	end
	// effects: the line at pos (4 ticks back) is read in F1, at jpos (2
	// ticks back) in F2
	S_F1: st <= S_F2;
	S_F2: begin
		li <= l_q;
		st <= S_F3;
	end
	S_F3: begin
		fv <= xs + (fx_f[0] ? $signed(l_q) >>> 1 : 16'sd0) + (fx_f[1] ? li >>> 1 : 16'sd0);
		st <= S_F4;
	end
	S_F4: begin
		vc   <= clamp16(fv);
		l_we <= 1'b1;
		l_wa <= {ch, pos};
		l_wd <= clamp16(fv);
		st <= S_F5;
	end
	S_F5: begin
		d1 <= ((vc + x1) >>> 1) - l1;
		st <= S_F6;
	end
	S_F6: begin
		dp <= d1 * $signed({1'b0, d1_a});
		st <= S_F6B;
	end
	S_F6B: begin
		if (fx_f[2]) begin
			l1 <= l1 + (dp >>> 8);
			x1 <= vc;
		end
		st <= S_F7;
	end
	S_F7: begin
		vd <= !fx_f[2] ? vc : d1_g == 3'd0 ? l1 : l1 + ((vc - l1) >>> d1_g);
		st <= S_F8;
	end
	S_F8: begin
		d2 <= ((vd + x2) >>> 1) - l2;
		st <= S_F9;
	end
	S_F9: begin
		dp <= d2 * $signed({1'b0, d2_a});
		st <= S_F9B;
	end
	S_F9B: begin
		if (fx_f[3]) begin
			l2 <= l2 + (dp >>> 8);
			x2 <= vd;
		end
		st <= S_F10;
	end
	S_F10: begin
		ve <= !fx_f[3] ? vd : d2_g == 3'd0 ? l2 : l2 + ((vd - l2) >>> d2_g);
		st <= S_F11;
	end
	S_F11: begin
		// v / 0x1000 rounding to 0 (distort high: of v - 0x1000 if v < 0)
		if (!fx_f[4] && ve < 0)
			dq <= ((ve - 20'sd4096) >>> 12) + ((ve[11:0] != 12'd0) ? 20'sd1 : 20'sd0);
		else
			dq <= (ve >>> 12) + ((ve < 0 && ve[11:0] != 12'd0) ? 20'sd1 : 20'sd0);
		st <= S_F12;
	end
	S_F12: begin
		fout <= fx_f[5:4] != 2'd0 ? dq * 32'sd4681 : ve;
		pos  <= pos == RV2 - 10'd1 ? 10'd0 : pos + 10'd1;
		st <= S_A1;
	end
	S_A1: begin
		a_we <= 1'b1;
		a_wa <= cnt[4:0];
		a_wd <= $signed(a_q) + (fx_on ? fout : xs);
		cnt  <= cnt + 13'd1;
		st <= cnt[4:0] == e_last ? S_WB : S_SMP;
	end
	S_WB: begin
		os_we <= v_on;
		os_wa <= slot;
		os_wd <= {ph, ph2, s};
		fs_we <= fx_on;
		fs_wa <= ch;
		fs_wd <= {pos, l1, x1, l2, x2};
		st <= S_IDLE;
	end

	// ------------------------------------------------------------ PUSH, READ
	S_P1: st <= S_P2;
	S_P2: if (push_ok) begin
		a_we <= 1'b1;
		a_wa <= cnt[4:0];
		a_wd <= 32'd0;
		cnt  <= cnt + 13'd1;
		st <= cnt[4:0] == e_last ? S_IDLE : S_P1;
	end
	S_R1: st <= S_R2;
	S_R2: begin
		ob_we <= 1'b1;
		ob_wa <= cnt[4:0];
		ob_wd <= clamp16({{8{a_q[31]}}, a_q});
		a_we <= 1'b1;
		a_wa <= cnt[4:0];
		a_wd <= 32'd0;
		cnt  <= cnt + 13'd1;
		st <= cnt[4:0] == 5'd31 ? S_IDLE : S_R1;
	end

	// ------------------------------------------------------------ RESET
	S_RST: begin
		l_we <= 1'b1;
		l_wa <= cnt[11:0];
		l_wd <= 16'd0;
		if (cnt < 13'd32) begin
			a_we <= 1'b1;
			a_wa <= cnt[4:0];
			a_wd <= 32'd0;
		end
		os_we <= cnt < 13'd8;  os_wa <= cnt[2:0]; os_wd <= 84'd0;
		fs_we <= cnt < 13'd4;  fs_wa <= cnt[1:0]; fs_wd <= 90'd0;
		ob_we <= cnt < 13'd32; ob_wa <= cnt[4:0]; ob_wd <= 16'd0;
		orp <= 6'd0;
		rng <= RNG_SEED;
		cnt <= cnt + 13'd1;
		if (cnt == 13'd4095) st <= S_IDLE;
	end
	default: st <= S_IDLE;
	endcase

	if (reset) begin
		st   <= S_RST;
		cnt  <= 13'd0;
		q_wp <= 8'd0;
		q_rp <= 8'd0;
	end
end

endmodule
