// Data cache: 32 KiB, 2-way set associative (LRU), write-back,
// write-allocate, 32-byte lines, 64-bit RAMs (one 8-bit RAM per byte lane
// and way).
//
// The RAMs are read with the E-stage address (with the M address instead
// while `m_hold`: a requester that waits in M with a fixed address, the
// table unit or p8.zline); the M-stage request (m_rd/m_wr) is answered in
// the next cycle: `ready` is high on a hit in either way (or when there is
// no request). A hit needs tags read in S_IDLE in the previous cycle
// (rd_ok); otherwise, and on a miss, `ready` is low and the requester
// asks again (the CPU replays the instruction, the table unit waits). A
// store hit writes its bytes in that cycle. On a miss the FSM picks a
// victim (an invalid way, else the least recently used one) from the tags
// read for the request (registered), and fills the line: the DDR request
// goes out with the miss, the read command as soon as the bus is granted.
// A dirty victim is first copied into the write-back buffer (wbuf); the
// fill goes first, and the buffer is written to DDR after it while the
// cache answers hits again (wb_pend). The next miss, a p8.zline or a
// flush waits for that write.
//
// The table unit (t_*, while t_act; the CPU then waits in M with no load
// or store) has its own tag compare, hit way and `t_ready`: the CPU's
// compare (late: load data way, replay, store enables) takes its address
// straight from the CPU's register, not through a requester select. The
// RAMs read t_addr; its answer (t_ready, the words) is for the address
// read in the previous cycle (t_aq), so the unit may present its next
// address while it checks a word.
//
// `m_zl` (p8.zline) zeroes the line at m_addr without reading memory: a
// miss copies a dirty victim into wbuf (written back after), then the line
// is written with zeros and marked dirty (S_ZERO); `zl_done` (and `ready`)
// pulse after that.
//
// `flush` (level) writes back every dirty line (lines stay valid, now
// clean); `flush_done` pulses at the end. Used for FENCE.I.

module p8_dcache (
	input             clk,
	input             reset,

	input      [13:0] e_addr,    // the E-stage address (bits 13-0)
	input      [31:0] m_addr,    // the CPU's M-stage request
	input             m_hold,
	input             m_rd,
	input             m_wr,
	input             m_zl,
	input       [3:0] m_be,
	input      [31:0] m_wdata,
	input             m_stv,     // p8.stv: 8 bytes, {m_tag, m_wdata} (m_be all)
	input       [2:0] m_tag,
	input             t_act,     // the table unit's request
	input      [31:0] t_addr,    // read now; answered next cycle
	input             t_rd,
	output     [31:0] rdata0,    // the addressed word in each way; the
	output     [31:0] rdata1,    // load data is rhit1 ? rdata1 : rdata0
	output            rhit1,
	output     [63:0] rdata64_0, // the whole 64-bit word of each way (for
	output     [63:0] rdata64_1, // p8_tunit, which selects by t_rhit1 last;
	                             // without the RAMs' write bypass: no store
	                             // hits while the table unit runs)
	output            t_rhit1,
	output            ready,     // the CPU's request
	output            t_ready,   // the table unit's
	output reg        zl_done,   // p8.zline finished (one cycle)

	input             flush,
	output reg        flush_done,
	// performance events: {busy, write-back, fill cycle, line filled}
	output reg  [3:0] pm,

	// DDR client (see p8_ddr_arb)
	output reg        mem_req,
	input             mem_gnt,
	output reg        mem_rd,
	output reg        mem_we,
	output     [21:0] mem_addr,
	output     [63:0] mem_wdata,
	input             mem_busy,
	input      [63:0] mem_rdata,
	input             mem_rvalid
);

localparam S_INIT    = 4'd0,
           S_IDLE    = 4'd1,
           S_TAG_CHK = 4'd3,
           S_WB_RD   = 4'd4,
           S_WB_WAIT = 4'd5,
           S_FILL_RQ = 4'd7,
           S_FILL    = 4'd8,
           S_FL_RD   = 4'd10,
           S_FL_CHK  = 4'd11,
           S_FL_NEXT = 4'd12,
           S_ZERO    = 4'd13;

reg  [3:0] st;
reg  [8:0] line;      // set being handled by the FSM
reg        way;       // way being handled by the FSM
reg [10:0] mtag;      // tag of the line to fill
reg [10:0] vtag;      // tag of the victim to write back
reg  [2:0] cnt;
reg  [1:0] beat;
reg        flushing;
// the line being written back, beat 0 in the low word: shifted in as it is
// read and out as the beats are taken, so the DDR write data is a register
reg [255:0] wbuf;
reg        wb_pend;   // wbuf holds a line to write back (to {vtag, line})
reg  [1:0] wbeat;     // its next beat
reg        zl;        // the miss being handled is a p8.zline

// ---------------------------------------------------------------- RAMs

// the request being served: the table unit's or the CPU's
reg  [31:0] t_aq;
always @(posedge clk) t_aq <= t_addr;
wire [31:0] r_addr = t_act ? t_addr : m_addr;   // read while m_hold
wire [31:0] a_addr = t_act ? t_aq : m_addr;     // the request answered

// read addresses of the tags (and LRU bit) and of the data lanes (one copy
// per way, below)
wire  [8:0] t_ra = st == S_IDLE ? (m_hold ? r_addr[13:5] : e_addr[13:5]) : line;

// per way: tag RAM {valid, dirty, tag[24:14]} and 8 data byte lanes
wire [12:0] tag_q  [0:1];
wire [63:0] data_q [0:1];   // without the RAMs' write bypass (see the store bypass)
reg         tag_we;
reg         tag_way;
reg   [8:0] tag_wa;
reg  [12:0] tag_wd;

// the RAM outputs are for the M-stage request (m_addr, or t_aq while the
// table unit runs): read in S_IDLE in the previous cycle
reg         rd_ok;
always @(posedge clk) rd_ok <= st == S_IDLE;

// hits of the CPU's request (_c) and the table unit's (_t), and of the
// request being served (the FSM and the LRU bit)
wire        c_acc  = m_rd || m_wr || m_zl;
wire        access = c_acc || t_rd;
wire        hit0_c = rd_ok && tag_q[0][12] && tag_q[0][10:0] == m_addr[24:14];
wire        hit1_c = rd_ok && tag_q[1][12] && tag_q[1][10:0] == m_addr[24:14];
wire        hit_c  = hit0_c || hit1_c;
wire        hit0_t = rd_ok && tag_q[0][12] && tag_q[0][10:0] == t_aq[24:14];
wire        hit1_t = rd_ok && tag_q[1][12] && tag_q[1][10:0] == t_aq[24:14];
wire        hit_t  = hit0_t || hit1_t;
wire        hit1   = t_act ? hit1_t : hit1_c;
wire        hit    = t_act ? hit_t : hit_c;
wire        idle   = st == S_IDLE;
wire        st_hit = idle && m_wr && hit_c;

assign ready   = idle && (!c_acc || (m_zl ? zl_done : hit_c));
assign t_ready = idle && (!t_rd || hit_t);

wire  [7:0] be8 = m_stv ? 8'hff : m_addr[2] ? {m_be, 4'b0000} : {4'b0000, m_be};
// the store's low word: p8.stv's tag, else the data (replicated in both)
wire [31:0] wd_lo = m_stv ? {29'd0, m_tag} : m_wdata;
wire        fill_we = st == S_FILL && mem_rvalid;
wire        zero_we = st == S_ZERO;

genvar gw, gi;
generate for (gw = 0; gw < 2; gw = gw + 1) begin : w
	p8_bram #(.AW(9), .DW(13)) tags (
		.clk(clk), .re(1'b1), .raddr(t_ra), .rdata(tag_q[gw]), .rdata_raw(),
		.we(tag_we && (st == S_INIT || tag_way == gw)), .waddr(tag_wa), .wdata(tag_wd)
	);
	wire sel = (fill_we || zero_we) ? way == gw : (gw ? hit1_c : !hit1_c);
	// the lanes' read address: kept once per way, so each select drives
	// its way's 16 RAM blocks (the late E address meets one LUT)
	(* keep = 1 *) wire [10:0] d_ra = st == S_IDLE ? (m_hold ? r_addr[13:3] : e_addr[13:3]) : {line, cnt[1:0]};
	for (gi = 0; gi < 8; gi = gi + 1) begin : lane
		p8_bram #(.AW(11), .DW(8), .BYPASS(0)) ram (
			.clk(clk), .re(1'b1), .raddr(d_ra), .rdata(data_q[gw][gi*8 +: 8]), .rdata_raw(),
			.we(sel && (fill_we || zero_we || (st_hit && be8[gi]))),
			.waddr(fill_we ? {line, beat} : zero_we ? {line, cnt[1:0]} : m_addr[13:3]),
			.wdata(fill_we ? mem_rdata[gi*8 +: 8] : zero_we ? 8'd0 : gi < 4 ? wd_lo[(gi%4)*8 +: 8] : m_wdata[(gi%4)*8 +: 8])
		);
	end
end endgenerate

// least recently used way per set
wire       lru_q;
reg        lru_we, lru_wd;
reg  [8:0] lru_wa;
p8_bram #(.AW(9), .DW(1)) lru (
	.clk(clk), .re(1'b1), .raddr(t_ra), .rdata(lru_q), .rdata_raw(),
	.we(lru_we), .waddr(lru_wa), .wdata(lru_wd)
);

// Store bypass: a store hit writes its bytes in the cycle the RAMs are read
// for the load behind it (in E); the load, now in M, takes those bytes from
// here. The RAMs are read raw, and the bytes are merged with the word
// select in one LUT per bit (a bypass in each byte lane's RAM would put
// a second one on the load data, and load the read address).
// (the store's row is compared with the row the RAMs read in its cycle,
// E's address, and the result registered: the load data meets no compare)
reg         sb_hit, sb_way;
reg   [7:0] sb_be;
reg  [31:0] sb_d, sb_lo;
always @(posedge clk) begin
	sb_hit <= st_hit && m_addr[13:3] == e_addr[13:3];
	sb_way <= hit1_c;
	sb_be  <= be8;
	sb_d   <= m_wdata;
	sb_lo  <= wd_lo;
end
// the bytes the load's word takes from the store (registers only)
wire [31:0] sb_w = m_addr[2] ? sb_d : sb_lo;
// the addressed word of a way, with the bytes the store wrote
function automatic [31:0] word_of(input [63:0] q, input way_i);
	reg [3:0] bs;
	integer j;
	begin
		bs = {4{sb_hit && sb_way == way_i}} & (m_addr[2] ? sb_be[7:4] : sb_be[3:0]);
		for (j = 0; j < 4; j = j + 1)
			word_of[8*j +: 8] = bs[j] ? sb_w[8*j +: 8] : m_addr[2] ? q[32 + 8*j +: 8] : q[8*j +: 8];
	end
endfunction
assign rdata0  = word_of(data_q[0], 1'b0);
assign rdata1  = word_of(data_q[1], 1'b1);
assign rhit1   = hit1_c;
assign t_rhit1 = hit1_t;
assign rdata64_0 = data_q[0];
assign rdata64_1 = data_q[1];

// the FSM's way (victim, line being zeroed or flushed)
wire [12:0] f_tag  = way ? tag_q[1] : tag_q[0];
wire [63:0] f_data = way ? data_q[1] : data_q[0];
// the tags and LRU bit read for the request, registered: on a miss they are
// the missing line's set (S_TAG_CHK picks the victim from them)
reg  [12:0] tq0, tq1;
reg         lq;
always @(posedge clk) begin
	tq0 <= tag_q[0];
	tq1 <= tag_q[1];
	lq  <= lru_q;
end
// victim: an invalid way, else the LRU one
wire        victim = !tq0[12] ? 1'b0 : !tq1[12] ? 1'b1 : lq;
wire [12:0] v_tag  = victim ? tq1 : tq0;

always @* begin
	tag_we  = 1'b0;
	tag_way = way;
	tag_wa  = line;
	tag_wd  = {2'b10, mtag};
	lru_we  = 1'b0;
	lru_wa  = line;
	lru_wd  = !way;
	case (st)
		S_INIT: begin
			tag_we = 1'b1;
			tag_wd = 13'd0;
			lru_we = 1'b1;
			lru_wd = 1'b0;
		end
		S_IDLE: begin
			// only the enable depends on the hit (late); a store is the CPU's
			tag_way = hit1_c;
			tag_wa  = m_addr[13:5];
			tag_wd  = {2'b11, m_addr[24:14]};
			tag_we  = st_hit && !(hit1_c ? tag_q[1][11] : tag_q[0][11]);
			if (access && !m_zl && hit) begin
				lru_we = 1'b1;
				lru_wa = a_addr[13:5];
				lru_wd = !hit1;
			end
		end
		S_FILL: begin
			tag_we = mem_rvalid && beat == 2'd3;
			lru_we = mem_rvalid && beat == 2'd3;
		end
		S_ZERO: if (cnt[1:0] == 2'd3) begin
			tag_we = 1'b1;
			tag_wd = {2'b11, mtag};
			lru_we = 1'b1;
		end
		// a flushed line is clean once it is in wbuf (the CPU waits in M)
		S_WB_RD: if (flushing && cnt == 3'd4) begin
			tag_we = 1'b1;
			tag_wd = {2'b10, vtag};
		end
		default: ;
	endcase
end

// ---------------------------------------------------------------- DDR

assign mem_addr  = {mem_we ? vtag : mtag, line, 2'b00};
assign mem_wdata = wbuf[63:0];

// ---------------------------------------------------------------- FSM

always @(posedge clk)
	pm <= {st != S_IDLE && st != S_INIT,
	       mem_we && !mem_busy && wbeat == 2'd3,
	       st == S_FILL_RQ || st == S_FILL,
	       st == S_FILL && mem_rvalid && beat == 2'd3};

always @(posedge clk) begin
	flush_done <= 1'b0;
	zl_done    <= 1'b0;
	if (reset) begin
		st       <= S_INIT;
		line     <= 9'd0;
		way      <= 1'b0;
		mem_req  <= 1'b0;
		mem_rd   <= 1'b0;
		mem_we   <= 1'b0;
		flushing <= 1'b0;
		zl       <= 1'b0;
		wb_pend  <= 1'b0;
	end else begin
		case (st)
		S_INIT: begin
			line <= line + 9'd1;
			if (line == 9'h1ff) st <= S_IDLE;
		end
		S_IDLE: begin
			if (flush && !flush_done) begin
				if (!wb_pend) begin
					flushing <= 1'b1;
					line     <= 9'd0;
					way      <= 1'b0;
					st       <= S_FL_RD;
				end
			end else if (m_zl) begin
				if (!zl_done && rd_ok && !wb_pend) begin
					line <= m_addr[13:5];
					mtag <= m_addr[24:14];
					way  <= hit1_c;
					zl   <= 1'b1;
					cnt  <= 3'd0;
					st   <= hit_c ? S_ZERO : S_TAG_CHK;
				end
			end else if (access && rd_ok && !hit && !wb_pend) begin
				line    <= a_addr[13:5];
				mtag    <= a_addr[24:14];
				zl      <= 1'b0;
				mem_req <= 1'b1;
				st      <= S_TAG_CHK;
			end
		end
		S_TAG_CHK: begin
			way <= victim;
			cnt <= 3'd0;
			if (v_tag[12] && v_tag[11]) begin
				vtag <= v_tag[10:0];
				st   <= S_WB_RD;
			end else if (zl) begin
				st   <= S_ZERO;
			end else begin
				mem_rd <= 1'b1;
				st     <= S_FILL_RQ;
			end
		end
		S_WB_RD: begin
			// RAM address {line, cnt}; data for cnt-1 arrives now
			cnt <= cnt + 3'd1;
			if (cnt != 3'd0) wbuf <= {f_data, wbuf[255:64]};
			if (cnt == 3'd4) begin
				wb_pend <= 1'b1;
				wbeat   <= 2'd0;
				cnt     <= 3'd0;
				if (flushing) begin
					st <= S_WB_WAIT;
				end else if (zl) begin
					st <= S_ZERO;
				end else begin
					mem_rd <= 1'b1;
					st     <= S_FILL_RQ;
				end
			end
		end
		S_WB_WAIT: if (!wb_pend) st <= S_FL_NEXT;
		S_ZERO: begin
			cnt <= cnt + 3'd1;
			if (cnt[1:0] == 2'd3) begin
				zl      <= 1'b0;
				zl_done <= 1'b1;
				st      <= S_IDLE;
			end
		end
		S_FILL_RQ: begin
			// mem_req since the miss, mem_rd since S_TAG_CHK / S_WB_RD
			if (mem_gnt && !mem_busy) begin
				mem_rd <= 1'b0;
				beat   <= 2'd0;
				st     <= S_FILL;
			end
		end
		S_FILL: begin
			if (mem_rvalid) begin
				beat <= beat + 2'd1;
				if (beat == 2'd3) begin
					// a write-back follows on the same grant
					if (!wb_pend) mem_req <= 1'b0;
					st <= S_IDLE;
				end
			end
		end
		S_FL_RD: st <= S_FL_CHK;
		S_FL_CHK: begin
			cnt <= 3'd0;
			if (f_tag[12] && f_tag[11]) begin
				vtag <= f_tag[10:0];
				st   <= S_WB_RD;
			end else begin
				st <= S_FL_NEXT;
			end
		end
		S_FL_NEXT: begin
			// both ways of a set, then the next set
			way <= !way;
			if (!way) begin
				st <= S_FL_RD;
			end else begin
				line <= line + 9'd1;
				if (line == 9'h1ff) begin
					flushing   <= 1'b0;
					flush_done <= 1'b1;
					st         <= S_IDLE;
				end else begin
					st <= S_FL_RD;
				end
			end
		end
		default: st <= S_IDLE;
		endcase

		// the write-back of wbuf, when no fill uses the bus (to {vtag,
		// line}: the next miss waits for it, so they stay). Only this
		// raises mem_we, so a beat is taken on mem_we && !mem_busy alone
		// (the HPS's busy meets one LUT in front of the shift).
		if (wb_pend && !mem_we && st != S_WB_RD && st != S_FILL_RQ && st != S_FILL) begin
			mem_req <= 1'b1;
			if (mem_gnt) mem_we <= 1'b1;
		end
		if (mem_we && !mem_busy) begin
			wbeat <= wbeat + 2'd1;
			wbuf  <= {64'd0, wbuf[255:64]};
			if (wbeat == 2'd3) begin
				mem_we  <= 1'b0;
				mem_req <= 1'b0;
				wb_pend <= 1'b0;
			end
		end
	end
end

endmodule
