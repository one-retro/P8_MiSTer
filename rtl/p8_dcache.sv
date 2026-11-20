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
// victim (an invalid way, else the least recently used one), writes it
// back if dirty and fills the line.
//
// `m_zl` (p8.zline) zeroes the line at m_addr without reading memory: a
// miss writes back the dirty victim, then the line is written with zeros
// and marked dirty (S_ZERO); `zl_done` (and `ready`) pulse after that.
//
// `flush` (level) writes back every dirty line (lines stay valid, now
// clean); `flush_done` pulses at the end. Used for FENCE.I.

module p8_dcache (
	input             clk,
	input             reset,

	input      [31:0] e_addr,
	input      [31:0] m_addr,
	input             m_hold,
	input             m_rd,
	input             m_wr,
	input             m_zl,
	input       [3:0] m_be,
	input      [31:0] m_wdata,
	output     [31:0] rdata0,    // the addressed word in each way; the
	output     [31:0] rdata1,    // load data is hit1 ? rdata1 : rdata0
	output            rhit1,
	output     [63:0] rdata64_0, // the whole 64-bit word of each way (for
	output     [63:0] rdata64_1, // p8_tunit, which selects by rhit1 last;
	                             // without the RAMs' write bypass: no store
	                             // hits while the table unit runs)
	output            ready,
	output reg        zl_done,   // p8.zline finished (one cycle)

	input             flush,
	output reg        flush_done,

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
           S_TAG_RD  = 4'd2,
           S_TAG_CHK = 4'd3,
           S_WB_RD   = 4'd4,
           S_WB_REQ  = 4'd5,
           S_WB_DONE = 4'd6,
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
reg [63:0] wbuf [0:3];
reg        to_fill;   // memory transaction is a fill (else write-back)
reg        zl;        // the miss being handled is a p8.zline

// ---------------------------------------------------------------- RAMs

reg   [8:0] t_ra;
reg  [10:0] d_ra;
always @* begin
	case (st)
		S_IDLE:   begin
			t_ra = m_hold ? m_addr[13:5] : e_addr[13:5];
			d_ra = m_hold ? m_addr[13:3] : e_addr[13:3];
		end
		default:  begin t_ra = line;         d_ra = {line, cnt[1:0]}; end
	endcase
end

// per way: tag RAM {valid, dirty, tag[24:14]} and 8 data byte lanes
wire [12:0] tag_q  [0:1];
wire [63:0] data_q [0:1];
wire [63:0] data_raw [0:1];
reg         tag_we;
reg         tag_way;
reg   [8:0] tag_wa;
reg  [12:0] tag_wd;

// the RAM outputs are for m_addr: read in S_IDLE in the previous cycle
reg         rd_ok;
always @(posedge clk) rd_ok <= st == S_IDLE;

wire        access = m_rd || m_wr || m_zl;
wire        hit0   = rd_ok && tag_q[0][12] && tag_q[0][10:0] == m_addr[24:14];
wire        hit1   = rd_ok && tag_q[1][12] && tag_q[1][10:0] == m_addr[24:14];
wire        hit    = hit0 || hit1;
wire        idle   = st == S_IDLE;
wire        st_hit = idle && m_wr && hit;

assign ready = idle && (!access || (m_zl ? zl_done : hit));

wire  [7:0] be8 = m_addr[2] ? {m_be, 4'b0000} : {4'b0000, m_be};
wire        fill_we = st == S_FILL && mem_rvalid;
wire        zero_we = st == S_ZERO;

genvar gw, gi;
generate for (gw = 0; gw < 2; gw = gw + 1) begin : w
	p8_bram #(.AW(9), .DW(13)) tags (
		.clk(clk), .re(1'b1), .raddr(t_ra), .rdata(tag_q[gw]), .rdata_raw(),
		.we(tag_we && (st == S_INIT || tag_way == gw)), .waddr(tag_wa), .wdata(tag_wd)
	);
	wire sel = (fill_we || zero_we) ? way == gw : (gw ? hit1 : !hit1);
	for (gi = 0; gi < 8; gi = gi + 1) begin : lane
		p8_bram #(.AW(11), .DW(8)) ram (
			.clk(clk), .re(1'b1), .raddr(d_ra), .rdata(data_q[gw][gi*8 +: 8]), .rdata_raw(data_raw[gw][gi*8 +: 8]),
			.we(sel && (fill_we || zero_we || (st_hit && be8[gi]))),
			.waddr(fill_we ? {line, beat} : zero_we ? {line, cnt[1:0]} : m_addr[13:3]),
			.wdata(fill_we ? mem_rdata[gi*8 +: 8] : zero_we ? 8'd0 : m_wdata[(gi%4)*8 +: 8])
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

wire [63:0] hit_q = hit1 ? data_q[1] : data_q[0];
assign rdata0  = m_addr[2] ? data_q[0][63:32] : data_q[0][31:0];
assign rdata1  = m_addr[2] ? data_q[1][63:32] : data_q[1][31:0];
assign rhit1   = hit1;
assign rdata64_0 = data_raw[0];
assign rdata64_1 = data_raw[1];

// the FSM's way (victim, line being zeroed or flushed)
wire [12:0] f_tag  = way ? tag_q[1] : tag_q[0];
wire [63:0] f_data = way ? data_q[1] : data_q[0];
// victim for the set in tag_q: an invalid way, else the LRU one
wire        victim = !tag_q[0][12] ? 1'b0 : !tag_q[1][12] ? 1'b1 : lru_q;
wire [12:0] v_tag  = victim ? tag_q[1] : tag_q[0];

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
			// only the enable depends on the hit (late)
			tag_way = hit1;
			tag_wa  = m_addr[13:5];
			tag_wd  = {2'b11, m_addr[24:14]};
			tag_we  = st_hit && !(hit1 ? tag_q[1][11] : tag_q[0][11]);
			if (access && !m_zl && hit) begin
				lru_we = 1'b1;
				lru_wa = m_addr[13:5];
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
		S_WB_DONE: if (flushing) begin
			tag_we = 1'b1;
			tag_wd = {2'b10, vtag};
		end
		default: ;
	endcase
end

// ---------------------------------------------------------------- DDR

assign mem_addr  = {to_fill ? mtag : vtag, line, 2'b00};
assign mem_wdata = wbuf[beat];

// ---------------------------------------------------------------- FSM

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
	end else begin
		case (st)
		S_INIT: begin
			line <= line + 9'd1;
			if (line == 9'h1ff) st <= S_IDLE;
		end
		S_IDLE: begin
			if (flush && !flush_done) begin
				flushing <= 1'b1;
				line     <= 9'd0;
				way      <= 1'b0;
				st       <= S_FL_RD;
			end else if (m_zl) begin
				if (!zl_done && rd_ok) begin
					line <= m_addr[13:5];
					mtag <= m_addr[24:14];
					way  <= hit1;
					zl   <= 1'b1;
					cnt  <= 3'd0;
					st   <= hit ? S_ZERO : S_TAG_RD;
				end
			end else if (access && rd_ok && !hit) begin
				line <= m_addr[13:5];
				mtag <= m_addr[24:14];
				zl   <= 1'b0;
				st   <= S_TAG_RD;
			end
		end
		S_TAG_RD: st <= S_TAG_CHK;
		S_TAG_CHK: begin
			way <= victim;
			cnt <= 3'd0;
			if (v_tag[12] && v_tag[11]) begin
				vtag <= v_tag[10:0];
				st   <= S_WB_RD;
			end else begin
				st   <= zl ? S_ZERO : S_FILL_RQ;
			end
		end
		S_WB_RD: begin
			// RAM address {line, cnt}; data for cnt-1 arrives now
			cnt <= cnt + 3'd1;
			if (cnt != 3'd0) wbuf[cnt - 3'd1] <= f_data;
			if (cnt == 3'd4) begin
				to_fill <= 1'b0;
				beat    <= 2'd0;
				mem_req <= 1'b1;
				st      <= S_WB_REQ;
			end
		end
		S_WB_REQ: begin
			if (mem_gnt) begin
				mem_we <= 1'b1;
				if (mem_we && !mem_busy) begin
					beat <= beat + 2'd1;
					if (beat == 2'd3) begin
						mem_we  <= 1'b0;
						mem_req <= 1'b0;
						st      <= S_WB_DONE;
					end
				end
			end
		end
		S_WB_DONE: begin
			cnt <= 3'd0;
			st  <= flushing ? S_FL_NEXT : zl ? S_ZERO : S_FILL_RQ;
		end
		S_ZERO: begin
			cnt <= cnt + 3'd1;
			if (cnt[1:0] == 2'd3) begin
				zl      <= 1'b0;
				zl_done <= 1'b1;
				st      <= S_IDLE;
			end
		end
		S_FILL_RQ: begin
			to_fill <= 1'b1;
			mem_req <= 1'b1;
			if (mem_gnt) begin
				mem_rd <= 1'b1;
				if (mem_rd && !mem_busy) begin
					mem_rd <= 1'b0;
					beat   <= 2'd0;
					st     <= S_FILL;
				end
			end
		end
		S_FILL: begin
			if (mem_rvalid) begin
				beat <= beat + 2'd1;
				if (beat == 2'd3) begin
					mem_req <= 1'b0;
					st      <= S_IDLE;
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
	end
end

endmodule
