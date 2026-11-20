// Instruction cache: 64 KiB, 2-way set associative (LRU), 32-byte lines,
// 64-bit RAMs.
//
// Predecode: a line fill also stores, per instruction, what the CPU's
// next-fetch logic needs: {JALR predicted (offset 0), a return (rd x0,
// rs1 ra/t0), conditional branch, statically predicted taken (JAL,
// backward branch, predicted JALR), uses rs1, uses rs2, absolute target
// [24:2]}. The target adder and the opcode
// decode run on the fill data, not in the fetch loop (RAM output -> next
// address -> RAM address). The DDR data is registered first (it comes
// from the HPS interface at the edge of the chip), so each beat is written
// a cycle after it arrives and the last one in S_LAST.
//
// Way prediction: a 1-bit RAM per set holds the most recently used way,
// read together with the tags. `data` comes from that way (no tag compare
// in the data path); a hit in the other way switches the prediction and
// answers 2 cycles later (S_DONE; the CPU presents the address again). The MRU bit is also the LRU replacement choice.
//
// `data`/`valid` answer for the current address q_addr, the last `addr`
// presented with `adv` high (the CPU's d_pc). Without `adv` the RAMs are
// not read and the output stays. `adv` is late (it depends on `valid`), so
// it only drives read enables; the RAM address is `addr`, or q_addr when
// the FSM re-reads it (S_DONE, where `valid` is low and the CPU advances
// only on `redir`, an early signal). A miss starts a line fill from DDR (4 x 64-bit
// burst); `valid` stays low until the line is present (S_DONE re-reads
// q_addr). `adv` may come during a fill (a CPU redirect): the fill goes
// on and S_DONE reads the new address.
//
// `inval` (level) invalidates everything; `inval_done` pulses when done.
// Address bits above 24 are ignored (32 MiB of DDR). The RAMs are never
// read in the cycle they are written (S_DONE after every write sequence),
// so they need no bypass.

module p8_icache (
	input             clk,
	input             reset,

	input      [31:0] addr,
	input             adv,
	input             redir,      // adv while `valid` is low (registered in the CPU)
	output     [31:0] data,
	output     [31:0] data0,      // the word in each way, and the way `data` is from
	output     [31:0] data1,
	output     [28:0] pre0,       // predecode of data0 / data1 (see above)
	output     [28:0] pre1,
	output            way_q,
	// both words of the 64-bit line in each way and which one `data*` is
	// (the CPU's register read decodes all four and selects last)
	output     [63:0] line0,
	output     [63:0] line1,
	output            word_q,
	output            valid,

	input             inval,
	output reg        inval_done,

	// DDR client (see p8_ddr_arb)
	output reg        mem_req,
	input             mem_gnt,
	output reg        mem_rd,
	output     [21:0] mem_addr,   // 64-bit word address (byte address [24:3])
	input             mem_busy,
	input      [63:0] mem_rdata,
	input             mem_rvalid
);

// set index bits (1024 sets x 2 ways x 32 bytes) and tag bits (byte address [24:5+SB])
localparam SB = 10, TB = 20 - SB;

localparam S_INIT = 3'd0, S_IDLE = 3'd1, S_REQ = 3'd2, S_FILL = 3'd3, S_LAST = 3'd4, S_DONE = 3'd5;

reg  [2:0] st;
reg [31:0] q_addr;
reg [SB-1:0] sweep;
reg        inv_run;
reg [19:0] line;       // byte address [24:5] of the line being filled
reg        way;        // way being filled
reg  [1:0] beat;
// the DDR beat received last cycle, written to the line now
reg        f_we;
reg  [1:0] f_beat;
reg [63:0] f_data;


always @(posedge clk) if (adv) q_addr <= addr;

always @(posedge clk) begin
	f_we   <= st == S_FILL && mem_rvalid;
	f_beat <= beat;
	f_data <= mem_rdata;
end

// RAM read address and enable: in S_IDLE `addr` when the CPU advances; in
// S_DONE the current address (or the new one if the CPU advances then;
// adv is early outside S_IDLE). Reads in the other states do not matter.
wire [31:0] r_addr = (st == S_DONE && !redir) ? q_addr : addr;
wire        r_en   = st != S_IDLE || adv;

// {jalr, ret, branch, pred, use rs1, use rs2, target[24:2]} of instruction x
// at byte address pc
function automatic [28:0] predec(input [31:0] x, input [24:2] pc);
	reg jal, br, jr, ret, use1, use2;
	reg [6:0] o;
	reg [24:2] imm;
	begin
		o    = x[6:0];
		jal  = o == 7'b1101111;
		br   = o == 7'b1100011;
		// JALR with offset 0 (the CPU checks rs1 against the prediction)
		jr   = o == 7'b1100111 && x[14:12] == 3'd0 && x[31:20] == 12'd0;
		ret  = jr && x[11:7] == 5'd0 && (x[19:15] == 5'd1 || x[19:15] == 5'd5);
		// p8.hfind (funct7 0 string key, 1 number key), p8.zline
		use1 = o == 7'b1100111 || br || o == 7'b0000011 || o == 7'b0100011 || o == 7'b0010011 ||
		       o == 7'b0110011 || (o == 7'b1110011 && x[13:12] != 2'd0 && !x[14]) ||
		       (o == 7'b0001011 && x[14:13] == 2'd0 && x[31:26] == 6'd0);
		// beqi / bnei (funct3 2, 3): rs2 is an immediate
		use2 = (br && x[14:13] != 2'b01) || o == 7'b0100011 || o == 7'b0110011 ||
		       (o == 7'b0001011 && x[14:12] == 3'd0 && x[31:26] == 6'd0);
		imm  = jal ? {{5{x[31]}}, x[19:12], x[20], x[30:22]} : {{13{x[31]}}, x[7], x[30:25], x[11:9]};
		predec = {jr, ret, br, jal | (br & x[31]) | jr, use1, use2, pc + imm};
	end
endfunction

wire [57:0] fill_pre = {predec(f_data[63:32], {line, f_beat, 1'b1}), predec(f_data[31:0], {line, f_beat, 1'b0})};

// per way: tag RAM {valid, tag} and data RAM {predecode hi, lo, words hi, lo}
wire [TB:0] tag_q [0:1];
wire [121:0] data_q [0:1];
reg         tag_we;
reg [SB-1:0] tag_wa;
reg  [TB:0] tag_wd;
genvar gw;
generate for (gw = 0; gw < 2; gw = gw + 1) begin : w
	p8_bram #(.AW(SB), .DW(TB+1), .BYPASS(0)) tags (
		.clk(clk), .re(r_en), .raddr(r_addr[SB+4:5]), .rdata(tag_q[gw]), .rdata_raw(),
		.we(tag_we && (st == S_INIT || way == gw)), .waddr(tag_wa), .wdata(tag_wd)
	);
	p8_bram #(.AW(SB+2), .DW(122), .BYPASS(0)) lines (
		.clk(clk), .re(r_en), .raddr(r_addr[SB+4:3]), .rdata(data_q[gw]), .rdata_raw(),
		.we(f_we && way == gw), .waddr({line[SB-1:0], f_beat}), .wdata({fill_pre, f_data})
	);
end endgenerate

// most recently used way per set
wire       mru_q;
reg        mru_we, mru_wd;
reg [SB-1:0] mru_wa;
p8_bram #(.AW(SB), .DW(1), .BYPASS(0)) mru (
	.clk(clk), .re(r_en), .raddr(r_addr[SB+4:5]), .rdata(mru_q), .rdata_raw(),
	.we(mru_we), .waddr(mru_wa), .wdata(mru_wd)
);

wire [TB:0] ptag = mru_q ? tag_q[1] : tag_q[0];
wire [TB:0] otag = mru_q ? tag_q[0] : tag_q[1];
wire hit   = ptag[TB] && ptag[TB-1:0] == q_addr[24:SB+5];   // predicted way
wire ohit  = otag[TB] && otag[TB-1:0] == q_addr[24:SB+5];   // the other way
wire [63:0] hit_q = mru_q ? data_q[1][63:0] : data_q[0][63:0];
// fill an invalid way first, else the least recently used one
wire victim = !tag_q[0][TB] ? 1'b0 : !tag_q[1][TB] ? 1'b1 : !mru_q;

always @* begin
	mru_we = 1'b0;
	mru_wa = q_addr[SB+4:5];
	mru_wd = !mru_q;
	if (st == S_IDLE && !hit && ohit) mru_we = 1'b1;       // switch prediction
	if (st == S_FILL && mem_rvalid && beat == 2'd3) begin  // filled way is MRU
		mru_we = 1'b1;
		mru_wa = line[SB-1:0];
		mru_wd = way;
	end
end

assign valid = st == S_IDLE && hit;
assign data  = q_addr[2] ? hit_q[63:32] : hit_q[31:0];
assign data0 = q_addr[2] ? data_q[0][63:32] : data_q[0][31:0];
assign data1 = q_addr[2] ? data_q[1][63:32] : data_q[1][31:0];
assign pre0  = q_addr[2] ? data_q[0][121:93] : data_q[0][92:64];
assign pre1  = q_addr[2] ? data_q[1][121:93] : data_q[1][92:64];
assign way_q = mru_q;
assign line0 = data_q[0][63:0];
assign line1 = data_q[1][63:0];
assign word_q = q_addr[2];
assign mem_addr = {line, 2'b00};

always @* begin
	tag_we = 1'b0;
	tag_wa = line[SB-1:0];
	tag_wd = {1'b1, line[19:SB]};
	if (st == S_INIT) begin
		tag_we = 1'b1;
		tag_wa = sweep;
		tag_wd = '0;
	end else if (st == S_FILL && mem_rvalid && beat == 2'd3) begin
		tag_we = 1'b1;
	end
end

always @(posedge clk) begin
	inval_done <= 1'b0;
	if (reset) begin
		st      <= S_INIT;
		sweep   <= '0;
		inv_run <= 1'b0;
		mem_req <= 1'b0;
		mem_rd  <= 1'b0;
	end else begin
		case (st)
		S_INIT: begin
			sweep <= sweep + 1'b1;
			if (&sweep) begin
				st <= S_DONE;
				if (inv_run) begin
					inv_run    <= 1'b0;
					inval_done <= 1'b1;
				end
			end
		end
		S_IDLE: begin
			if (inval && !inval_done) begin
				inv_run <= 1'b1;
				sweep   <= '0;
				st      <= S_INIT;
			end else if (ohit && !hit) begin
				st <= S_DONE;
			end else if (!hit) begin
				line    <= q_addr[24:5];
				way     <= victim;
				mem_req <= 1'b1;
				st      <= S_REQ;
			end
		end
		S_REQ: begin
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
					st      <= S_LAST;
				end
			end
		end
		S_LAST: st <= S_DONE;
		default: st <= S_IDLE;  // S_DONE: the CPU re-reads the line
		endcase
	end
end

endmodule
