// RV32IM + Zbb pipeline for the P8 soft CPU.
//
// Stages: D (instruction arrives from the I$, decode, register read),
// E (ALU, branches, divider), M (data memory, load alignment), W (write
// back). The fetch address is chosen combinationally every cycle and
// registered by the I$ RAM, so there is no separate F stage register.
//
// Late signals (I$ hit, D$ hit) never select an address or gate the
// pipeline registers. A D-stage stall only holds the I$ output
// (if_adv = 0) and d_pc. A D$ miss is not a stall: the access is replayed
// (the instruction does not retire, the younger ones are dropped and
// fetching restarts at it, like a mispredict) until it hits. The only
// pipeline stalls come from registered state: p8.hfind, p8.zline and
// FENCE.I waiting in M, and the divider in E.
//
// - Prediction in D: JAL taken; a conditional branch follows the static
//   rule (backward taken) unless its counter in a 1K x 2-bit table, read
//   with the fetch address like the I$, says it disagrees ("agree"
//   predictor; the counters start at 0 = strongly agree, the RAM's
//   power-up value). The counter is updated a cycle after the branch
//   leaves E. A JALR with offset 0 (predecoded) is predicted taken to its
//   entry of a 256 x 23-bit target buffer (no tags; read with the fetch
//   address, written a cycle after a JALR leaves M). With RAS = 1 a return
//   (rd x0, rs1 ra or t0) predicts from an 8-entry return-address stack
//   instead (JAL/JALR with rd ra or t0 push pc + 4 when they leave D); it
//   is off: the extra select does not fit in the fetch loop at 100 MHz.
//   M checks rs1 (registered in E, off the forwarding paths) against the
//   predicted target and redirects on a mismatch (3 bubbles).
// - Branch / JALR resolution in E, D$ replays and FENCE.I in M: the
//   redirect is registered and fetches the target in the next cycle
//   (2 bubbles on a mispredict or a JALR with an offset; 3 for a
//   mispredicted JALR, found in M).
// - Full forwarding from M (ALU results) and W (everything), selected by
//   registers set in D. A load or MUL followed by a dependent instruction
//   costs one bubble.
// - MUL/MULH* in one cycle (DSP, product registered into W).
// - Zbb (andn orn xnor clz ctz cpop sext.b/h zext.h min[u] max[u] rol ror
//   rori orc.b rev8): computed in M from registered operands, like MUL,
//   so the ALU result path does not grow; the next instruction waits a
//   cycle for the result.
// - DIV/REM: 33-cycle restoring divider in E (operands registered first).
// - FENCE.I waits in M for the memory system (D$ write-back + I$
//   invalidate), then refetches the next instruction.
// - CSRs: cycle/time/instret (and *h) read-only, mscratch read-write;
//   other CSRs read 0. No traps or interrupts: ECALL/EBREAK/FENCE are
//   no-ops.
//   Misaligned accesses are not supported (the compilers never emit them).
// - p8.hfind rd, rs1, rs2 (custom-0, funct3 0, funct7 0: a string key, 1: a
//   number key): table lookup done by the memory system (p8_tunit); waits
//   in M like a load. CSR 0x7c0 (p8heap) holds the heap arena base for it.
// - beqi / bnei rs1, imm5, offset (BRANCH funct3 2 / 3): compare rs1 with
//   the unsigned immediate in the rs2 field.
// - CSR 0x7c1 (p8feat, read-only) lists these extensions: 1 number-key
//   p8.hfind, 2 beqi/bnei, 4 JALR prediction (earlier cores read 0).
// - p8.zline rs1 (custom-0, funct3 1): the D$ zeroes the 32-byte line at
//   rs1 without reading memory; waits in M like a store.
//
// Memory interface: the memory system receives the E-stage address
// (de_addr) for its synchronous RAMs and the registered M-stage request
// (dm_*). It answers in M with dm_rdata0/1 (+ dm_rsel1) and dm_ready. For
// loads and stores dm_ready = 0 means "replay"; p8.hfind, p8.zline and
// FENCE.I wait in M for dm_done (an OR of registered pulses, each only
// for its own operation) and their address stays in dm_addr.

module rv32_cpu #(
	// 1: returns predict from the return-address stack; 0: from the target
	// buffer like other JALRs (the target select is then one LUT, as deep
	// as the way select alone)
	parameter RAS = 0
) (
	input             clk,
	input             reset,

	// instruction fetch
	output reg [31:0] if_addr,   // next fetch address
	output            if_adv,    // fetch if_addr (else the I$ keeps its output)
	output            if_redir,  // if_adv from registers only (reset or redirect)
	input      [31:0] if_data,   // instruction at d_pc (the address last fetched)
	// the same word from both I$ ways, its predecode (from the line fill:
	// {branch, predict taken, uses rs1, uses rs2, target[24:2]}) and the way
	// `if_data` comes from: the next-fetch logic (prediction, load-use
	// hazard) looks at both and selects late, keeping the way mux off the
	// fetch-address path
	input      [31:0] if_data0,
	input      [31:0] if_data1,
	input      [63:0] if_line0,  // both words of each way's line, and which word
	input      [63:0] if_line1,  // if_data0/1 are (for the register read)
	input             if_word,
	input      [28:0] if_pre0,
	input      [28:0] if_pre1,
	input             if_way,
	input             if_valid,

	// data memory
	output     [31:0] de_addr,   // E-stage effective address
	output            de_load,   // E holds a load (hint for shared read ports)
	// a shared read port's word address (PICO-8 RAM): de_addr[15:2] when
	// ra_cpu, else ra_other (both early)
	input             ra_cpu,
	input      [13:0] ra_other,
	output     [13:0] ra_addr,
	output     [31:0] dm_addr,
	output            dm_rd,
	output            dm_wr,
	output      [3:0] dm_be,
	output     [31:0] dm_wdata,
	output            dm_fencei,
	output            dm_hfind,    // M holds p8.hfind: table dm_addr, key dm_hkey
	output            dm_hnum,     // ... of a number key
	output            dm_zline,    // M holds p8.zline: line dm_addr
	output     [31:0] dm_hkey,
	output reg [31:0] p8heap,
	input      [31:0] dm_rdata0,   // load data: dm_rsel1 ? dm_rdata1 : dm_rdata0
	input      [31:0] dm_rdata1,   // (dm_rsel1 is late: the D$ hit way)
	input             dm_rsel1,
	input             dm_ready,    // load/store answered (else replay)
	input      [31:0] dm_hres,     // p8.hfind result (a register, valid with dm_done)
	input             dm_done      // hfind/zline/fence.i finished (registered pulses, ORed)
);

localparam OP_LUI    = 7'b0110111;
localparam OP_AUIPC  = 7'b0010111;
localparam OP_JAL    = 7'b1101111;
localparam OP_JALR   = 7'b1100111;
localparam OP_BRANCH = 7'b1100011;
localparam OP_LOAD   = 7'b0000011;
localparam OP_STORE  = 7'b0100011;
localparam OP_IMM    = 7'b0010011;
localparam OP_OP     = 7'b0110011;
localparam OP_MISC   = 7'b0001111;
localparam OP_SYSTEM = 7'b1110011;
localparam OP_CUST0  = 7'b0001011;

// ---------------------------------------------------------------- state

// D
reg [31:0] d_pc;

// E. e_v is E's valid bit as loaded from D. After any redirect (rd_pend,
// a register) E holds the instruction that was in D, or nothing: it is
// dropped then, which keeps the late branch compare off e_v. Where that is
// harmless (D's hazard and forwarding decisions, D being squashed by
// rd_pend too) e_v is used directly.
reg        e_v, rd_pend;
wire       e_valid = e_v && !rd_pend;
reg [31:0] e_pc, e_imm, e_a, e_b;
reg [31:0] e_pc4;          // e_pc + 4 (from D: off the result mux)
reg  [4:0] e_rd;
reg        e_wen;
reg  [2:0] e_f3;
reg        e_alt, e_op1pc, e_op1zero, e_op2imm;
reg        e_branch, e_jal, e_jalr, e_load, e_store, e_mul, e_div, e_fencei, e_pred, e_csr, e_hfind, e_zline;
reg        e_hnum;         // p8.hfind of a number key
reg        e_jp;           // a predicted JALR (offset 0) ...
reg [22:0] e_ptgt;         // ... and its predicted target [24:2]
reg  [4:0] e_bimm;         // beqi / bnei immediate
reg        e_zbb;
reg  [4:0] e_zop;
reg        e_late;         // its result comes from M or W (load, mul, div, CSR, hfind, Zbb)
reg        e_st;           // a branch's static prediction (backward taken)
reg  [1:0] e_bdis;         // its predictor counter
reg  [4:0] e_zimm;
reg        e_fam, e_faw, e_fbm, e_fbw;   // forward operand a/b from M / W

// M
reg        m_valid;
reg [31:0] m_pc, m_result, m_addr, m_wdata, m_mul_a, m_mul_b, m_hkey, m_zb_b;
reg  [3:0] m_be;
reg  [4:0] m_rd;
reg        m_wen, m_load, m_store, m_mul, m_fencei, m_hfind, m_hnum, m_zline, m_zbb, m_ldsel, m_dc;
reg        m_jp;           // a predicted JALR (rs1 in m_mul_a) ...
reg [22:0] m_ptgt;         // ... and its predicted target
reg [31:0] m_dres;         // divider / CSR result (late, like MUL)
reg  [4:0] m_zop;
reg  [2:0] m_f3;

// W
reg        w_valid;
reg  [4:0] w_rd;
reg        w_wen;
reg [31:0] w_val;

reg [31:0] rf [0:31];

// ---------------------------------------------------------------- W

// load alignment happens on the way into W (M), so the W result is a
// register and forwarding from W has no logic in front of it
wire [31:0] w_result = w_val;
wire        w_write  = w_valid && w_wen && (w_rd != 5'd0);

always @(posedge clk) if (w_write) rf[w_rd] <= w_result;

// ---------------------------------------------------------------- D

wire [31:0] di  = if_data;
wire  [6:0] opc = di[6:0];
wire  [2:0] f3  = di[14:12];
wire  [4:0] d_rd  = di[11:7];
wire  [4:0] d_rs1 = di[19:15];
wire  [4:0] d_rs2 = di[24:20];

wire d_lui    = opc == OP_LUI;
wire d_auipc  = opc == OP_AUIPC;
wire d_jal    = opc == OP_JAL;
wire d_jalr   = opc == OP_JALR;
wire d_branch = opc == OP_BRANCH;
wire d_load   = opc == OP_LOAD;
wire d_store  = opc == OP_STORE;
wire d_opimm  = opc == OP_IMM;
wire d_op     = opc == OP_OP;
wire d_fencei = opc == OP_MISC && f3 == 3'd1;
wire d_muldiv = d_op && di[31:25] == 7'b0000001;
wire d_csr    = opc == OP_SYSTEM && f3[1:0] != 2'd0;
wire d_hfind  = opc == OP_CUST0 && f3 == 3'd0 && di[31:26] == 6'd0;   // funct7 0 or 1
wire d_zline  = opc == OP_CUST0 && f3 == 3'd1 && di[31:25] == 7'd0;

// Zbb, and which operation (m_zop)
localparam Z_ANDN = 5'd0, Z_ORN = 5'd1, Z_XNOR = 5'd2, Z_CLZ = 5'd3, Z_CTZ = 5'd4, Z_CPOP = 5'd5,
           Z_SEXTB = 5'd6, Z_SEXTH = 5'd7, Z_ZEXTH = 5'd8, Z_MAX = 5'd9, Z_MAXU = 5'd10, Z_MIN = 5'd11,
           Z_MINU = 5'd12, Z_ROL = 5'd13, Z_ROR = 5'd14, Z_ORCB = 5'd15, Z_REV8 = 5'd16;
reg       d_zbb;
reg [4:0] d_zop;
always @* begin
	d_zbb = 1'b1;
	d_zop = Z_ANDN;
	if (d_op && di[31:25] == 7'b0100000 && f3 == 3'd7) d_zop = Z_ANDN;
	else if (d_op && di[31:25] == 7'b0100000 && f3 == 3'd6) d_zop = Z_ORN;
	else if (d_op && di[31:25] == 7'b0100000 && f3 == 3'd4) d_zop = Z_XNOR;
	else if (d_op && di[31:25] == 7'b0000101 && f3 == 3'd4) d_zop = Z_MIN;
	else if (d_op && di[31:25] == 7'b0000101 && f3 == 3'd5) d_zop = Z_MINU;
	else if (d_op && di[31:25] == 7'b0000101 && f3 == 3'd6) d_zop = Z_MAX;
	else if (d_op && di[31:25] == 7'b0000101 && f3 == 3'd7) d_zop = Z_MAXU;
	else if (d_op && di[31:25] == 7'b0110000 && f3 == 3'd1) d_zop = Z_ROL;
	else if (d_op && di[31:25] == 7'b0110000 && f3 == 3'd5) d_zop = Z_ROR;
	else if (d_op && di[31:20] == 12'h080 && f3 == 3'd4) d_zop = Z_ZEXTH;
	else if (d_opimm && di[31:20] == 12'h600 && f3 == 3'd1) d_zop = Z_CLZ;
	else if (d_opimm && di[31:20] == 12'h601 && f3 == 3'd1) d_zop = Z_CTZ;
	else if (d_opimm && di[31:20] == 12'h602 && f3 == 3'd1) d_zop = Z_CPOP;
	else if (d_opimm && di[31:20] == 12'h604 && f3 == 3'd1) d_zop = Z_SEXTB;
	else if (d_opimm && di[31:20] == 12'h605 && f3 == 3'd1) d_zop = Z_SEXTH;
	else if (d_opimm && di[31:25] == 7'b0110000 && f3 == 3'd5) d_zop = Z_ROR;   // rori
	else if (d_opimm && di[31:20] == 12'h287 && f3 == 3'd5) d_zop = Z_ORCB;
	else if (d_opimm && di[31:20] == 12'h698 && f3 == 3'd5) d_zop = Z_REV8;
	else d_zbb = 1'b0;
end

wire d_use1 = d_jalr | d_branch | d_load | d_store | d_opimm | d_op | (d_csr & ~f3[2]) | d_hfind | d_zline;
wire d_use2 = (d_branch && f3[2:1] != 2'b01) | d_store | d_op | d_hfind;   // not beqi/bnei
wire d_wen  = d_lui | d_auipc | d_jal | d_jalr | d_load | d_opimm | d_op | d_csr | d_hfind;

// immediate of the word of each I$ way, selected by the (late) way last
function automatic [31:0] imm_of(input [31:0] x);
	reg [6:0] o;
	begin
		o = x[6:0];
		if (o == OP_LUI || o == OP_AUIPC)
			imm_of = {x[31:12], 12'b0};
		else if (o == OP_JAL)
			imm_of = {{11{x[31]}}, x[31], x[19:12], x[20], x[30:21], 1'b0};
		else if (o == OP_BRANCH)
			imm_of = {{19{x[31]}}, x[31], x[7], x[30:25], x[11:8], 1'b0};
		else if (o == OP_STORE)
			imm_of = {{20{x[31]}}, x[31:25], x[11:7]};
		else if (o == OP_CUST0 && x[14:13] == 2'd0 && x[31:26] == 6'd0)
			imm_of = 32'd0;   // p8.hfind, p8.zline
		else
			imm_of = {{20{x[31]}}, x[31:20]};
	end
endfunction
wire [31:0] d_imm = if_way ? imm_of(if_data1) : imm_of(if_data0);

// Register read for the word of each I$ way in parallel (two copies of the
// read ports), selected by the way last: the way is late.
function automatic [31:0] rf_rd(input [4:0] r, input [31:0] v);
	rf_rd = (r == 5'd0) ? 32'd0 : (w_write && w_rd == r) ? w_result : v;
endfunction
// Four copies: each word of each way's 64-bit line, straight from the RAM
// outputs; the word (if_word, from the address) and the way select last
// (the word select would otherwise sit in front of the 32:1 read mux).
function automatic [31:0] rf_rd4(input [4:0] lo0, input [4:0] hi0, input [4:0] lo1, input [4:0] hi1);
	rf_rd4 = if_way ? (if_word ? rf_rd(hi1, rf[hi1]) : rf_rd(lo1, rf[lo1]))
	                : (if_word ? rf_rd(hi0, rf[hi0]) : rf_rd(lo0, rf[lo0]));
endfunction
wire [31:0] rf_a = rf_rd4(if_line0[19:15], if_line0[51:47], if_line1[19:15], if_line1[51:47]);
wire [31:0] rf_b = rf_rd4(if_line0[24:20], if_line0[56:52], if_line1[24:20], if_line1[56:52]);

// Prediction (see the top): the I$ predecodes {branch, static prediction,
// target} (code lives below 32 MiB, so the target has no bits above 24);
// the predictor table is read with the fetch address, so its counter for
// the D instruction arrives with it. Per way: static ^ (branch & disagree),
// selected by the late way last (one LUT, like the static prediction).
wire  [1:0] bht_q;
// e_rd of a result that is late (load, mul, hfind, Zbb): D must wait a cycle
// (e_late is one register: this feeds the I$ read enables through hazard_d)
wire  [4:0] late_rd = (e_v && e_late) ? e_rd : 5'd0;
wire haz0 = (if_pre0[24] && if_data0[19:15] == late_rd) || (if_pre0[23] && if_data0[24:20] == late_rd);
wire haz1 = (if_pre1[24] && if_data1[19:15] == late_rd) || (if_pre1[23] && if_data1[24:20] == late_rd);

wire        d_pred   = if_way ? if_pre1[25] ^ (if_pre1[26] & bht_q[1]) : if_pre0[25] ^ (if_pre0[26] & bht_q[1]);
wire        d_st     = if_way ? if_pre1[25] : if_pre0[25];
// a predicted JALR's target: the return stack's top (a register) or the
// target buffer's entry (read with the fetch address, like bht_q). It is
// its own input of the fetch-address mux (d_jp implies d_pred), so the
// predecoded target keeps a single way select in front of that mux.
reg  [22:0] ras_top;
wire [22:0] btb_q;
wire        d_jp     = if_way ? if_pre1[28] : if_pre0[28];
wire        d_ret    = RAS && (if_way ? if_pre1[27] : if_pre0[27]);
wire [22:0] d_jtgt   = d_ret ? ras_top : btb_q;
wire [31:0] d_target = {7'd0, if_way ? if_pre1[22:0] : if_pre0[22:0], 2'b00};
wire [22:0] d_tgt23  = d_jp ? d_jtgt : d_target[24:2];

// ---------------------------------------------------------------- E

// Forwarding, decided in D for the instructions that will be in M and W
// when the D instruction is in E (registers e_f*). x0 and unused operands
// never match (rf_a/rf_b are 0 for x0).
wire [31:0] fa = e_fam ? m_result : e_faw ? w_result : e_a;
wire [31:0] fb = e_fbm ? m_result : e_fbw ? w_result : e_b;

wire [31:0] op1 = e_op1pc ? e_pc : e_op1zero ? 32'd0 : fa;
wire [31:0] op2 = e_op2imm ? e_imm : fb;

reg [31:0] alu;
always @* begin
	case (e_f3)
		3'd0: alu = e_alt ? op1 - op2 : op1 + op2;
		3'd1: alu = op1 << op2[4:0];
		3'd2: alu = {31'd0, $signed(op1) < $signed(op2)};
		3'd3: alu = {31'd0, op1 < op2};
		3'd4: alu = op1 ^ op2;
		3'd5: alu = e_alt ? $unsigned($signed(op1) >>> op2[4:0]) : op1 >> op2[4:0];
		3'd6: alu = op1 | op2;
		default: alu = op1 & op2;
	endcase
end

reg taken;
always @* begin
	case (e_f3)
		3'd0: taken = fa == fb;
		3'd1: taken = fa != fb;
		3'd2: taken = fa == {27'd0, e_bimm};   // beqi
		3'd3: taken = fa != {27'd0, e_bimm};   // bnei
		3'd4: taken = $signed(fa) < $signed(fb);
		3'd5: taken = $signed(fa) >= $signed(fb);
		3'd6: taken = fa < fb;
		3'd7: taken = fa >= fb;
		default: taken = 1'b0;
	endcase
end

wire [31:0] e_addr   = fa + e_imm;
wire [31:0] e_btgt   = e_pc + e_imm;

// divider
reg        div_busy, div_done, div_init, div_negq, div_negr;
reg  [5:0] div_cnt;
reg [31:0] div_q, div_r, div_d, div_res;
wire       e_div_stall = e_valid && e_div && !div_done;

// pipeline control. stall_m / stall_e come from registers only, two LUTs
// deep: they hold the E and M registers and, through if_adv, the I$ read
// enables. m_wait: M holds p8.hfind, p8.zline or FENCE.I.
reg  m_wait;
wire stall_m = m_wait && !dm_done;
wire stall_e = stall_m || e_div_stall;

// late: a load or store in M that must be replayed
wire replay = m_valid && (m_load || m_store) && !dm_ready;

// a predicted JALR that did not go where rs1 says (target rs1 & ~1), from
// registers
wire m_jmiss    = m_valid && m_jp && m_mul_a[31:1] != {7'd0, m_ptgt, 1'b0};
wire redirect_m = (m_valid && m_fencei && dm_done) || m_jmiss;
// a predicted JALR (offset 0) is checked in M (m_jmiss)
wire e_mispred  = (e_jalr && !e_jp) || (e_branch && (taken != e_pred));
wire redirect_e = e_valid && !stall_e && e_mispred;
// a branch only redirects when mispredicted, so its target follows from
// the prediction (keeps the compare off the target)
wire [31:0] e_target = e_jalr ? {e_addr[31:1], 1'b0} : e_pred ? e_pc4 : e_btgt;

// registered redirect: fetch the target next cycle, squash what is in D.
// M (replay, FENCE.I) is older than E.
// The M and E targets are registered separately and selected afterwards
// (replay and the JALR target are both late).
reg        rd_m;
reg [31:0] rd_tgt_m, rd_tgt_e;
always @(posedge clk) begin
	rd_pend  <= !reset && (replay || redirect_m || redirect_e);
	rd_m     <= replay || redirect_m;
	rd_tgt_m <= m_fencei ? m_pc4 : m_jp ? {m_mul_a[31:1], 1'b0} : m_pc;
	rd_tgt_e <= e_target;
end
wire [31:0] rd_tgt = rd_m ? rd_tgt_m : rd_tgt_e;

wire kill_e   = replay || redirect_m;   // E's instruction does not go on
wire d_ok     = if_valid && !rd_pend && !kill_e;
wire hazard_d = if_valid && late_rd != 5'd0 && (if_way ? haz1 : haz0);
wire stall_d  = stall_e || hazard_d;

// d_pc + 4, kept with d_pc (off the fetch-address path)
reg [31:0] d_pc4;
always @* begin
	if (reset)        if_addr = 32'd0;
	else if (rd_pend) if_addr = rd_tgt;
	else if (d_jp)    if_addr = {7'd0, d_jtgt, 2'b00};
	else if (d_pred)  if_addr = d_target;
	else              if_addr = d_pc4;
end
assign if_adv   = reset || rd_pend || (if_valid && !stall_d);
assign if_redir = reset || rd_pend;

always @(posedge clk) if (if_adv) begin
	d_pc  <= if_addr;
	d_pc4 <= if_addr + 32'd4;
end

// branch predictor table: read with the fetch address when the I$ is
// (its output holds for d_pc otherwise); written a cycle after a branch
// leaves E (registered: `taken` is late)
reg       bht_we;
reg [9:0] bht_wa;
wire [1:0] bht_wd;
p8_bram #(.AW(10), .DW(2), .BYPASS(0)) bht (
	.clk(clk), .re(if_adv), .raddr(if_addr[11:2]), .rdata(), .rdata_raw(bht_q),
	.we(bht_we), .waddr(bht_wa), .wdata(bht_wd)
);
// JALR target buffer: read like the counters, written a cycle after a
// predicted JALR leaves M
reg        btb_we;
reg  [7:0] btb_wa;
reg [22:0] btb_wd;
p8_bram #(.AW(8), .DW(23), .BYPASS(0)) btb (
	.clk(clk), .re(if_adv), .raddr(if_addr[9:2]), .rdata(), .rdata_raw(btb_q),
	.we(btb_we), .waddr(btb_wa), .wdata(btb_wd)
);
always @(posedge clk) begin
	btb_we <= !reset && m_valid && m_jp && !replay;
	btb_wa <= m_pc[9:2];
	btb_wd <= m_mul_a[24:2];
end

// return-address stack: 8 entries, updated as an instruction leaves D (not
// on the fetch path: the top is a register of its own). Wrong-path
// instructions that leave D before a redirect squashes them can change it:
// that costs mispredicts only (M checks every prediction).
reg [22:0] ras [0:7];
reg  [2:0] ras_sp;   // index of the top
wire       d_leave  = d_ok && !hazard_d && !stall_e;
wire       d_link   = (d_jal || d_jalr) && (d_rd == 5'd1 || d_rd == 5'd5);
always @(posedge clk) begin
	if (d_leave && d_link) begin
		// (a JALR that links: push; a return that also links is rare)
		ras[ras_sp + 3'd1] <= d_pc4[24:2];
		ras_sp  <= ras_sp + 3'd1;
		ras_top <= d_pc4[24:2];
	end else if (d_leave && d_jp && d_ret) begin
		ras_sp  <= ras_sp - 3'd1;
		ras_top <= ras[ras_sp - 3'd1];
	end
	if (reset) ras_sp <= 3'd0;
end

// counter: 0 strongly agree .. 3 strongly disagree with the static rule.
// Only the outcome is registered behind the (forwarded) compare; the new
// counter comes from registers at the RAM's write port.
reg        bht_agree;
reg  [1:0] bht_old;
always @(posedge clk) begin
	bht_we    <= !reset && e_valid && e_branch && !stall_e;
	bht_wa    <= e_pc[11:2];
	bht_agree <= taken == e_st;
	bht_old   <= e_bdis;
end
assign bht_wd = bht_agree ? (bht_old == 2'd0 ? 2'd0 : bht_old - 2'd1) : (bht_old == 2'd3 ? 2'd3 : bht_old + 2'd1);

assign de_addr = e_addr;
// (e_v, not e_valid: a load squashed by a redirect may take the shared
// read port for a cycle, harmless, and the redirect stays off the port's
// address path)
assign de_load = e_v && e_load;

// The shared port's address has its own add, with the other requester's
// address muxed into the operands instead of after the sum: the late
// forwarded base then goes through one LUT (forwarding and the select) and
// the carry chain, and straight to the RAM blocks, which are spread out
// (64 KiB in byte lanes).
wire [15:0] ra_base = ra_cpu ? fa[15:0] : 16'd0;
wire [15:0] ra_off  = ra_cpu ? e_imm[15:0] : {ra_other, 2'b00};
wire [15:0] ra_sum  = ra_base + ra_off;
assign ra_addr = ra_sum[15:2];

// ---------------------------------------------------------------- D -> E

// forwarding for the D instruction: M's instruction next cycle is E's now
// (if it gets there), W's is M's
wire e_to_m_fwd = e_v && e_wen && !e_late && e_rd != 5'd0;
wire m_to_w_wr  = m_valid && m_wen && m_rd != 5'd0;

// Operands and forwarding selects hold only while M waits (stall_m): a
// stalled divide has latched its operands (div_a/div_b), so they reload
// from D every cycle and the last load, when the divide leaves E, counts.
// This keeps the divider's stall off the 64 operand muxes.
always @(posedge clk) begin
	if (stall_m) begin
		// hold; keep forwarded operands since their producers move on.
		// While M waits, W is empty next cycle and M keeps its result, so
		// only an M-to-M forward stays.
		e_a   <= fa;
		e_b   <= fb;
		e_fam <= 1'b0;
		e_faw <= 1'b0;
		e_fbm <= 1'b0;
		e_fbw <= 1'b0;
	end else begin
		e_fam <= d_use1 && e_to_m_fwd && e_rd == d_rs1;
		e_faw <= d_use1 && m_to_w_wr && m_rd == d_rs1;
		e_fbm <= d_use2 && e_to_m_fwd && e_rd == d_rs2;
		e_fbw <= d_use2 && m_to_w_wr && m_rd == d_rs2;
		e_a   <= rf_a;
		e_b   <= rf_b;
	end
end

// kill_e (a replay, late) only clears e_v; it is not in the enable of
// the other E registers.
always @(posedge clk) begin
	if (!stall_e) begin
		e_v       <= d_ok && !hazard_d;
		e_pc      <= d_pc;
		e_imm     <= d_imm;
		e_rd      <= d_rd;
		e_wen     <= d_wen;
		e_f3      <= (d_op | d_opimm | d_branch | d_load | d_store | d_csr) ? f3 : 3'd0;
		e_alt     <= (d_op && di[30]) || (d_opimm && f3 == 3'd5 && di[30]);
		e_op1pc   <= d_auipc;
		e_op1zero <= d_lui;
		e_op2imm  <= !d_op;
		e_branch  <= d_branch;
		e_jal     <= d_jal;
		e_jalr    <= d_jalr;
		e_load    <= d_load;
		e_store   <= d_store;
		e_mul     <= d_muldiv && !f3[2];
		e_div     <= d_muldiv && f3[2];
		e_fencei  <= d_fencei;
		e_pred    <= d_pred;
		e_st      <= d_st;
		e_bdis    <= bht_q;
		e_zbb     <= d_zbb;
		e_zop     <= d_zop;
		e_late    <= d_load || d_muldiv || d_hfind || d_zbb || d_csr;
		e_pc4     <= d_pc4;
		e_csr     <= d_csr;
		e_hfind   <= d_hfind;
		e_hnum    <= di[25];
		e_jp      <= d_jp;
		e_ptgt    <= d_tgt23;
		e_bimm    <= d_rs2;
		e_zline   <= d_zline;
		e_zimm    <= d_rs1;
	end
	if (reset || kill_e) e_v <= 1'b0;
end

// ---------------------------------------------------------------- divider

// The operands are registered first (div_a/div_b), off the forwarding path.
reg  [31:0] div_a, div_b;
wire        div_sgn = !e_f3[0];
wire [31:0] div_aa  = (div_sgn && div_a[31]) ? -div_a : div_a;
wire [31:0] div_ba  = (div_sgn && div_b[31]) ? -div_b : div_b;
wire [32:0] div_sh  = {div_r, div_q[31]};
wire [32:0] div_df  = div_sh - {1'b0, div_d};

always @(posedge clk) begin
	if (div_busy) begin
		if (!div_df[32]) begin
			div_r <= div_df[31:0];
			div_q <= {div_q[30:0], 1'b1};
		end else begin
			div_r <= div_sh[31:0];
			div_q <= {div_q[30:0], 1'b0};
		end
		div_cnt <= div_cnt - 6'd1;
		if (div_cnt == 6'd1) begin
			div_busy <= 1'b0;
			div_done <= 1'b1;
		end
	end else if (div_done) begin
		// result is consumed when the instruction leaves E
		if (!stall_m) div_done <= 1'b0;
	end else if (div_init) begin
		div_init <= 1'b0;
		div_negq <= 1'b0;
		div_negr <= 1'b0;
		if (div_b == 32'd0) begin
			// x/0 = all ones, x%0 = x
			div_q    <= 32'hffffffff;
			div_r    <= div_a;
			div_done <= 1'b1;
		end else begin
			div_q    <= div_aa;
			div_r    <= 32'd0;
			div_d    <= div_ba;
			div_negq <= div_sgn && (div_a[31] ^ div_b[31]);
			div_negr <= div_sgn && div_a[31];
			div_cnt  <= 6'd32;
			div_busy <= 1'b1;
		end
	end else if (e_valid && e_div) begin
		div_a    <= fa;
		div_b    <= fb;
		div_init <= 1'b1;
	end
	if (reset || kill_e) begin
		div_busy <= 1'b0;
		div_done <= 1'b0;
		div_init <= 1'b0;
	end
end

always @* div_res = e_f3[1] ? (div_negr ? -div_r : div_r) : (div_negq ? -div_q : div_q);

// ---------------------------------------------------------------- E -> M

// ---------------------------------------------------------------- CSRs

reg [63:0] cnt_cycle, cnt_instret;
reg [31:0] mscratch;
reg [31:0] csr_old;
wire [11:0] csr_a = e_imm[11:0];
// CSR writes happen when the instruction is in M (it cannot be killed
// there; kill_e is late), forwarded to a CSR read in E
reg        m_csr_we;
reg [11:0] m_csr_a;
reg [31:0] m_csr_v;
wire [63:0] instret_e = cnt_instret + m_valid + w_valid;  // older instructions in flight
always @* begin
	case (csr_a)
		12'hc00, 12'hc01, 12'hb00: csr_old = cnt_cycle[31:0];
		12'hc80, 12'hc81, 12'hb80: csr_old = cnt_cycle[63:32];
		12'hc02, 12'hb02:          csr_old = instret_e[31:0];
		12'hc82, 12'hb82:          csr_old = instret_e[63:32];
		12'h301:                   csr_old = 32'h40001100;   // misa: RV32IM
		12'h340:                   csr_old = mscratch;
		12'h7c0:                   csr_old = p8heap;
		12'h7c1:                   csr_old = 32'd7;          // p8feat
		default:                   csr_old = 32'd0;
	endcase
	if (m_valid && m_csr_we && m_csr_a == csr_a) csr_old = m_csr_v;
end
wire [31:0] csr_src = e_f3[2] ? {27'd0, e_zimm} : fa;
wire        csr_wr  = e_f3[1:0] == 2'd1 || e_zimm != 5'd0;
wire [31:0] csr_new = e_f3[1:0] == 2'd1 ? csr_src : e_f3[1:0] == 2'd2 ? csr_old | csr_src : csr_old & ~csr_src;

always @(posedge clk) begin
	if (reset) begin
		cnt_cycle   <= 64'd0;
		cnt_instret <= 64'd0;
		mscratch    <= 32'd0;
		p8heap      <= 32'd0;
	end else begin
		cnt_cycle <= cnt_cycle + 64'd1;
		if (w_valid) cnt_instret <= cnt_instret + 64'd1;
		if (m_valid && m_csr_we && m_csr_a == 12'h340) mscratch <= m_csr_v;
		if (m_valid && m_csr_we && m_csr_a == 12'h7c0) p8heap <= m_csr_v;
	end
end

// divider and CSR results go to W from M (m_dres, late: e_late), keeping
// this mux, on the forwarding loop, to two inputs
wire [31:0] e_result = (e_jal | e_jalr) ? e_pc4 : alu;

reg  [3:0] e_be;
reg [31:0] e_wd;
always @* begin
	case (e_f3[1:0])
		2'd0: begin e_be = 4'b0001 << e_addr[1:0]; e_wd = {4{fb[7:0]}}; end
		2'd1: begin e_be = e_addr[1] ? 4'b1100 : 4'b0011; e_wd = {2{fb[15:0]}}; end
		default: begin e_be = 4'b1111; e_wd = fb; end
	endcase
end

always @(posedge clk) begin
	if (reset) begin
		m_valid <= 1'b0;
		m_wait  <= 1'b0;
	end else if (!stall_m) begin
		m_valid  <= e_valid && !e_div_stall && !kill_e;
		m_wait   <= e_valid && !kill_e && (e_fencei || e_hfind || e_zline);
		m_pc     <= e_pc;
		m_result <= e_result;
		m_addr   <= e_addr;
		m_wdata  <= e_wd;
		m_be     <= e_be;
		m_rd     <= e_rd;
		m_wen    <= e_wen;
		m_load   <= e_load;
		m_store  <= e_store;
		m_mul    <= e_mul;
		m_fencei <= e_fencei;
		m_hfind  <= e_hfind;
		m_hnum   <= e_hnum;
		m_jp     <= e_jalr && e_jp;
		m_ptgt   <= e_ptgt;
		m_zline  <= e_zline;
		m_hkey   <= fb;
		m_f3     <= e_f3;
		m_mul_a  <= fa;
		m_mul_b  <= fb;
		m_zb_b   <= op2;
		m_zbb    <= e_zbb;
		m_zop    <= e_zop;
		m_dc     <= e_div || e_csr;
		m_dres   <= e_div ? div_res : csr_old;
		m_ldsel  <= e_load;
		m_csr_we <= e_csr && csr_wr;
		m_csr_a  <= csr_a;
		m_csr_v  <= csr_new;
	end
end

assign dm_addr   = m_addr;
assign dm_rd     = m_valid && m_load;
assign dm_wr     = m_valid && m_store;
assign dm_be     = m_be;
assign dm_wdata  = m_wdata;
assign dm_fencei = m_valid && m_fencei;
assign dm_hfind  = m_valid && m_hfind;
assign dm_hnum   = m_hnum;
assign dm_zline  = m_valid && m_zline;
assign dm_hkey   = m_hkey;

// ---------------------------------------------------------------- M: multiply

// f3: 0 MUL, 1 MULH (s*s), 2 MULHSU (s*u), 3 MULHU (u*u)
wire signed [32:0] mul_a = {(m_f3[1:0] != 2'd3) & m_mul_a[31], m_mul_a};
wire signed [32:0] mul_b = {(m_f3[1:0] == 2'd1) & m_mul_b[31], m_mul_b};
wire signed [65:0] mul_p = mul_a * mul_b;
wire [31:0] mul_res = (m_f3[1:0] == 2'd0) ? mul_p[31:0] : mul_p[63:32];

// ---------------------------------------------------------------- M: Zbb

// from registered operands (m_mul_a = rs1, m_zb_b = rs2 or the immediate)
wire [31:0] za = m_mul_a, zb = m_zb_b;
function automatic [5:0] clz32(input [31:0] x);
	integer i;
	begin
		clz32 = 6'd32;
		for (i = 0; i < 32; i = i + 1) if (x[i]) clz32 = 6'd31 - i[5:0];
	end
endfunction
function automatic [5:0] cpop32(input [31:0] x);
	integer i;
	begin
		cpop32 = 6'd0;
		for (i = 0; i < 32; i = i + 1) cpop32 = cpop32 + {5'd0, x[i]};
	end
endfunction
function automatic [31:0] rev32(input [31:0] x);
	integer i;
	begin
		for (i = 0; i < 32; i = i + 1) rev32[i] = x[31 - i];
	end
endfunction
wire        z_lt  = $signed(za) < $signed(zb);
wire        z_ltu = za < zb;
wire [63:0] z_rl  = {za, za} << zb[4:0];
wire [63:0] z_rr  = {za, za} >> zb[4:0];
reg  [31:0] zbb_res;
always @* begin
	case (m_zop)
		Z_ANDN:  zbb_res = za & ~zb;
		Z_ORN:   zbb_res = za | ~zb;
		Z_XNOR:  zbb_res = ~(za ^ zb);
		Z_CLZ:   zbb_res = {26'd0, clz32(za)};
		Z_CTZ:   zbb_res = {26'd0, clz32(rev32(za))};
		Z_CPOP:  zbb_res = {26'd0, cpop32(za)};
		Z_SEXTB: zbb_res = {{24{za[7]}}, za[7:0]};
		Z_SEXTH: zbb_res = {{16{za[15]}}, za[15:0]};
		Z_ZEXTH: zbb_res = {16'd0, za[15:0]};
		Z_MAX:   zbb_res = z_lt ? zb : za;
		Z_MAXU:  zbb_res = z_ltu ? zb : za;
		Z_MIN:   zbb_res = z_lt ? za : zb;
		Z_MINU:  zbb_res = z_ltu ? za : zb;
		Z_ROL:   zbb_res = z_rl[63:32];
		Z_ROR:   zbb_res = z_rr[31:0];
		Z_ORCB:  zbb_res = {{8{|za[31:24]}}, {8{|za[23:16]}}, {8{|za[15:8]}}, {8{|za[7:0]}}};
		default: zbb_res = {za[7:0], za[15:8], za[23:16], za[31:24]};   // rev8
	endcase
end

// ---------------------------------------------------------------- M -> W

wire [31:0] m_pc4 = m_pc + 32'd4;

// load alignment (p8.hfind answers a word), applied to both candidate
// words in parallel with the D$ tag compare that selects between them
function automatic [31:0] ld_align(input [31:0] d, input [1:0] sz, input sx, input [1:0] lo);
	reg [7:0] b;
	reg [15:0] h;
	begin
		case (lo)
			2'd0: b = d[7:0];
			2'd1: b = d[15:8];
			2'd2: b = d[23:16];
			default: b = d[31:24];
		endcase
		h = lo[1] ? d[31:16] : d[15:0];
		case (sz)
			2'd0: ld_align = {{24{sx & b[7]}}, b};
			2'd1: ld_align = {{16{sx & h[15]}}, h};
			default: ld_align = d;
		endcase
	end
endfunction

wire  [1:0] m_ldsz = m_hfind ? 2'd2 : m_f3[1:0];
wire [31:0] m_ld0  = ld_align(dm_rdata0, m_ldsz, ~m_f3[2], m_addr[1:0]);
wire [31:0] m_ld1  = ld_align(dm_rdata1, m_ldsz, ~m_f3[2], m_addr[1:0]);
wire [31:0] m_ld   = dm_rsel1 ? m_ld1 : m_ld0;

// The early results (ALU, MUL, Zbb, p8.hfind: registers and the DSP) are selected
// first and kept as one node, so the late load data (D$ way select,
// alignment) meets a single 2:1 select into w_val.
(* keep = 1 *) wire [31:0] m_other = m_hfind ? dm_hres : m_mul ? mul_res : m_zbb ? zbb_res : m_dc ? m_dres : m_result;

// stall_m only makes W a bubble; the other W registers load every cycle
always @(posedge clk) begin
	w_valid <= !reset && !stall_m && m_valid && !replay;
	w_rd    <= m_rd;
	w_wen   <= m_wen;
	w_val   <= m_ldsel ? m_ld : m_other;
end

endmodule
