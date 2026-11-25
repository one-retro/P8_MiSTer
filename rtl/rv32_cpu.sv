// RV32IM + Zbb pipeline for the P8 soft CPU.
//
// Stages: D (instruction arrives from the I$, decode, register read),
// E (ALU, branches, divider), M (data memory, load alignment), W (write
// back). The fetch address is chosen combinationally every cycle and
// registered by the I$ RAM, so there is no separate F stage register.
//
// Late signals (I$ hit, D$ hit, the I$ outputs) never select an address or
// gate the pipeline registers or the I$ read enables. An I$ miss only holds
// d_pc (if_adv = 0). A D$ miss is not a stall: the access is replayed
// (the instruction does not retire, the younger ones are dropped and
// fetching restarts at it, like a mispredict) until it hits. The only
// pipeline stalls come from registered state: p8.hfind, p8.zline and
// FENCE.I waiting in M, the divider in E, and a load-use hazard: found in
// D and registered (e_hz), the instruction waits a cycle in E.
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
//   so the ALU result path does not grow (min / max take the compare from
//   the ALU's slt[u] in E); the next instruction waits a cycle for the
//   result.
// - DIV/REM: radix-4 restoring divider in E, 2 bits a cycle (operands
//   registered first, then the absolute values, then 3 x the divisor; 19
//   cycles in E).
// - FENCE.I waits in M for the memory system (D$ write-back + I$
//   invalidate), then refetches the next instruction.
// - CSRs: cycle/time/instret (and *h) read-only, mscratch read-write,
//   performance counters hpmcounter16-31 (0xc10-0xc1f) read-only; other
//   CSRs read 0. No traps or interrupts: ECALL/EBREAK/FENCE are
//   no-ops.
//   Misaligned accesses are not supported (the compilers never emit them).
// - p8.hfind rd, rs1, rs2 (custom-0, funct3 0, funct7 0: a string key, 1: a
//   number key): table lookup done by the memory system (p8_tunit); waits
//   in M like a load. CSR 0x7c0 (p8heap) holds the heap arena base for it.
// - beqi / bnei rs1, imm5, offset (BRANCH funct3 2 / 3): compare rs1 with
//   the unsigned immediate in the rs2 field.
// - p8.fdiv rd, rs1, rs2 (custom-0, funct3 2, funct7 0): the PICO-8
//   division of 16.16 numbers: rs2 = 1.0 gives rs1; otherwise
//   floor(|rs1| * 2^16 / |rs2|) with the sign of rs1 ^ rs2 when it is
//   below 2^31; else (rs2 = 0, overflow) 0x7fffffff if the signs agree,
//   else 0x80000001. On the divider, like DIV (the partial remainder starts
//   at |rs1| >> 16: the quotient's first 16 bits are 0 when it fits).
// - p8.stv rs2, imm(rs1) (custom-1, S-type, funct3 = a tag 0-7): the
//   8-byte value {tag, rs2} (a Lua value: tag word, then payload) at an
//   8-byte aligned address in SDRAM, one D$ write.
// - CSR 0x7c1 (p8feat, read-only) lists these extensions: 1 number-key
//   p8.hfind, 2 beqi/bnei, 4 JALR prediction, 8 p8.fdiv, 16 p8.stv
//   (earlier cores read 0).
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
	output            if_adv,    // d_pc <= if_addr (late: the I$ hit)
	output            if_ren,    // read the I$ at if_addr, else it keeps its output (from registers)
	input             if_reread, // the I$ re-reads (S_DONE): if_addr is d_pc unless redirected
	input      [31:0] if_data,   // instruction at d_pc (the address last fetched)
	// the same word from both I$ ways, its predecode (from the line fill:
	// {branch, predict taken, uses rs1, uses rs2, target[24:2]}) and the way
	// `if_data` comes from: the next-fetch logic (prediction, load-use
	// hazard) looks at both and selects late, keeping the way mux off the
	// fetch-address path
	input      [31:0] if_data0,
	input      [31:0] if_data1,
	input      [28:0] if_pre0,
	input      [28:0] if_pre1,
	// the way, in copies of the I$'s MRU RAM (one RAM output cannot drive
	// all of D): 0 the fetch address and predecode, 1 the immediate, 2 / 3
	// the rs1 / rs2 read
	input       [3:0] if_way,
	input             if_valid,

	// data memory
	output     [13:0] de_addr,   // E-stage effective address, bits 13-0 (the D$ read address)
	output     [13:0] ra_addr,   // its word address in the PICO-8 RAM (the CPU's copy)
	output     [31:0] dm_addr,
	output            dm_rd,
	output            dm_wr,
	output      [3:0] dm_be,
	output     [31:0] dm_wdata,
	output            dm_fencei,
	output            dm_hfind,    // M holds p8.hfind: table dm_addr, key dm_hkey
	output            dm_hnum,     // ... of a number key
	output            dm_zline,    // M holds p8.zline: line dm_addr
	output            dm_stv,      // the store is p8.stv: tag dm_tag, then dm_wdata
	output      [2:0] dm_tag,
	output     [31:0] dm_hkey,
	output reg [31:0] p8heap,
	input      [31:0] dm_rdata0,   // load data: dm_rsel1 ? dm_rdata1 : dm_rdata0
	input      [31:0] dm_rdata1,   // (dm_rsel1 is late: the D$ hit way) ...
	input             dm_rsel1,
	input      [31:0] dm_pdata,    // ... or outside DDR the PICO-8 RAM's word
	input      [31:0] dm_mdata,    // or the MMIO value (a register)
	input             dm_p8,       // dm_addr is in the PICO-8 RAM / elsewhere (MMIO;
	input             dm_mmio,     // both decoded from the registered address)
	input             dm_ready,    // load/store answered (else replay)
	input      [31:0] dm_hres,     // p8.hfind result (a register, valid with dm_done)
	input             dm_done,     // hfind/zline/fence.i finished (registered pulses, ORed)

	// events of the memory system for the performance counters 0-7 (see
	// the CSRs; registers)
	input       [7:0] pm_ev
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
localparam OP_CUST1  = 7'b0101011;

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
reg        e_shl, e_shr;   // e_f3 is 1 / 5: a shift (it has its own path into the result)
reg        e_cmp;          // e_f3 is 2 / 3: slt[u] (likewise)
reg        e_alt, e_op1pc, e_op1zero, e_op2imm;
reg        e_branch, e_jal, e_jalr, e_load, e_store, e_mul, e_div, e_fencei, e_pred, e_csr, e_hfind, e_zline;
// a CSR read: which CSR (decoded in D), and whether the instruction ahead
// (in E when this one is in D, in M when it is in E) writes it
reg  [3:0] e_csrk;
reg        e_hpm;          // a performance counter (e_imm[3:0])
reg        e_csrf;
reg        e_hnum;         // p8.hfind of a number key
reg        e_fdv;          // p8.fdiv (e_div; e_f3 0: signed, the quotient)
reg        e_stv;          // p8.stv (e_store, a word: e_f3 2), its tag
reg  [2:0] e_tag;
reg        e_jp;           // a predicted JALR (offset 0) ...
reg [22:0] e_ptgt;         // ... and its predicted target [24:2]
reg  [4:0] e_bimm;         // beqi / bnei immediate
reg        e_zbb;
reg        e_late;         // its result comes from M or W (load, mul, div, CSR, hfind, Zbb)
// funct3 bit 0 (the compare inverted) ^ the prediction / the static rule
// (backward taken)
reg        e_bk, e_bs;
reg  [1:0] e_bdis;         // its predictor counter
reg  [4:0] e_zimm;
reg        e_fam, e_faw, e_fbm, e_fbw;   // forward operand a/b from M / W
// load-use hazard: E waits a cycle for the late result now in M (found in
// D, registered; the rs fields let E forward it from W after the wait)
reg        e_hz;
reg  [4:0] e_rs1, e_rs2;
reg        e_u1, e_u2;
reg [11:0] e_zh;           // Zbb: instruction bits 31-20, funct3, OP (else OP-IMM),
reg  [2:0] e_zf;           // decoded in E (e_zop)
reg        e_zo;

// M
reg        m_valid;
reg [31:0] m_pc, m_result, m_addr, m_wdata, m_mul_a, m_mul_b, m_hkey, m_zb_b;
reg  [3:0] m_be;
reg  [4:0] m_rd;
reg        m_wen, m_load, m_store, m_mul, m_fencei, m_hfind, m_hnum, m_zline, m_zbb, m_ldsel, m_dc, m_stv;
reg  [2:0] m_tag;
// load alignment, decoded in E from the size and the address's low bits
// (early in its add): the byte for bits 7-0, bits 15-8 from the upper half
// or the extension, bits 31-16 the extension, the byte whose bit 7 is the
// sign, sign extension
reg  [1:0] m_bsel, m_sidx;
reg        m_hup, m_hext, m_wext, m_sen;
reg        m_jp;           // a predicted JALR (rs1 in m_mul_a) ...
reg [22:0] m_ptgt;         // ... and its predicted target
reg [31:0] m_dres;         // divider / CSR result (late, like MUL)
reg  [4:0] m_zop;
reg        m_zrl, m_zrr;   // m_zop is rol / ror
reg  [2:0] m_f3;

// W
reg        w_valid;
reg  [4:0] w_rd;
reg        w_wen;
reg [31:0] w_val;


// ---------------------------------------------------------------- W

// load alignment happens on the way into W (M), so the W result is a
// register and forwarding from W has no logic in front of it
wire [31:0] w_result = w_val;
wire        w_write  = w_valid && w_wen && (w_rd != 5'd0);


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
wire d_fdiv   = opc == OP_CUST0 && f3 == 3'd2 && di[31:25] == 7'd0;
wire d_stv    = opc == OP_CUST1;

// Zbb: {is Zbb, operation (m_zop)} of an OP (op) or OP-IMM (opimm)
// instruction with bits 31-20 h and funct3 f. D needs only whether it is
// Zbb (a late result); E decodes the operation (e_zop), off the I$ output.
localparam Z_ANDN = 5'd0, Z_ORN = 5'd1, Z_XNOR = 5'd2, Z_CLZ = 5'd3, Z_CTZ = 5'd4, Z_CPOP = 5'd5,
           Z_SEXTB = 5'd6, Z_SEXTH = 5'd7, Z_ZEXTH = 5'd8, Z_MAX = 5'd9, Z_MAXU = 5'd10, Z_MIN = 5'd11,
           Z_MINU = 5'd12, Z_ROL = 5'd13, Z_ROR = 5'd14, Z_ORCB = 5'd15, Z_REV8 = 5'd16;
function automatic [5:0] zbb_of(input [11:0] h, input [2:0] f, input op, input opimm);
	begin
		zbb_of = {1'b1, Z_ANDN};
		if (op && h[11:5] == 7'b0100000 && f == 3'd7) zbb_of[4:0] = Z_ANDN;
		else if (op && h[11:5] == 7'b0100000 && f == 3'd6) zbb_of[4:0] = Z_ORN;
		else if (op && h[11:5] == 7'b0100000 && f == 3'd4) zbb_of[4:0] = Z_XNOR;
		else if (op && h[11:5] == 7'b0000101 && f == 3'd4) zbb_of[4:0] = Z_MIN;
		else if (op && h[11:5] == 7'b0000101 && f == 3'd5) zbb_of[4:0] = Z_MINU;
		else if (op && h[11:5] == 7'b0000101 && f == 3'd6) zbb_of[4:0] = Z_MAX;
		else if (op && h[11:5] == 7'b0000101 && f == 3'd7) zbb_of[4:0] = Z_MAXU;
		else if (op && h[11:5] == 7'b0110000 && f == 3'd1) zbb_of[4:0] = Z_ROL;
		else if (op && h[11:5] == 7'b0110000 && f == 3'd5) zbb_of[4:0] = Z_ROR;
		else if (op && h == 12'h080 && f == 3'd4) zbb_of[4:0] = Z_ZEXTH;
		else if (opimm && h == 12'h600 && f == 3'd1) zbb_of[4:0] = Z_CLZ;
		else if (opimm && h == 12'h601 && f == 3'd1) zbb_of[4:0] = Z_CTZ;
		else if (opimm && h == 12'h602 && f == 3'd1) zbb_of[4:0] = Z_CPOP;
		else if (opimm && h == 12'h604 && f == 3'd1) zbb_of[4:0] = Z_SEXTB;
		else if (opimm && h == 12'h605 && f == 3'd1) zbb_of[4:0] = Z_SEXTH;
		else if (opimm && h[11:5] == 7'b0110000 && f == 3'd5) zbb_of[4:0] = Z_ROR;   // rori
		else if (opimm && h == 12'h287 && f == 3'd5) zbb_of[4:0] = Z_ORCB;
		else if (opimm && h == 12'h698 && f == 3'd5) zbb_of[4:0] = Z_REV8;
		else zbb_of[5] = 1'b0;
	end
endfunction
wire [5:0] d_zd  = zbb_of(di[31:20], f3, d_op, d_opimm);
wire       d_zbb = d_zd[5];
// min[u] / max[u]: E's ALU does the compare (slt / sltu), M selects
wire       d_zmm = d_op && di[31:25] == 7'b0000101 && f3[2];
wire [2:0] d_f3  = d_zmm ? {2'b01, f3[0]} :   // min[u] / max[u]: slt[u]
                   (d_op | d_opimm | d_branch | d_load | d_store | d_csr) ? f3 :
                   d_stv ? 3'd2 : 3'd0;   // p8.stv: a word store (and its tag)

// uses rs1 / rs2: from the predecode (the I$ decodes them at the fill, for
// the hazard check), so the forwarding selects do not wait for the decode
// the CSR a read selects (e_csrk)
localparam K_ZERO = 4'd0, K_CYC = 4'd1, K_CYCH = 4'd2, K_INS = 4'd3, K_INSH = 4'd4,
           K_MISA = 4'd5, K_MSCR = 4'd6, K_HEAP = 4'd7, K_FEAT = 4'd8;
function automatic [3:0] csr_of(input [11:0] a);
	case (a)
		12'hc00, 12'hc01, 12'hb00: csr_of = K_CYC;
		12'hc80, 12'hc81, 12'hb80: csr_of = K_CYCH;
		12'hc02, 12'hb02:          csr_of = K_INS;
		12'hc82, 12'hb82:          csr_of = K_INSH;
		12'h301:                   csr_of = K_MISA;
		12'h340:                   csr_of = K_MSCR;
		12'h7c0:                   csr_of = K_HEAP;
		12'h7c1:                   csr_of = K_FEAT;
		default:                   csr_of = K_ZERO;
	endcase
endfunction

wire d_use1 = if_way[0] ? if_pre1[24] : if_pre0[24];
wire d_use2 = if_way[0] ? if_pre1[23] : if_pre0[23];   // not beqi/bnei
wire d_wen  = d_lui | d_auipc | d_jal | d_jalr | d_load | d_opimm | d_op | d_csr | d_hfind | d_fdiv;

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
		else if (o == OP_STORE || o == OP_CUST1)   // and p8.stv
			imm_of = {{20{x[31]}}, x[31:25], x[11:7]};
		else if (o == OP_CUST0 && x[14:13] == 2'd0 && x[31:26] == 6'd0)
			imm_of = 32'd0;   // p8.hfind, p8.zline
		else
			imm_of = {{20{x[31]}}, x[31:20]};
	end
endfunction
wire [31:0] d_imm = if_way[1] ? imm_of(if_data1) : imm_of(if_data0);

// Register read for the word of each I$ way in parallel, straight from the
// RAM outputs (the I$ RAMs select the word), selected by the way last: the
// way is late. The register file is one LUT RAM (MLAB) per read port (rs1
// and rs2 of each way), all written by W: a read is about a LUT's delay
// after the index, where registers need a 32:1 mux tree whose selects (the
// index bits) drive 160 LUTs. W's write in the same cycle is forwarded.
// (the indices are kept as nodes of their own: synthesis would otherwise
// take the I$ RAM's output register as the LUT RAMs' address register, and
// the I$ RAMs could not be inferred)
(* keep = 1 *) wire [4:0] ri_a0 = if_data0[19:15];
(* keep = 1 *) wire [4:0] ri_a1 = if_data1[19:15];
(* keep = 1 *) wire [4:0] ri_b0 = if_data0[24:20];
(* keep = 1 *) wire [4:0] ri_b1 = if_data1[24:20];
wire [31:0] rf_a0, rf_a1, rf_b0, rf_b1;
p8_lutram #(.AW(5), .DW(32)) rfa0 (.clk(clk), .raddr(ri_a0), .rdata(rf_a0), .we(w_write), .waddr(w_rd), .wdata(w_result));
p8_lutram #(.AW(5), .DW(32)) rfa1 (.clk(clk), .raddr(ri_a1), .rdata(rf_a1), .we(w_write), .waddr(w_rd), .wdata(w_result));
p8_lutram #(.AW(5), .DW(32)) rfb0 (.clk(clk), .raddr(ri_b0), .rdata(rf_b0), .we(w_write), .waddr(w_rd), .wdata(w_result));
p8_lutram #(.AW(5), .DW(32)) rfb1 (.clk(clk), .raddr(ri_b1), .rdata(rf_b1), .we(w_write), .waddr(w_rd), .wdata(w_result));
function automatic [31:0] rf_rd(input [4:0] ri, input [31:0] v);
	rf_rd = (ri == 5'd0) ? 32'd0 : (w_write && w_rd == ri) ? w_result : v;
endfunction
wire [31:0] rf_a = if_way[2] ? rf_rd(if_data1[19:15], rf_a1) : rf_rd(if_data0[19:15], rf_a0);
wire [31:0] rf_b = if_way[3] ? rf_rd(if_data1[24:20], rf_b1) : rf_rd(if_data0[24:20], rf_b0);

// Prediction (see the top): the I$ predecodes {branch, static prediction,
// target} (code lives below 32 MiB, so the target has no bits above 24);
// the predictor table is read with the fetch address, so its counter for
// the D instruction arrives with it. Per way: static ^ (branch & disagree),
// selected by the late way last (one LUT, like the static prediction).
wire  [1:0] bht_q;
// e_rd of a result that is late (load, mul, hfind, Zbb): the D instruction
// that reads it waits a cycle in E (hazard_d, registered in e_hz)
wire  [4:0] late_rd = (e_v && e_late) ? e_rd : 5'd0;
wire haz0 = (if_pre0[24] && if_data0[19:15] == late_rd) || (if_pre0[23] && if_data0[24:20] == late_rd);
wire haz1 = (if_pre1[24] && if_data1[19:15] == late_rd) || (if_pre1[23] && if_data1[24:20] == late_rd);

wire        d_pred   = if_way[0] ? if_pre1[25] ^ (if_pre1[26] & bht_q[1]) : if_pre0[25] ^ (if_pre0[26] & bht_q[1]);
wire        d_st     = if_way[0] ? if_pre1[25] : if_pre0[25];
// a predicted JALR's target: the return stack's top (a register) or the
// target buffer's entry (read with the fetch address, like bht_q). It is
// its own input of the fetch-address mux (d_jp implies d_pred), so the
// predecoded target keeps a single way select in front of that mux.
reg  [22:0] ras_top;
wire [22:0] btb_q;
wire        d_jp     = if_way[0] ? if_pre1[28] : if_pre0[28];
wire        d_ret    = RAS && (if_way[0] ? if_pre1[27] : if_pre0[27]);
wire [22:0] d_jtgt   = d_ret ? ras_top : btb_q;
wire [31:0] d_target = {7'd0, if_way[0] ? if_pre1[22:0] : if_pre0[22:0], 2'b00};
wire [22:0] d_tgt23  = d_jp ? d_jtgt : d_target[24:2];

// ---------------------------------------------------------------- E

// Forwarding, decided in D for the instructions that will be in M and W
// when the D instruction is in E (registers e_f*). x0 and unused operands
// never match (rf_a/rf_b are 0 for x0).
wire [31:0] fa = e_fam ? m_result : e_faw ? w_result : e_a;
wire [31:0] fb = e_fbm ? m_result : e_fbw ? w_result : e_b;
// The forwarded base also feeds the D$ and PICO-8 RAM address adds, each
// in front of RAM blocks spread over the chip: they get copies of their
// own (kept), placed next to their adders, rather than sharing fa with the
// ALU, the branch compare and the M registers (~20 loads per bit).
(* keep = 1 *) wire [13:0] fa_d = e_fam ? m_result[13:0] : e_faw ? w_result[13:0] : e_a[13:0];
(* keep = 1 *) wire [15:0] fa_p = e_fam ? m_result[15:0] : e_faw ? w_result[15:0] : e_a[15:0];

wire [31:0] op1 = e_op1pc ? e_pc : e_op1zero ? 32'd0 : fa;
wire [31:0] op2 = e_op2imm ? e_imm : fb;

// all but the shifts and compares (below)
reg [31:0] alu;
always @* begin
	case (e_f3)
		3'd4: alu = op1 ^ op2;
		3'd6: alu = op1 | op2;
		3'd7: alu = op1 & op2;
		default: alu = e_alt ? op1 - op2 : op1 + op2;
	endcase
end

// The shifts are the ALU's deepest path (a 32:1 select per bit): they take
// their operands from copies of their own (kept; the value is always rs1,
// never pc or 0) and the left and right shifts enter the last select of
// the result directly (e_result), after the others have met theirs.
(* keep = 1 *) wire [31:0] sh_a  = e_fam ? m_result : e_faw ? w_result : e_a;
(* keep = 1 *) wire  [4:0] sh_nl = e_op2imm ? e_imm[4:0] : e_fbm ? m_result[4:0] : e_fbw ? w_result[4:0] : e_b[4:0];
(* keep = 1 *) wire  [4:0] sh_nr = e_op2imm ? e_imm[4:0] : e_fbm ? m_result[4:0] : e_fbw ? w_result[4:0] : e_b[4:0];
wire [31:0] sh_l = sh_a << sh_nl;
wire [32:0] sh_r = $signed({e_alt & sh_a[31], sh_a}) >>> sh_nr;   // srl / sra

// slt[u] (and min / max): a subtraction's carry chain, on operands of its
// own (kept), into bit 0 of the last select (as a compare, synthesis built
// a deeper LUT tree). The borrow is the unsigned compare.
(* keep = 1 *) wire [31:0] cm_a = e_fam ? m_result : e_faw ? w_result : e_a;
(* keep = 1 *) wire [31:0] cm_b = e_op2imm ? e_imm : e_fbm ? m_result : e_fbw ? w_result : e_b;
wire [32:0] cm_d  = {1'b0, cm_a} - {1'b0, cm_b};
wire        cm_lt = e_f3[0] ? cm_d[32] : (cm_a[31] ^ cm_b[31]) ? cm_a[31] : cm_d[31];

// Branches: taken = funct3[0] ^ (eq, eqi (beqi / bnei), lt or ltu by
// funct3[2:1]). The compare has operands of its own (kept), the order
// compares are a subtraction's carry chain, and funct3[0] is folded with
// the prediction (e_bk) and the static rule (e_bs) in D, so a mispredict
// is the selected compare ^ e_bk.
(* keep = 1 *) wire [31:0] br_a = e_fam ? m_result : e_faw ? w_result : e_a;
(* keep = 1 *) wire [31:0] br_b = e_fbm ? m_result : e_fbw ? w_result : e_b;
wire [32:0] br_d = {1'b0, br_a} - {1'b0, br_b};
reg         br_c;
always @* begin
	case (e_f3[2:1])
		2'd0: br_c = br_a == br_b;
		2'd1: br_c = br_a == {27'd0, e_bimm};
		2'd2: br_c = (br_a[31] ^ br_b[31]) ? br_a[31] : br_d[31];
		default: br_c = br_d[32];
	endcase
end

wire [31:0] e_addr   = fa + e_imm;
wire [31:0] e_btgt   = e_pc + e_imm;

// divider
reg        div_busy, div_done, div_init, div_prep, div_negq, div_negr;
reg  [4:0] div_cnt;
reg [31:0] div_q, div_r, div_d, div_res;
reg [33:0] div_d3;
wire       e_div_stall = e_valid && e_div && !div_done;

// pipeline control. stall_m / stall_e come from registers only, two LUTs
// deep: they hold the E and M registers and, through if_ren, the I$ read
// enables. m_wait: M holds p8.hfind, p8.zline or FENCE.I. stall_h: E's
// instruction reads the late result now in M (e_hz, found in D): it waits
// a cycle and then takes it from W.
reg  m_wait;
wire stall_m = m_wait && !dm_done;
wire stall_h = e_valid && e_hz;
wire stall_e = stall_m || e_div_stall || stall_h;

// late: a load or store in M that must be replayed
wire replay = m_valid && (m_load || m_store) && !dm_ready;

// a predicted JALR that did not go where rs1 says (target rs1 & ~1), from
// registers
wire m_jmiss    = m_valid && m_jp && m_mul_a[31:1] != {7'd0, m_ptgt, 1'b0};
wire redirect_m = (m_valid && m_fencei && dm_done) || m_jmiss;
// The redirects from E: a JALR with an offset (a predicted one, offset 0,
// is checked in M: m_jmiss), and a mispredicted branch (br_c ^ e_bk). With
// redirect_m, the terms from registers are combined first (rd_early, br_v,
// kept), so rd_pend meets the branch compare in one LUT.
(* keep = 1 *) wire rd_early = redirect_m || (e_valid && !stall_e && e_jalr && !e_jp);
(* keep = 1 *) wire br_v     = e_valid && !stall_e && e_branch;
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
	// (as one LUT after the branch compare: the terms from registers are
	// combined first, kept)
	rd_pend  <= !reset && (replay || rd_early || (br_v && (br_c ^ e_bk)));
	rd_m     <= replay || redirect_m;
	rd_tgt_m <= m_fencei ? m_pc4 : m_jp ? {m_mul_a[31:1], 1'b0} : m_pc;
	rd_tgt_e <= e_target;
end
wire [31:0] rd_tgt = rd_m ? rd_tgt_m : rd_tgt_e;

wire kill_e   = replay || redirect_m;   // E's instruction does not go on
wire d_ok     = if_valid && !rd_pend && !kill_e;
// a load-use hazard of the D instruction: it goes on to E and waits there
// (e_hz), so the I$ read enables do not wait for this compare
wire hazard_d = late_rd != 5'd0 && (if_way[0] ? haz1 : haz0);

// The next fetch address. The early choices (registers only: reset, a
// redirect, the I$ re-reading d_pc in S_DONE, else d_pc + 4) are made
// first; the predictions from the I$ outputs select last. d_pc + 4 is kept
// with d_pc (off the fetch-address path).
reg  [31:0] d_pc4;
wire        f_early = reset || rd_pend || if_reread;
wire [31:0] f_seq   = reset ? 32'd0 : rd_pend ? rd_tgt : if_reread ? d_pc : d_pc4;
always @* begin
	if (!f_early && d_jp)        if_addr = {7'd0, d_jtgt, 2'b00};
	else if (!f_early && d_pred) if_addr = d_target;
	else                         if_addr = f_seq;
end
// if_adv waits for the I$ hit; the I$ read enables do not: a read on a
// miss is harmless (the I$ re-reads d_pc afterwards)
assign if_adv = reset || rd_pend || (if_valid && !stall_e);
assign if_ren = reset || rd_pend || !stall_e;

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
wire       d_leave  = d_ok && !stall_e;
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
	bht_agree <= !(br_c ^ e_bs);   // taken == the static rule
	bht_old   <= e_bdis;
end
assign bht_wd = bht_agree ? (bht_old == 2'd0 ? 2'd0 : bht_old - 2'd1) : (bht_old == 2'd3 ? 2'd3 : bht_old + 2'd1);

// the D$ read address: its own add (e_addr feeds M and the jump target)
wire [13:0] da_sum = fa_d + e_imm[13:0];
assign de_addr = da_sum;

// The PICO-8 RAM has a read copy for the CPU alone, read with E's address
// whatever E holds (M decides from the registered address whether the data
// is used). Its address has its own add (the D$ address keeps its loads):
// the late forwarded base goes through one LUT and the carry chain, and
// straight to the RAM blocks, which are spread out (64 KiB in byte lanes).
wire [15:0] ra_sum = fa_p + e_imm[15:0];
assign ra_addr = ra_sum[15:2];

// ---------------------------------------------------------------- D -> E

// forwarding for the D instruction: M's instruction next cycle is E's now
// (if it gets there), W's is M's
wire e_to_m_fwd = e_v && e_wen && !e_late && e_rd != 5'd0;
wire m_to_w_wr  = m_valid && m_wen && m_rd != 5'd0;

// Operands and forwarding selects hold only while M waits (stall_m) or E
// waits for a late result (stall_h): a stalled divide has latched its
// operands (div_a/div_b), so they reload from D every cycle and the last
// load, when the divide leaves E, counts. This keeps the divider's stall
// off the 64 operand muxes.
always @(posedge clk) begin
	if (stall_m || stall_h) begin
		// hold; keep forwarded operands since their producers move on.
		// While M waits, W is empty next cycle and M keeps its result (a
		// late one is not forwarded from M). When M goes on, the late
		// result E waits for (stall_h) is in W next cycle.
		e_a   <= fa;
		e_b   <= fb;
		e_fam <= 1'b0;
		e_faw <= !stall_m && e_u1 && m_to_w_wr && m_rd == e_rs1;
		e_fbm <= 1'b0;
		e_fbw <= !stall_m && e_u2 && m_to_w_wr && m_rd == e_rs2;
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
		e_v       <= d_ok;
		e_hz      <= hazard_d;
		e_rs1     <= d_rs1;
		e_rs2     <= d_rs2;
		e_u1      <= d_use1;
		e_u2      <= d_use2;
		e_pc      <= d_pc;
		e_imm     <= d_imm;
		e_rd      <= d_rd;
		e_wen     <= d_wen;
		e_f3      <= d_f3;
		e_shl     <= d_f3 == 3'd1 && (d_op | d_opimm);
		e_shr     <= d_f3 == 3'd5 && (d_op | d_opimm);
		e_cmp     <= d_f3[2:1] == 2'b01;
		e_alt     <= (d_op && di[30]) || (d_opimm && f3 == 3'd5 && di[30]);
		e_op1pc   <= d_auipc;
		e_op1zero <= d_lui;
		e_op2imm  <= !d_op;
		e_branch  <= d_branch;
		e_jal     <= d_jal;
		e_jalr    <= d_jalr;
		e_load    <= d_load;
		e_store   <= d_store || d_stv;
		e_stv     <= d_stv;
		e_tag     <= f3;
		e_mul     <= d_muldiv && !f3[2];
		e_div     <= (d_muldiv && f3[2]) || d_fdiv;
		e_fdv     <= d_fdiv;
		e_fencei  <= d_fencei;
		e_pred    <= d_pred;
		e_bk      <= f3[0] ^ d_pred;
		e_bs      <= f3[0] ^ d_st;
		e_bdis    <= bht_q;
		e_zbb     <= d_zbb;
		e_zh      <= di[31:20];
		e_zf      <= f3;
		e_zo      <= d_op;
		e_late    <= d_load || d_muldiv || d_hfind || d_zbb || d_csr || d_fdiv;
		e_pc4     <= d_pc4;
		e_csr     <= d_csr;
		e_csrk    <= csr_of(di[31:20]);
		e_hpm     <= di[31:24] == 8'hc1 || di[31:24] == 8'hb1;
		e_csrf    <= e_csr && (e_f3[1:0] == 2'd1 || e_zimm != 5'd0) && e_imm[11:0] == di[31:20];
		e_hfind   <= d_hfind;
		e_hnum    <= di[25];
		e_jp      <= d_jp;
		e_ptgt    <= d_tgt23;
		e_bimm    <= d_rs2;
		e_zline   <= d_zline;
		e_zimm    <= d_rs1;
	end else if (!stall_m) begin
		e_hz      <= 1'b0;   // the late result reaches W: no more waiting
	end
	if (reset || kill_e) e_v <= 1'b0;
end

wire [5:0] e_zd  = zbb_of(e_zh, e_zf, e_zo, !e_zo);
wire [4:0] e_zop = e_zd[4:0];

// ---------------------------------------------------------------- divider

// The operands are registered first (div_a/div_b), off the forwarding path.
reg  [31:0] div_a, div_b;
wire        div_sgn = !e_f3[0];
wire [31:0] div_aa  = (div_sgn && div_a[31]) ? -div_a : div_a;
wire [31:0] div_ba  = (div_sgn && div_b[31]) ? -div_b : div_b;
// a step: the partial remainder shifted by 2 bits minus 1, 2 and 3 x the
// divisor, in parallel; the largest that is not negative is the digit
wire [33:0] div_sh  = {div_r, div_q[31:30]};
wire [34:0] div_s1  = {1'b0, div_sh} - {3'd0, div_d};
wire [34:0] div_s2  = {1'b0, div_sh} - {2'd0, div_d, 1'b0};
wire [34:0] div_s3  = {1'b0, div_sh} - {1'b0, div_d3};
// p8.fdiv: the saturated result, and the overflow (quotient >= 2^31:
// |x| >> 15 >= |y|) found from the registers in the prep cycle
wire [31:0] div_sat = (div_a[31] ^ div_b[31]) ? 32'h80000001 : 32'h7fffffff;
wire        div_ovf = div_d[31:17] == 15'd0 && {div_r[15:0], div_q[31]} >= div_d[16:0];

always @(posedge clk) begin
	if (div_busy) begin
		if (!div_s3[34]) begin
			div_r <= div_s3[31:0];
			div_q <= {div_q[29:0], 2'd3};
		end else if (!div_s2[34]) begin
			div_r <= div_s2[31:0];
			div_q <= {div_q[29:0], 2'd2};
		end else if (!div_s1[34]) begin
			div_r <= div_s1[31:0];
			div_q <= {div_q[29:0], 2'd1};
		end else begin
			div_r <= div_sh[31:0];
			div_q <= {div_q[29:0], 2'd0};
		end
		div_cnt <= div_cnt - 5'd1;
		if (div_cnt == 5'd1) begin
			div_busy <= 1'b0;
			div_done <= 1'b1;
		end
	end else if (div_prep) begin
		div_prep <= 1'b0;
		div_d3   <= {2'd0, div_d} + {1'b0, div_d, 1'b0};
		div_cnt  <= 5'd16;
		if (e_fdv && div_ovf) begin
			div_q    <= div_sat;
			div_negq <= 1'b0;
			div_done <= 1'b1;
		end else begin
			div_busy <= 1'b1;
		end
	end else if (div_done) begin
		// result is consumed when the instruction leaves E
		if (!stall_m) div_done <= 1'b0;
	end else if (div_init) begin
		div_init <= 1'b0;
		div_negq <= 1'b0;
		div_negr <= 1'b0;
		if (div_b == 32'd0 || (e_fdv && div_b == 32'h10000)) begin
			// x/0 = all ones, x%0 = x; p8.fdiv: x/0 saturates, x/1.0 = x
			div_q    <= !e_fdv ? 32'hffffffff : div_b[16] ? div_a : div_sat;
			div_r    <= div_a;
			div_done <= 1'b1;
		end else begin
			// p8.fdiv: the dividend |x| * 2^16
			div_q    <= e_fdv ? {div_aa[15:0], 16'd0} : div_aa;
			div_r    <= e_fdv ? {16'd0, div_aa[31:16]} : 32'd0;
			div_d    <= div_ba;
			div_negq <= div_sgn && (div_a[31] ^ div_b[31]);
			div_negr <= div_sgn && div_a[31];
			div_prep <= 1'b1;
		end
	end else if (e_valid && e_div && !e_hz) begin
		div_a    <= fa;
		div_b    <= fb;
		div_init <= 1'b1;
	end
	if (reset || kill_e) begin
		div_busy <= 1'b0;
		div_done <= 1'b0;
		div_init <= 1'b0;
		div_prep <= 1'b0;
	end
end

always @* div_res = e_f3[1] ? (div_negr ? -div_r : div_r) : (div_negq ? -div_q : div_q);

// ---------------------------------------------------------------- E -> M

// ---------------------------------------------------------------- CSRs

// Performance counters, 32 bits, wrapping, counting since power-up (a
// reset does not clear them). Each event goes into a 5-bit count of its
// own; every cycle one counter, in turn, adds that count to its total, in
// LUT RAM (one copy for this, one for CSR reads); a read adds the count to
// the total, so it is exact (events up to 2 cycles before it). (Counters in
// registers, 16 adders and a 32-bit 16:1 read mux, crowded the E / M
// stages: the forwarding paths lost 0.2-0.4 ns.)
//  0 I$ line fills              1 I$ fill cycles (request to last beat)
//  2 I$ way switches (a hit in the way not predicted: 2 cycles each)
//  3 D$ line fills (the CPU's and the table unit's)
//  4 D$ fill cycles (request to last beat)
//  5 D$ write-backs (dirty victims, FENCE.I)
//  6 D$ busy cycles (its FSM not idle: fills, write-backs, zeroing, flushes)
//  7 p8.hfind lookups answered by the table unit's lookup cache
//  8 replays (a load or store not answered: D$ miss, PICO-8 RAM, MMIO)
//  9 ... of those, PICO-8 RAM accesses while the blitter owns it
// 10 load-use stall cycles (a load / MUL / Zbb / CSR / hfind result; not
//    while M replays or redirects: those cycles are the miss's)
// 11 cycles M waits (p8.hfind, p8.zline, FENCE.I)
// 12 divider stall cycles
// 13 mispredicted branches (2 bubbles each)
// 14 JALR redirects (an offset JALR, 2 bubbles; a mispredicted target, 3)
// 15 p8.hfind lookups
// The events come from registers only (a late signal such as the branch
// compare or replay would get another load): a cycle later, rd_m says M
// replayed or redirected (pq_redm: redirected), bht_agree / bht_we give a
// branch's outcome.
reg [15:0] pm_q;
reg        pq_jre, pq_jmiss, pq_redm, pq_p8, pq_sh, pq_kd;
reg  [4:0] pc_loc [0:15];          // events not in the totals yet
reg  [3:0] pc_i = 4'd0;            // the counter adding them now
wire [31:0] pc_tot, pc_csr;
wire [31:0] pc_sum = pc_tot + {27'd0, pc_loc[pc_i]};
p8_lutram #(.AW(4), .DW(32)) pc_f (.clk(clk), .raddr(pc_i), .rdata(pc_tot), .we(1'b1), .waddr(pc_i), .wdata(pc_sum));
p8_lutram #(.AW(4), .DW(32)) pc_r (.clk(clk), .raddr(e_imm[3:0]), .rdata(pc_csr), .we(1'b1), .waddr(pc_i), .wdata(pc_sum));
integer hi;
always @(posedge clk) begin
	pq_jre   <= e_valid && !stall_e && e_jalr && !e_jp;   // an offset JALR leaves E
	pq_jmiss <= m_jmiss;
	pq_redm  <= redirect_m;
	pq_p8    <= dm_p8;
	pq_sh    <= stall_h;
	pq_kd    <= e_bk ^ e_bs;      // the prediction differs from the static rule
	pm_q <= {m_valid && m_hfind && dm_done,
	         (pq_jre && !rd_m) || pq_jmiss,
	         bht_we && (!bht_agree ^ pq_kd) && !rd_m,
	         e_div_stall, stall_m, pq_sh && !rd_m,
	         rd_m && !pq_redm && pq_p8, rd_m && !pq_redm,
	         pm_ev};
	pc_i <= pc_i + 4'd1;
	for (hi = 0; hi < 16; hi = hi + 1)
		pc_loc[hi] <= (pc_i == hi ? 5'd0 : pc_loc[hi]) + {4'd0, pm_q[hi]};
end

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
// (the CSR and the write forward are decoded in D: e_csrk, e_csrf)
always @* begin
	case (e_csrk)
		K_CYC:   csr_old = cnt_cycle[31:0];
		K_CYCH:  csr_old = cnt_cycle[63:32];
		K_INS:   csr_old = instret_e[31:0];
		K_INSH:  csr_old = instret_e[63:32];
		K_MISA:  csr_old = 32'h40001100;   // misa: RV32IM
		K_MSCR:  csr_old = mscratch;
		K_HEAP:  csr_old = p8heap;
		K_FEAT:  csr_old = 32'd31;         // p8feat
		default: csr_old = e_hpm ? pc_csr + {27'd0, pc_loc[e_imm[3:0]]} : 32'd0;
	endcase
	if (m_valid && e_csrf) csr_old = m_csr_v;
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
// this mux, on the forwarding loop, to two inputs, and the shifts' to the
// last LUT
wire [31:0] e_result = e_shl ? sh_l : e_shr ? sh_r[31:0] : e_cmp ? {31'd0, cm_lt} : (e_jal | e_jalr) ? e_pc4 : alu;

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
		m_valid  <= e_valid && !e_div_stall && !stall_h && !kill_e;
		m_wait   <= e_valid && !stall_h && !kill_e && (e_fencei || e_hfind || e_zline);
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
		m_stv    <= e_stv;
		m_tag    <= e_tag;
		m_hkey   <= fb;
		m_f3     <= e_f3;
		m_mul_a  <= fa;
		m_mul_b  <= fb;
		m_zb_b   <= op2;
		m_zbb    <= e_zbb;
		m_zop    <= e_zop;
		m_zrl    <= e_zop == Z_ROL;
		m_zrr    <= e_zop == Z_ROR;
		m_dc     <= e_div || e_csr;
		m_dres   <= e_div ? div_res : csr_old;
		m_ldsel  <= e_load;
		m_bsel   <= e_f3[1:0] == 2'd0 ? e_addr[1:0] : e_f3[1:0] == 2'd1 ? {e_addr[1], 1'b0} : 2'd0;
		m_hup    <= e_f3[1:0] == 2'd1 && e_addr[1];
		m_hext   <= e_f3[1:0] == 2'd0;
		m_wext   <= !e_f3[1];
		m_sidx   <= e_f3[1:0] == 2'd0 ? e_addr[1:0] : {e_addr[1], 1'b1};
		m_sen    <= !e_f3[2];
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
assign dm_stv    = m_stv;
assign dm_tag    = m_tag;
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
// min[u] / max[u]: rs1 < rs2 (signed or not) is E's ALU result (slt[u])
wire        z_lt  = m_result[0];
wire [63:0] z_rl  = {za, za} << zb[4:0];
wire [63:0] z_rr  = {za, za} >> zb[4:0];
// the rotates (a funnel shift by a register) have their own path into the
// result, after the other operations have met theirs
reg  [31:0] zbb_oth;
wire [31:0] zbb_res = m_zrl ? z_rl[63:32] : m_zrr ? z_rr[31:0] : zbb_oth;
always @* begin
	case (m_zop)
		Z_ANDN:  zbb_oth = za & ~zb;
		Z_ORN:   zbb_oth = za | ~zb;
		Z_XNOR:  zbb_oth = ~(za ^ zb);
		Z_CLZ:   zbb_oth = {26'd0, clz32(za)};
		Z_CTZ:   zbb_oth = {26'd0, clz32(rev32(za))};
		Z_CPOP:  zbb_oth = {26'd0, cpop32(za)};
		Z_SEXTB: zbb_oth = {{24{za[7]}}, za[7:0]};
		Z_SEXTH: zbb_oth = {{16{za[15]}}, za[15:0]};
		Z_ZEXTH: zbb_oth = {16'd0, za[15:0]};
		Z_MAX:   zbb_oth = z_lt ? zb : za;
		Z_MAXU:  zbb_oth = z_lt ? zb : za;
		Z_MIN:   zbb_oth = z_lt ? za : zb;
		Z_MINU:  zbb_oth = z_lt ? za : zb;
		Z_ORCB:  zbb_oth = {{8{|za[31:24]}}, {8{|za[23:16]}}, {8{|za[15:8]}}, {8{|za[7:0]}}};
		default: zbb_oth = {za[7:0], za[15:8], za[23:16], za[31:24]};   // rev8
	endcase
end

// ---------------------------------------------------------------- M -> W

wire [31:0] m_pc4 = m_pc + 32'd4;

// load alignment from the pre-decoded selects (m_bsel ...): bits 7-0 are
// one LUT from the word, the others two (the sign first); applied to each
// candidate word in parallel with the D$ tag compare that selects between
// the ways
function automatic [31:0] ld_align(input [31:0] d);
	reg s;
	begin
		s = m_sen && d[8*m_sidx + 7];
		ld_align[7:0]   = d[8*m_bsel +: 8];
		ld_align[15:8]  = m_hext ? {8{s}} : m_hup ? d[31:24] : d[15:8];
		ld_align[31:16] = m_wext ? {16{s}} : d[31:16];
	end
endfunction

wire [31:0] m_ld0 = ld_align(dm_rdata0);
wire [31:0] m_ld1 = ld_align(dm_rdata1);
wire [31:0] m_ldp = ld_align(dm_pdata);
wire [31:0] m_ldm = ld_align(dm_mdata);

// The early results (ALU, MUL, Zbb, p8.hfind: registers and the DSP) are selected
// first and kept as one node. A load outside DDR (PICO-8 RAM, MMIO) has
// its own select in front of the last one, so the late D$ data (word and
// way select, alignment) meets a single select into w_val.
(* keep = 1 *) wire [31:0] m_other = m_hfind ? dm_hres : m_mul ? mul_res : m_zbb ? zbb_res : m_dc ? m_dres : m_result;
wire        m_lddc = m_ldsel && !dm_p8 && !dm_mmio;
wire [31:0] m_alt  = (m_ldsel && dm_p8) ? m_ldp : (m_ldsel && dm_mmio) ? m_ldm : m_other;

// stall_m only makes W a bubble; the other W registers load every cycle
always @(posedge clk) begin
	w_valid <= !reset && !stall_m && m_valid && !replay;
	w_rd    <= m_rd;
	w_wen   <= m_wen;
	w_val   <= m_lddc ? (dm_rsel1 ? m_ld1 : m_ld0) : m_alt;
end

endmodule
