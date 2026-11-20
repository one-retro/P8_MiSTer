// Table unit: executes p8.hfind for the CPU (custom-0, funct3 0: the
// hash-part lookup of the runtime's tables).
//
// rd = absolute address of the node of table `tab` (arena offset) whose key
// is the interned string `key` (arena offset), or 0. Heap arena at `base`.
// With `num` (funct7 1) the key is the number whose 16.16 payload is `key`,
// hashed here as the runtime hashes number keys:
// h = (key ^ 3 * 0x85eb_ca6b) * 0x9e37_79b1, index (h ^ h >> 15) & nmask.
//
// While `active`, the unit drives the D$ (E-side RAM address and M-side
// read request) and reads 64-bit words, one every 2 cycles on a hit:
//   {nodes, nmask} at tab+16, {hash, len} at key+8 (a number key: the
//   product, from a DSP during the header read, gives the index in one
//   cycle instead), then one node per probe.
// `done` is a one-cycle answer to the CPU's M stage (dm_ready).
//
// The D$ word arrives as both ways' words and the hit way (late): every
// compare and address add is done for both words in parallel and the hit
// way selects the results.

module p8_tunit (
	input             clk,
	input             reset,

	input             start,      // M holds p8.hfind (level, until done)
	input             num,        // a number key (with start)
	input      [31:0] base,
	input      [31:0] tab,
	input      [31:0] key,
	output reg        done,
	output reg [31:0] result,

	// D$ access
	output            active,
	output     [31:0] addr,       // E-side and M-side address
	output            rd,         // M-side read request
	input             ready,
	input      [63:0] rdata0,     // the word is rsel1 ? rdata1 : rdata0
	input      [63:0] rdata1,
	input             rsel1
);

localparam S_IDLE = 4'd0, S_HDR_E = 4'd1, S_HDR_M = 4'd2, S_HASH_E = 4'd3,
           S_HASH_M = 4'd4, S_NODE_E = 4'd5, S_NODE_M = 4'd6, S_DONE = 4'd7, S_NHASH = 4'd8;

localparam T_NUMBER = 32'd3, T_STRING = 32'd4;

reg  [3:0] st;
reg        nm;                       // the lookup's key is a number
reg [31:0] kx, kprod;                // key ^ 3 * 0x85eb_ca6b, its product
reg [31:0] a;
reg [31:0] nbase, nmask, idx, cnt;   // nbase: address of node 0

// st != S_IDLE && st != S_DONE, as a register: it selects the D$ address
// (timing-critical path into the CPU's stall logic)
reg act;
assign active = act;
assign addr   = a;
assign rd     = st == S_HDR_M || st == S_HASH_M || st == S_NODE_M;

wire [31:0] next_idx = (idx + 32'd1) & nmask;

// per D$ way: header (nodes == 0, node base), hash (index, node address),
// node (key found, empty)
function automatic [97:0] word_ops(input [63:0] d, input [31:0] base_i, input [31:0] nbase_i,
                                    input [31:0] nmask_i, input [31:0] key_i, input nm_i);
	reg [31:0] hidx;
	begin
		hidx     = d[31:0] & nmask_i;
		word_ops = {d[31:0] == 32'd0,
		            d[31:0] == (nm_i ? T_NUMBER : T_STRING) && d[63:32] == key_i,
		            base_i + d[31:0] + 32'd8,
		            hidx,
		            nbase_i + {hidx[27:0], 4'b0000}};
	end
endfunction
wire [97:0] ops0 = word_ops(rdata0, base, nbase, nmask, key, nm);
wire [97:0] ops1 = word_ops(rdata1, base, nbase, nmask, key, nm);
wire [97:0] ops  = rsel1 ? ops1 : ops0;
wire        w_zero    = ops[97];
wire        w_match   = ops[96];
wire [31:0] w_nbase   = ops[95:64];
wire [31:0] hash_idx  = ops[63:32];
wire [31:0] hash_addr = ops[31:0];
wire [31:0] w_nmask   = rsel1 ? rdata1[63:32] : rdata0[63:32];
// a number key's index, from registers (S_NHASH)
wire [31:0] kmix      = kprod ^ {15'd0, kprod[31:15]};
wire [31:0] nidx      = kmix & nmask;

always @(posedge clk) begin
	done <= 1'b0;
	if (reset) begin
		st  <= S_IDLE;
		act <= 1'b0;
	end else begin
		case (st)
		S_IDLE: if (start && !done) begin
			a   <= base + tab + 32'd16;
			act <= 1'b1;
			nm  <= num;
			kx  <= key ^ 32'h91c3_5f41;   // 3 * 0x85eb_ca6b
			st  <= S_HDR_E;
		end
		S_HDR_E: begin
			kprod <= kx * 32'h9e37_79b1;
			st    <= S_HDR_M;
		end
		S_HDR_M: if (ready) begin
			nbase <= w_nbase;
			nmask <= w_nmask;
			if (w_zero) begin
				result <= 32'd0;
				done   <= 1'b1;
				act    <= 1'b0;
				st     <= S_DONE;
			end else if (nm) begin
				st <= S_NHASH;
			end else begin
				a  <= base + key + 32'd8;
				st <= S_HASH_E;
			end
		end
		S_NHASH: begin
			idx <= nidx;
			a   <= nbase + {nidx[27:0], 4'b0000};
			cnt <= 32'd0;
			st  <= S_NODE_E;
		end
		S_HASH_E: st <= S_HASH_M;
		S_HASH_M: if (ready) begin
			idx <= hash_idx;
			a   <= hash_addr;
			cnt <= 32'd0;
			st  <= S_NODE_E;
		end
		S_NODE_E: st <= S_NODE_M;
		S_NODE_M: if (ready) begin
			if (w_match) begin
				result <= a;
				done   <= 1'b1;
				act    <= 1'b0;
				st     <= S_DONE;
			end else if (w_zero || cnt == nmask) begin
				result <= 32'd0;
				done   <= 1'b1;
				act    <= 1'b0;
				st     <= S_DONE;
			end else begin
				idx <= next_idx;
				a   <= nbase + {next_idx[27:0], 4'b0000};
				cnt <= cnt + 32'd1;
				st  <= S_NODE_E;
			end
		end
		default: st <= S_IDLE;   // S_DONE: the CPU takes the result now
		endcase
	end
end

endmodule
