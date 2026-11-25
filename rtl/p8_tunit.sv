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
// read request) and reads 64-bit words: {nodes, nmask} at tab+16, {hash,
// len} at key+8 (a number key: the product, from a DSP during the header
// read, gives the index in one cycle instead), then one node per probe.
// The D$ answers a read in the next cycle (for the address read then), so
// the unit presents the next address while it checks a word where it
// knows it: the key's hash is read in the cycle after the header (2
// cycles per word otherwise). `done` is a one-cycle answer to the CPU's
// M stage (dm_ready).
//
// Lookup cache: 256 entries, direct-mapped by bits 11-4 of the table and
// of the key and 23-16 of the key (an integer number key's low bits),
// each {table, key, number key, the table's nodes and nmask when the key
// was found, the node's offset}, written when a lookup finds its key. An
// entry for this table and key gives the node to read in the cycle after
// the header, instead of the hash; if the header still holds that nodes
// and nmask, and the node the key, it is the answer (otherwise the lookup
// goes on as without the cache). That needs no invalidation: the node is
// inside the table's current node array (same array, same size), which
// holds a key at most once and only where a probe from its hash finds it
// (the runtime's hash part loses keys only by being rebuilt into a new
// array; see table.rs), so it is the node a probe would find. Offsets,
// not addresses, so a new arena base is harmless too.
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
	output reg        pm_hit,     // a lookup answered from the cache (counter)

	// D$ access
	output            active,
	output     [31:0] addr,       // E-side address (answered next cycle)
	output            rd,         // M-side read request (last cycle's address)
	input             ready,
	input      [63:0] rdata0,     // the word is rsel1 ? rdata1 : rdata0
	input      [63:0] rdata1,
	input             rsel1
);

localparam S_IDLE = 4'd0, S_HDR_E = 4'd1, S_HDR_M = 4'd2, S_HASH_E = 4'd3,
           S_HASH_M = 4'd4, S_NODE_E = 4'd5, S_NODE_M = 4'd6, S_DONE = 4'd7, S_NHASH = 4'd8,
           S_CNODE_M = 4'd9;
localparam T_NUMBER = 32'd3, T_STRING = 32'd4;

reg  [3:0] st;
reg        nm;                       // the lookup's key is a number
reg [31:0] kx, kprod;                // key ^ 3 * 0x85eb_ca6b, its product
reg [31:0] a;
reg [31:0] nbase, nmask, idx, cnt;   // nbase: address of node 0
reg [31:0] hnodes;                   // the header's nodes (arena offset)

// st != S_IDLE && st != S_DONE, as a register: it selects the D$ address
// (timing-critical path into the CPU's stall logic)
reg act;
assign active = act;
assign addr   = a;
assign rd     = st == S_HDR_M || st == S_HASH_M || st == S_NODE_M || st == S_CNODE_M;

wire [31:0] next_idx = (idx + 32'd1) & nmask;
wire [31:0] hdr_addr = base + tab + 32'd16;
wire [31:0] str_addr = base + key + 32'd8;

// ---------------------------------------------------------------- lookup cache

// {valid, num, tab[24:0], key, nodes[24:0], nmask[15:0], node offset[24:0]}
localparam CW = 1 + 1 + 25 + 32 + 25 + 16 + 25;
wire  [7:0] c_ra = tab[11:4] ^ key[11:4] ^ key[23:16];
wire [CW-1:0] c_q;
reg         c_we;
reg   [7:0] c_wa;
reg  [CW-1:0] c_wd;
p8_bram #(.AW(8), .DW(CW), .BYPASS(0)) lcache (
	.clk(clk), .re(1'b1), .raddr(c_ra), .rdata(c_q), .rdata_raw(),
	.we(c_we), .waddr(c_wa), .wdata(c_wd)
);
wire        ce_valid = c_q[CW-1];
wire        ce_num   = c_q[CW-2];
wire [24:0] ce_tab   = c_q[CW-3 -: 25];
wire [31:0] ce_key   = c_q[CW-28 -: 32];
wire [24:0] ce_nodes = c_q[CW-60 -: 25];
wire [15:0] ce_nmask = c_q[CW-85 -: 16];
wire [24:0] ce_noff  = c_q[24:0];
// (read in S_IDLE with this lookup's table and key, which stay)
wire        ce_hit   = ce_valid && ce_num == nm && ce_tab == tab[24:0] && tab[31:25] == 7'd0 && ce_key == key;

reg         cpend;                   // the word read after the header is the cache's node
reg  [24:0] c_nodes;
reg  [15:0] c_nmask;

// ---------------------------------------------------------------- words

// per D$ way: header (nodes == 0, nodes/nmask as cached, node base), hash
// (index, node address), node (key found, empty)
function automatic [98:0] word_ops(input [63:0] d, input [31:0] base_i, input [31:0] nbase_i,
                                    input [31:0] nmask_i, input [31:0] key_i, input nm_i,
                                    input [24:0] cn_i, input [15:0] cm_i);
	reg [31:0] hidx;
	begin
		hidx     = d[31:0] & nmask_i;
		word_ops = {d[31:0] == 32'd0,
		            d[31:0] == {7'd0, cn_i} && d[63:32] == {16'd0, cm_i},
		            d[31:0] == (nm_i ? T_NUMBER : T_STRING) && d[63:32] == key_i,
		            base_i + d[31:0] + 32'd8,
		            hidx,
		            nbase_i + {hidx[27:0], 4'b0000}};
	end
endfunction
wire [98:0] ops0 = word_ops(rdata0, base, nbase, nmask, key, nm, c_nodes, c_nmask);
wire [98:0] ops1 = word_ops(rdata1, base, nbase, nmask, key, nm, c_nodes, c_nmask);
wire [98:0] ops  = rsel1 ? ops1 : ops0;
wire        w_zero    = ops[98];
wire        w_cval    = ops[97];
wire        w_match   = ops[96];
wire [31:0] w_nbase   = ops[95:64];
wire [31:0] hash_idx  = ops[63:32];
wire [31:0] hash_addr = ops[31:0];
wire [31:0] w_nodes   = rsel1 ? rdata1[31:0] : rdata0[31:0];
wire [31:0] w_nmask   = rsel1 ? rdata1[63:32] : rdata0[63:32];
// a number key's index, from registers (S_NHASH)
wire [31:0] kmix      = kprod ^ {15'd0, kprod[31:15]};
wire [31:0] nidx      = kmix & nmask;
// the found node's offset, for the cache
wire [31:0] noff      = a - base;

always @(posedge clk) begin
	done   <= 1'b0;
	pm_hit <= 1'b0;
	c_we   <= 1'b0;
	if (reset) begin
		st  <= S_IDLE;
		act <= 1'b0;
	end else begin
		case (st)
		S_IDLE: if (start && !done) begin
			a   <= hdr_addr;
			act <= 1'b1;
			nm  <= num;
			kx  <= key ^ 32'h91c3_5f41;   // 3 * 0x85eb_ca6b
			st  <= S_HDR_E;
		end
		S_HDR_E: begin
			// the D$ reads the header; next, the cache's node or the hash
			kprod   <= kx * 32'h9e37_79b1;
			cpend   <= ce_hit;
			c_nodes <= ce_nodes;
			c_nmask <= ce_nmask;
			if (ce_hit)   a <= base + {7'd0, ce_noff};
			else if (!nm) a <= str_addr;
			st <= S_HDR_M;
		end
		S_HDR_M: if (!ready) begin
			// read the header again (the D$ now reads the other word)
			a  <= hdr_addr;
			st <= S_HDR_E;
		end else begin
			nbase  <= w_nbase;
			nmask  <= w_nmask;
			hnodes <= w_nodes;
			if (w_zero) begin
				result <= 32'd0;
				done   <= 1'b1;
				act    <= 1'b0;
				st     <= S_DONE;
			end else if (cpend && w_cval) begin
				st <= S_CNODE_M;             // the cache's node, read now
			end else if (nm) begin
				st <= S_NHASH;
			end else if (cpend) begin
				a  <= str_addr;
				st <= S_HASH_E;
			end else begin
				st <= S_HASH_M;              // the hash, read now
			end
		end
		S_CNODE_M: if (ready) begin
			if (w_match) begin
				result <= a;
				done   <= 1'b1;
				pm_hit <= 1'b1;
				act    <= 1'b0;
				st     <= S_DONE;
			end else if (nm) begin
				st <= S_NHASH;
			end else begin
				a  <= str_addr;
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
				// remember it (offsets in 25 bits, up to 64K nodes)
				c_we <= tab[31:25] == 7'd0 && hnodes[31:25] == 7'd0 && nmask[31:16] == 16'd0 &&
				        noff[31:25] == 7'd0;
				c_wa <= c_ra;
				c_wd <= {1'b1, nm, tab[24:0], key, hnodes[24:0], nmask[15:0], noff[24:0]};
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
