// Simple dual-port block RAM (one read port, one write port, one clock).
//
// Synchronous read when `re` is high; otherwise rdata keeps its value
// (the M10K read enable). With BYPASS=1 a read of the address that is written
// in the same cycle returns the written value (explicit bypass, so the
// M10K mixed-port behaviour does not matter); with BYPASS=0 that read is
// undefined and the user must not depend on it. The bypass compare is done
// after the read, on registered addresses, in parallel with the RAM access
// (raddr is often late). `rdata_raw` is the RAM output without the bypass,
// for a reader that is never served in the cycle after a write to its
// address (it skips the compare and mux).

module p8_bram #(
	parameter AW = 10,
	parameter DW = 8,
	parameter BYPASS = 1,
	parameter INIT = ""      // optional $readmemh file with the initial contents
) (
	input               clk,
	input               re,
	input      [AW-1:0] raddr,
	output     [DW-1:0] rdata,
	output     [DW-1:0] rdata_raw,
	input               we,
	input      [AW-1:0] waddr,
	input      [DW-1:0] wdata
);

(* ramstyle = "no_rw_check" *) reg [DW-1:0] mem [0:(1<<AW)-1];

initial if (INIT != "") $readmemh(INIT, mem);

reg [DW-1:0] q;
reg [AW-1:0] ra_q, wa_q;
reg          we_q;
reg [DW-1:0] byp_d;

always @(posedge clk) begin
	if (we) mem[waddr] <= wdata;
	if (re) begin
		q     <= mem[raddr];
		ra_q  <= raddr;
		wa_q  <= waddr;
		we_q  <= we;
		byp_d <= wdata;
	end
end

wire byp = we_q && wa_q == ra_q;

`ifdef VERILATOR
// make reliance on an undefined read-during-write visible in simulation
assign rdata = byp ? (BYPASS ? byp_d : ~q) : q;
assign rdata_raw = byp ? ~q : q;
`else
assign rdata = (BYPASS && byp) ? byp_d : q;
assign rdata_raw = q;
`endif

endmodule

// Mixed-width simple dual-port block RAM: writes two DW-bit words at once
// (entry waddr), reads one (word raddr[0] of entry raddr[AW:1]). The M10K
// selects the word itself, so a reader of one word has no mux after the
// RAM. Synchronous read when `re` is high, else rdata keeps its value. A
// read of the entry written in the same cycle is undefined (no bypass).
module p8_bram_mw #(
	parameter AW = 10,       // entry address bits
	parameter DW = 8         // word width
) (
	input               clk,
	input               re,
	input      [AW:0]   raddr,
	output     [DW-1:0] rdata,
	input               we,
	input      [AW-1:0] waddr,
	input  [2*DW-1:0]   wdata
);

(* ramstyle = "no_rw_check" *) logic [1:0][DW-1:0] mem [0:(1<<AW)-1];

reg [DW-1:0] q;
always @(posedge clk) begin
	if (we) mem[waddr] <= wdata;
	if (re) q <= mem[raddr[AW:1]][raddr[0]];
end

`ifdef VERILATOR
// make reliance on an undefined read-during-write visible in simulation
reg [AW-1:0] ra_q, wa_q;
reg          we_q;
always @(posedge clk) if (re) begin
	ra_q <= raddr[AW:1];
	wa_q <= waddr;
	we_q <= we;
end
assign rdata = (we_q && wa_q == ra_q) ? ~q : q;
`else
assign rdata = q;
`endif

endmodule

// RAM with a synchronous write port and an asynchronous read port, in LUT
// RAM (MLAB): a read is about a LUT's delay after its address, for a reader
// whose address is late (the CPU's register read, from the I$ RAMs). A
// read of the word written in the same cycle gives the old value.
module p8_lutram #(
	parameter AW = 5,
	parameter DW = 32
) (
	input           clk,
	input  [AW-1:0] raddr,
	output [DW-1:0] rdata,
	input           we,
	input  [AW-1:0] waddr,
	input  [DW-1:0] wdata
);

(* ramstyle = "MLAB, no_rw_check" *) reg [DW-1:0] mem [0:(1<<AW)-1];

always @(posedge clk) if (we) mem[waddr] <= wdata;
assign rdata = mem[raddr];

endmodule
