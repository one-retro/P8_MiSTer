// DDR3 (HPS f2sdram, Avalon-MM 64-bit) arbiter for the I$, D$, loader and
// save state port.
//
// A client raises `req` and keeps it high for its whole transaction
// (address phase and every data beat); it may drive rd/we only while it
// has `gnt`. Dropping `req` releases the bus; the grant is registered, so
// it changes one cycle after `req`. Priority: D$, I$, loader, save states.
// `hold` stops new grants to the caches (used to quiesce the CPU before a
// reset; the loader is not affected). There is no reset: the DDR
// controller does not see one, so a transaction must always complete.
//
// Client addresses are 64-bit word addresses in the core's 32 MiB window,
// which starts at DDR byte address 0x3000_0000; the save state port's are
// in the 64 MiB save state area at 0x3400_0000 (MiSTer Main's four slots).

module p8_ddr_arb (
	input             clk,
	input             hold,
	output            idle_caches,   // no cache owns the bus

	// client 0: D$
	input             c0_req, c0_rd, c0_we,
	input      [21:0] c0_addr,
	input      [63:0] c0_wdata,
	output            c0_gnt,
	// client 1: I$ (read only)
	input             c1_req, c1_rd,
	input      [21:0] c1_addr,
	output            c1_gnt,
	// client 2: loader (single-beat writes)
	input             c2_req, c2_we,
	input      [21:0] c2_addr,
	input      [63:0] c2_wdata,
	input       [7:0] c2_be,
	output            c2_gnt,
	// client 3: save state port (single beats)
	input             c3_req, c3_rd, c3_we,
	input      [22:0] c3_addr,
	input      [63:0] c3_wdata,
	output            c3_gnt,

	// shared response
	output            busy,
	output     [63:0] rdata,
	output            rvalid,

	// DDRAM
	output     [28:0] DDRAM_ADDR,
	output      [7:0] DDRAM_BURSTCNT,
	output            DDRAM_RD,
	output            DDRAM_WE,
	output     [63:0] DDRAM_DIN,
	output      [7:0] DDRAM_BE,
	input             DDRAM_BUSY,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY
);

localparam O_NONE = 3'd0, O_DC = 3'd1, O_IC = 3'd2, O_LD = 3'd3, O_SS = 3'd4;
reg [2:0] owner = O_NONE;

always @(posedge clk) begin
	case (owner)
		O_DC: if (!c0_req) owner <= O_NONE;
		O_IC: if (!c1_req) owner <= O_NONE;
		O_LD: if (!c2_req) owner <= O_NONE;
		O_SS: if (!c3_req) owner <= O_NONE;
		default:
			if (c0_req && !hold)      owner <= O_DC;
			else if (c1_req && !hold) owner <= O_IC;
			else if (c2_req)          owner <= O_LD;
			else if (c3_req)          owner <= O_SS;
	endcase
end

assign c0_gnt = owner == O_DC;
assign c1_gnt = owner == O_IC;
assign c2_gnt = owner == O_LD;
assign c3_gnt = owner == O_SS;
assign idle_caches = owner != O_DC && owner != O_IC;

reg [21:0] a;
always @* begin
	case (owner)
		O_DC:    a = c0_addr;
		O_IC:    a = c1_addr;
		default: a = c2_addr;
	endcase
end

assign DDRAM_ADDR     = c3_gnt ? {4'b0011, 2'b01, c3_addr} : {4'b0011, 3'b000, a};
assign DDRAM_BURSTCNT = (c2_gnt || c3_gnt) ? 8'd1 : 8'd4;
assign DDRAM_RD       = (c0_gnt && c0_rd) || (c1_gnt && c1_rd) || (c3_gnt && c3_rd);
assign DDRAM_WE       = (c0_gnt && c0_we) || (c2_gnt && c2_we) || (c3_gnt && c3_we);
assign DDRAM_DIN      = c2_gnt ? c2_wdata : c3_gnt ? c3_wdata : c0_wdata;
assign DDRAM_BE       = c2_gnt ? c2_be : 8'hff;

assign busy   = DDRAM_BUSY;
assign rdata  = DDRAM_DOUT;
assign rvalid = DDRAM_DOUT_READY;

endmodule
