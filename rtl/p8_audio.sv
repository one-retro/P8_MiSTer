// Audio sample FIFO: the firmware synthesises PICO-8 audio and pushes
// signed 16-bit mono samples; they are played at 22050 Hz.
// On underrun the last sample is held.

module p8_audio #(
	parameter CLK_HZ = 100000000,
	parameter RATE   = 22050
) (
	input             clk,
	input             reset,
	input             wr,
	input      [15:0] wdata,
	output     [11:0] level,     // samples queued (0-2047)
	output reg [15:0] out
);

reg [10:0] wp, rp;
reg [31:0] acc;
wire [15:0] q;

assign level = {1'b0, wp - rp};
wire full  = (wp + 11'd1) == rp;
wire empty = wp == rp;

p8_bram #(.AW(11), .DW(16)) fifo (
	.clk(clk), .re(1'b1), .raddr(rp), .rdata(q), .rdata_raw(),
	.we(wr && !full), .waddr(wp), .wdata(wdata)
);

always @(posedge clk) begin
	if (reset) begin
		wp  <= 11'd0;
		rp  <= 11'd0;
		acc <= 32'd0;
		out <= 16'd0;
	end else begin
		if (wr && !full) wp <= wp + 11'd1;
		if (acc + RATE >= CLK_HZ) begin
			acc <= acc + RATE - CLK_HZ;
			if (!empty) begin
				out <= q;
				rp  <= rp + 11'd1;
			end
		end else begin
			acc <= acc + RATE;
		end
	end
end

endmodule
