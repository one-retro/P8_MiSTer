// HPS file download (ioctl, 8-bit) into DDR.
//
// A program is an image that runs from CPU address 0: the BIOS (or any
// p8c image) loaded with the core, games/P8/boot.rom (index 0) or
// boot1.rom (index 0x40). Its size is reported in CART_SIZE.
//
// The OSD's carts: "Load Cart" (F1, index 1: .p8 / .p8.png) and "Load
// Compiled Cart" (F2, index 2: .p8c). They go to the cart buffer at byte
// 0x0100_0000, where the BIOS (which says it is there with LOAD_CTRL bit
// 0) compiles or runs them; LOAD_CTRL counts these downloads and gives the
// slot of the last one, LOAD_SIZE its size. Without the BIOS a .p8c is a
// program (to address 0, as boot.rom). Any download stops the CPU, which
// starts again from address 0 afterwards.
//
// The program stays loaded through a reset (OSD Reset, USER button): the
// CPU starts it again from address 0. Its image is not modified while it
// runs (.data is copied out of it at start-up). The loader has no reset
// (power-up values only): a reset in the middle of a DDR write would
// leave the controller waiting for it.
//
// Bytes are packed into 64-bit words and written with byte enables.
// Downloads stall (ioctl_wait) until the CPU is stopped (`cpu_stopped`).

module p8_loader (
	input             clk,

	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input      [26:0] ioctl_addr,
	input       [7:0] ioctl_dout,
	output            ioctl_wait,

	input             cpu_stopped,
	output            busy,           // download or write-out in progress
	output reg        loaded = 1'b0,  // a complete program is in DDR
	output reg [31:0] cart_size = 32'd0,
	input             bios_wr,        // LOAD_CTRL write: bit 0 of bios_data
	input             bios_data,
	output reg        bios = 1'b0,    // the program at 0 is the BIOS
	output reg [15:0] cart_seq = 16'd0,   // cart buffer downloads so far
	output reg  [5:0] cart_slot = 6'd0,   // slot of the last one (1, 2)
	output reg [31:0] buf_size = 32'd0,   // its size

	// DDR client (single-beat writes)
	output reg        mem_req = 1'b0,
	input             mem_gnt,
	output reg        mem_we = 1'b0,
	output reg [21:0] mem_addr,
	output reg [63:0] mem_wdata,
	output reg  [7:0] mem_be,
	input             mem_busy
);

// to the cart buffer: .p8 / .p8.png always, .p8c when the BIOS is there
// (bits 7-6 of the index: which of the menu entry's extensions the file
// has, e.g. 0x41 for a .png from "Load Cart"; 0x40 is boot1.rom)
wire        to_buf  = ioctl_index[15:8] == 8'd0 && (ioctl_index[5:0] == 6'd1 || (bios && ioctl_index[5:0] == 6'd2));
wire        is_prog = ioctl_index == 16'h0000 || ioctl_index == 16'h0040 || (ioctl_index[15:8] == 8'd0 && ioctl_index[5:0] == 6'd2 && !bios);
reg         dl_buf = 1'b0;   // this download goes to the cart buffer
reg         dl_any = 1'b0;   // ... or to address 0
wire [24:0] dst     = dl_buf ? {2'b10, ioctl_addr[22:0]} : ioctl_addr[24:0];   // buffer: byte 0x0100_0000, up to 8 MiB

reg        dl_d = 1'b0;
reg        flush = 1'b0;   // write-out of the packed word in progress
reg  [7:0] be = 8'd0;      // lanes collected so far
reg [63:0] buf64;
reg [21:0] word;

assign ioctl_wait = ioctl_download && (!cpu_stopped || flush);
assign busy       = ioctl_download || dl_d || flush;

always @(posedge clk) begin
	dl_d <= ioctl_download;

	if (bios_wr) bios <= bios_data;

	if (ioctl_download && !dl_d) begin
		dl_buf <= to_buf;
		dl_any <= to_buf || is_prog;
		if (to_buf) buf_size <= 32'd0;
		if (is_prog) begin
			cart_size <= 32'd0;
			loaded    <= 1'b0;
			bios      <= 1'b0;
		end
	end

	// (the first byte is written a cycle after the start: dl_buf is set)
	if (ioctl_download && dl_d && ioctl_wr && dl_any) begin
		buf64[ioctl_addr[2:0]*8 +: 8] <= ioctl_dout;
		be[ioctl_addr[2:0]] <= 1'b1;
		word <= dst[24:3];
		if (dl_buf) buf_size <= {5'd0, ioctl_addr} + 32'd1;
		else cart_size <= {5'd0, ioctl_addr} + 32'd1;
		if (ioctl_addr[2:0] == 3'd7) flush <= 1'b1;
	end

	// end of download: write out a partial last word
	if (!ioctl_download && dl_d) begin
		if (be != 8'd0) flush <= 1'b1;
		if (dl_any && !dl_buf) loaded <= 1'b1;
		if (dl_buf) begin
			cart_seq  <= cart_seq + 16'd1;
			cart_slot <= ioctl_index[5:0];
		end
	end

	if (flush) begin
		if (!mem_req) begin
			mem_req   <= 1'b1;
			mem_addr  <= word;
			mem_wdata <= buf64;
			mem_be    <= be;
		end else if (mem_gnt) begin
			mem_we <= 1'b1;
			if (mem_we && !mem_busy) begin
				mem_we  <= 1'b0;
				mem_req <= 1'b0;
				flush   <= 1'b0;
				be      <= 8'd0;
			end
		end
	end
end

endmodule
