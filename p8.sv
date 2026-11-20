//============================================================================
//
//  P8 (1Retro): PICO-8 compatible fantasy console for MiSTer.
//
//  A soft RV32IM CPU runs carts compiled to RISC-V by `p8c` (.p8c files:
//  the PICO-8 runtime plus the compiled cart, loaded at address 0).
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];


`include "build_id.v"
localparam CONF_STR = {
	// save states: MiSTer Main's four slots of 16 MiB at DDR 0x3400_0000
	// (savestates/P8/<cart>_N.ss)
	"P8;SS34000000:1000000;",
	"-;",
	// carts for the BIOS (games/P8/boot.rom) to compile; .p8c compiled
	// with p8c (a program of its own without the BIOS)
	"FS1,P8 PNG,Load Cart;",
	"FS2,P8C,Load Compiled Cart;",
	// the cart's pause menu (as the Pause button opens it); R closes the OSD
	"R[11],Cart menu;",
	"-;",
	"R[12],Save state;",
	"R[13],Load state;",
	"O[15:14],State slot,1,2,3,4;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[10:9],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"O[4:3],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
	"O[5],Show FPS,Off,On;",
	"O[6],Pause when OSD open,On,Off;",
	"O[8:7],Mouse speed,Normal,Slow,Fast,Faster;",
	"-;",
	// Reset before the joystick lines: with them in between, MiSTer Main
	// ran the wrong entry for the Reset row (no status[0] pulse)
	"R[0],Reset;",
	"J1,O,X,Pause;",
	"jn,A,B,Start;",
	"jp,B,A,Start;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire [24:0] ps2_mouse;
wire [15:0] ps2_mouse_ext;
wire [31:0] joy0, joy1;
wire [21:0] gamma_bus;

wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_wait;
wire        img_mounted, img_readonly, sd_rd, sd_wr, sd_ack, sd_buff_wr;
wire [63:0] img_size;
wire [31:0] sd_lba;
wire [13:0] sd_buff_addr;
wire  [7:0] sd_buff_dout, sd_buff_din;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask(16'd0),

	.joystick_0(joy0),
	.joystick_1(joy1),
	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse),
	.ps2_mouse_ext(ps2_mouse_ext),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	// save file: FS above makes main mount saves/P8/<cart>.sav as disk 0
	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba('{sd_lba}),
	.sd_blk_cnt('{6'd0}),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din('{sd_buff_din}),
	.sd_buff_wr(sd_buff_wr)
);


///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.locked(pll_locked)
);

wire reset = RESET | status[0] | buttons[1] | !pll_locked;

///////////////////////   INPUT   ////////////////////////////////

// PICO-8 button bits: 0 left, 1 right, 2 up, 3 down, 4 O, 5 X, 6 pause.
// MiSTer joystick bits: 0 right, 1 left, 2 down, 3 up, 4.. buttons (J1 order).
function [7:0] p8_joy(input [31:0] j);
	p8_joy = {1'b0, j[6], j[5], j[4], j[2], j[3], j[0], j[1]};
endfunction

// Keyboard as in PICO-8. Player 0: cursor keys, Z/C/N = O, X/V/M = X,
// Enter/P = pause. Player 1: S/F/E/D = left/right/up/down, LShift/Tab = O,
// A/Q = X.
reg [7:0] kb0 = 0, kb1 = 0;
wire       kp = ps2_key[9];
reg        ps2_tog;
always @(posedge clk_sys) begin
	ps2_tog <= ps2_key[10];
	if (ps2_tog != ps2_key[10]) begin
		case ({ps2_key[8], ps2_key[7:0]})
			9'h16b: kb0[0] <= kp;  // left
			9'h174: kb0[1] <= kp;  // right
			9'h175: kb0[2] <= kp;  // up
			9'h172: kb0[3] <= kp;  // down
			9'h01a, 9'h021, 9'h031: kb0[4] <= kp;  // Z C N
			9'h022, 9'h02a, 9'h03a: kb0[5] <= kp;  // X V M
			9'h05a, 9'h04d: kb0[6] <= kp;          // Enter P
			9'h01b: kb1[0] <= kp;  // S
			9'h02b: kb1[1] <= kp;  // F
			9'h024: kb1[2] <= kp;  // E
			9'h023: kb1[3] <= kp;  // D
			9'h012, 9'h00d: kb1[4] <= kp;  // LShift Tab
			9'h01c, 9'h015: kb1[5] <= kp;  // A Q
			default: ;
		endcase
	end
end

wire [15:0] p8_buttons = {p8_joy(joy1) | kb1, p8_joy(joy0) | kb0};

// the cart pauses (at its next frame) while the OSD is open, unless the
// option is off
reg [1:0] osd_s = 2'd0;
always @(posedge clk_sys) osd_s <= {osd_s[0], OSD_STATUS};
wire       p8_pause = osd_s[1] && !status[6];

// OSD "Cart menu": a pulse on the trigger's rising edge
reg        menu_s = 1'b0;
always @(posedge clk_sys) menu_s <= status[11];
wire       p8_menu = status[11] && !menu_s;

// OSD "Save state" / "Load state": pulses likewise
reg  [1:0] ss_s = 2'd0;
always @(posedge clk_sys) ss_s <= status[13:12];
wire       p8_ss_save = status[12] && !ss_s[0];
wire       p8_ss_load = status[13] && !ss_s[1];

///////////////////////   CORE   /////////////////////////////////

wire ce_pix;
wire [7:0] r, g, b;
wire hs, vs, hblank, vblank;
wire [15:0] audio;
wire running;

p8_soc #(.CLK_HZ(100000000)) soc
(
	.clk(clk_sys),
	.reset(reset),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.buttons(p8_buttons),
	.pause(p8_pause),
	.menu(p8_menu),
	.ss_save(p8_ss_save),
	.ss_load(p8_ss_load),
	.ss_slot(status[15:14]),
	.mouse_speed(status[8:7]),
	.ps2_mouse(ps2_mouse),
	.mouse_wheel(ps2_mouse_ext[7:0]),
	.ps2_key(ps2_key),
	.frames(32'd0),
	.show_fps(status[5]),
	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size[31:0]),
	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr[8:0]),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	.DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.DDRAM_RD(DDRAM_RD),
	.DDRAM_WE(DDRAM_WE),
	.DDRAM_DIN(DDRAM_DIN),
	.DDRAM_BE(DDRAM_BE),
	.DDRAM_BUSY(DDRAM_BUSY),
	.DDRAM_DOUT(DDRAM_DOUT),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY),

	.ce_pix(ce_pix),
	.r(r), .g(g), .b(b),
	.hs(hs), .vs(vs), .hblank(hblank), .vblank(vblank),

	.audio(audio),

	.running(running),
	.con_wr(), .con_data(), .exit_wr(), .exit_code(), .frame_wr()
);

assign DDRAM_CLK = clk_sys;

///////////////////////   VIDEO   ////////////////////////////////

assign CLK_VIDEO = clk_sys;
assign VGA_SL = status[4] ? status[4:3] - 2'd1 : 2'd0;

wire vga_de;

video_mixer #(.LINE_LENGTH(400), .HALF_DEPTH(0), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.ce_pix(ce_pix),

	.scandoubler(forced_scandoubler || status[4:3] != 2'd0),
	.hq2x(status[4:3] == 2'd1),
	.gamma_bus(gamma_bus),

	.R(r), .G(g), .B(b),
	.HSync(hs), .VSync(vs), .HBlank(hblank), .VBlank(vblank),

	.HDMI_FREEZE(1'b0),
	.freeze_sync(),

	.VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B),
	.VGA_VS(VGA_VS), .VGA_HS(VGA_HS), .VGA_DE(vga_de)
);

// 128x128 square pixels; integer scaling by the framework's scaler
video_freak video_freak
(
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.VGA_VS(VGA_VS),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),

	.VGA_DE_IN(vga_de),
	.ARX((!ar) ? 12'd1 : (ar - 1'd1)),
	.ARY((!ar) ? 12'd1 : 12'd0),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE({1'b0, status[10:9]})
);

///////////////////////   AUDIO   ////////////////////////////////

assign AUDIO_S   = 1;
assign AUDIO_MIX = 0;
assign AUDIO_L   = audio;
assign AUDIO_R   = audio;

///////////////////////   LEDS   /////////////////////////////////

assign LED_USER = ioctl_download | !running;

endmodule
