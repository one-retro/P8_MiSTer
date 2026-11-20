// Verilator testbench for p8_soc: DDR3 Avalon model, HPS ioctl download of
// a program (p8c image, or any RV32IM flat image), console/exit capture,
// screen hash (FNV-1a 64 of the screen), video and audio capture.
//
// usage: Vp8_soc --image prog.p8c [--frames N] [--max-cycles N]
//                [--screen-hash] [--blit-stats] [--ppm out.ppm] [--check-video] [--wav out.wav] [--show-fps]
//                [--screen-hex out.hex] [--buttons HEX] [--trace-pc]
//                [--ddr-latency N] [--ddr-busy PERCENT] [--seed N] [--reset-at-frame K] [--reset-at-cycle C] [--reset-on-read N] [--reset-cycles N]
//                [--pause CYCLE:LENGTH]
//                [--ss-save FRAME] [--ss-load FRAME] [--ss-slot N] (OSD Save / Load state
//                once FRAME frames are done, into / from slot N 0-3)
//                [--ppm-at CYCLE:out.ppm]... (the last complete video frame at that cycle)
//                [--mouse FRAME:DX,DY,BUTTONS,WHEEL]... (a PS/2 mouse packet once FRAME frames are done)
//                [--key FRAME:EVENT]... (an hps_io key event {pressed, extended, code}, hex)
//                [--sav FILE] (save file: mounted as main does after the download,
//                              created on the first write, written back at exit)
//                [--download FRAME:INDEX:FILE]... (an OSD download once FRAME frames
//                              are done: index 1 Load Cart, 2 Load Compiled Cart)
//
// The image is downloaded as boot.rom (index 0): the program at address 0.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <string>
#include <vector>

#include "Vp8_soc.h"
#include "Vp8_soc___024root.h"
#include "verilated.h"

static std::vector<uint8_t> read_file(const char* path) {
	FILE* f = fopen(path, "rb");
	if (!f) { perror(path); exit(2); }
	std::vector<uint8_t> d;
	uint8_t buf[65536];
	size_t n;
	while ((n = fread(buf, 1, sizeof buf, f)) > 0) d.insert(d.end(), buf, buf + n);
	fclose(f);
	return d;
}

static uint64_t rng_state = 1;
static uint32_t rnd() {
	rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
	return (uint32_t)rng_state;
}

// DDR model: 32 MiB window at DDR byte address 0x3000_0000, then the
// 64 MiB save state area at 0x3400_0000.
struct Ddr {
	std::vector<uint64_t> mem = std::vector<uint64_t>(96 * 1024 * 1024 / 8);
	int latency = 16;
	int busy_pct = 10;
	// pending reads: (ready cycle, word address, beats)
	struct Rd { uint64_t at; uint32_t addr; int beats; };
	std::vector<Rd> rds;
	int wr_left = 0;      // beats left in a write burst
	uint32_t wr_addr = 0;
	bool busy = false;
	uint64_t errors = 0;
	uint64_t reads = 0;   // read bursts accepted

	uint32_t map(uint32_t a29) {
		if ((a29 >> 23) == 0x0d) return 0x400000 + (a29 & 0x7fffff);
		if ((a29 >> 25) != 3 || (a29 & 0x1c00000)) {
			if (errors++ < 5) fprintf(stderr, "ddr: address outside the core window: %07x\n", a29);
		}
		return a29 & 0x3fffff;
	}

	// evaluated before the clock edge with the DUT outputs of this cycle
	void edge(Vp8_soc* t, uint64_t cyc) {
		bool accept = !busy;
		if (accept && t->DDRAM_RD) {
			if (wr_left) { fprintf(stderr, "ddr: read during write burst\n"); errors++; }
			rds.push_back({cyc + latency, map(t->DDRAM_ADDR), t->DDRAM_BURSTCNT});
			reads++;
		}
		if (accept && t->DDRAM_WE) {
			uint32_t a;
			if (wr_left == 0) { a = map(t->DDRAM_ADDR); wr_left = t->DDRAM_BURSTCNT; wr_addr = a; }
			a = wr_addr++;
			uint64_t v = mem[a];
			for (int i = 0; i < 8; i++)
				if (t->DDRAM_BE & (1 << i)) v = (v & ~(0xffull << (8 * i))) | ((uint64_t)((t->DDRAM_DIN >> (8 * i)) & 0xff) << (8 * i));
			mem[a] = v;
			wr_left--;
		}
	}
	// drive inputs for the next cycle
	void drive(Vp8_soc* t, uint64_t cyc) {
		busy = (int)(rnd() % 100) < busy_pct;
		t->DDRAM_BUSY = busy;
		t->DDRAM_DOUT_READY = 0;
		if (!rds.empty() && rds[0].at <= cyc) {
			Rd& r = rds[0];
			t->DDRAM_DOUT = mem[r.addr];
			t->DDRAM_DOUT_READY = 1;
			r.addr++;
			if (--r.beats == 0) rds.erase(rds.begin());
		}
	}
};

// HPS download: one byte every few cycles, honouring ioctl_wait.
struct Download {
	std::vector<uint8_t> data;
	int index;
	size_t pos = 0;
	int phase = 0;   // 0 = idle, 1 = start, 2 = sending, 3 = end
	int delay = 0;
	int frame = 0;   // starts once this many frames are done
};

static const uint32_t PAL[32] = {
	0x000000, 0x1d2b53, 0x7e2553, 0x008751, 0xab5236, 0x5f574f, 0xc2c3c7, 0xfff1e8,
	0xff004d, 0xffa300, 0xffec27, 0x00e436, 0x29adff, 0x83769c, 0xff77a8, 0xffccaa,
	0x291814, 0x111d35, 0x422136, 0x125359, 0x742f29, 0x49333b, 0xa28879, 0xf3ef7d,
	0xbe1250, 0xff6c24, 0xa8e72e, 0x00b543, 0x065ab5, 0x754665, 0xff6e59, 0xff9d81,
};

// p8_gfx::render_rgba (screen palette, screen modes, raster modes)
static void render(const uint8_t* ram, uint32_t* out) {
	auto argb = [](uint8_t p) { return PAL[(p & 0xf) | ((p & 0x80) >> 3)]; };
	int sb = ram[0x5f55] << 8;
	uint8_t mode = ram[0x5f2c], raster = ram[0x5f5f];
	auto rbit = [&](int y) { return (ram[0x5f70 + y / 8] >> (y % 8)) & 1; };
	for (int y = 0; y < 128; y++)
		for (int x = 0; x < 128; x++) {
			int sx, sy;
			if ((mode & 0xbc) == 0x84) {
				int xx = x, yy = y;
				if (mode & 1) std::swap(xx, yy);
				sx = (mode & 2) ? 127 - xx : xx;
				sy = ((uint8_t)(mode + 1) & 2) ? 127 - yy : yy;
			} else {
				switch (mode & 0xbd) { case 0x05: sx = std::min(x, 127 - x); break; case 0x01: sx = x / 2; break; case 0x81: sx = 127 - x; break; default: sx = x; }
				switch (mode & 0xbe) { case 0x06: sy = std::min(y, 127 - y); break; case 0x02: sy = y / 2; break; case 0x82: sy = 127 - y; break; default: sy = y; }
			}
			uint8_t b = ram[(sb + sy * 64 + sx / 2) & 0xffff];
			int c = (b >> ((sx & 1) * 4)) & 0xf;
			uint32_t v;
			if (raster == 0x10 && rbit(sy)) v = argb(ram[0x5f60 + c]);
			else if ((raster & 0x30) == 0x30 && (raster & 0xf) == c) v = argb(ram[0x5f60 + (sy / 8 + rbit(sy)) % 16]);
			else v = argb(ram[0x5f10 + c]);
			out[y * 128 + x] = v;
		}
}

static void write_ppm(const char* path, const std::vector<uint32_t>& src) {
	FILE* f = fopen(path, "wb");
	fprintf(f, "P6\n128 128\n255\n");
	for (uint32_t p : src) { uint8_t px[3] = {(uint8_t)(p >> 16), (uint8_t)(p >> 8), (uint8_t)p}; fwrite(px, 1, 3, f); }
	fclose(f);
}

int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);
	const char* image = nullptr;
	const char* screen_hex = nullptr;
	const char* ppm = nullptr;
	const char* wav = nullptr;
	const char* sav = nullptr;
	uint32_t frames = 0;
	uint64_t max_cycles = 2000000000ull;
	bool hash = false, trace_pc = false, check_video = false, show_fps = false, blit_stats = false, blit_trace = false;
	uint64_t blt_ops = 0, blt_cyc = 0;
	uint32_t buttons = 0;
	int reset_at = 0;
	uint64_t reset_cyc = 0;
	uint64_t reset_rd = 0;
	uint64_t reset_len = 16;   // --reset-cycles N: length of those resets (OSD Reset: a few us)   // --reset-on-read N: reset right after the N-th DDR read
	uint64_t pause_from = 0, pause_to = 0;   // --pause CYCLE:LENGTH (OSD open)
	std::vector<std::pair<uint64_t, std::string>> snaps;   // --ppm-at CYCLE:PATH
	// --mouse / --key: input events by frame, applied in order, a few
	// cycles apart (each is one hps_io toggle)
	struct InputEv { int frame; bool mouse; int dx, dy, btn, wheel, key; };
	std::vector<InputEv> inputs;
	std::vector<Download> later;   // --download
	int ss_save_at = -1, ss_load_at = -1, ss_slot = 0;
	uint64_t next_input = 0;
	Ddr ddr;
	for (int i = 1; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() { if (i + 1 >= argc) { fprintf(stderr, "%s needs a value\n", a.c_str()); exit(2); } return argv[++i]; };
		if (a == "--image") image = next();
		else if (a == "--screen-hex") screen_hex = next();
		else if (a == "--frames") frames = strtoul(next(), 0, 0);
		else if (a == "--max-cycles") max_cycles = strtoull(next(), 0, 0);
		else if (a == "--screen-hash") hash = true;
		else if (a == "--blit-stats") blit_stats = true;
		else if (a == "--blit-trace") blit_trace = true;
		else if (a == "--ppm") ppm = next();
		else if (a == "--wav") wav = next();
		else if (a == "--buttons") buttons = strtoul(next(), 0, 0);
		else if (a == "--trace-pc") trace_pc = true;
		else if (a == "--check-video") check_video = true;
		else if (a == "--show-fps") show_fps = true;
		else if (a == "--ddr-latency") ddr.latency = atoi(next());
		else if (a == "--ddr-busy") ddr.busy_pct = atoi(next());
		else if (a == "--seed") rng_state = strtoull(next(), 0, 0) | 1;
		else if (a == "--reset-at-frame") reset_at = atoi(next());
		else if (a == "--reset-at-cycle") reset_cyc = strtoull(next(), 0, 0);
		else if (a == "--reset-on-read") reset_rd = strtoull(next(), 0, 0);
		else if (a == "--reset-cycles") reset_len = strtoull(next(), 0, 0);
		else if (a == "--pause") {
			std::string v = next();
			size_t c = v.find(':');
			pause_from = strtoull(v.substr(0, c).c_str(), 0, 0);
			pause_to = pause_from + strtoull(v.substr(c + 1).c_str(), 0, 0);
		}
		else if (a == "--ppm-at") {
			std::string v = next();
			size_t c = v.find(':');
			snaps.push_back({strtoull(v.substr(0, c).c_str(), 0, 0), v.substr(c + 1)});
		}
		else if (a == "--mouse") {
			InputEv e = {0, true, 0, 0, 0, 0, 0};
			if (sscanf(next(), "%d:%d,%d,%d,%d", &e.frame, &e.dx, &e.dy, &e.btn, &e.wheel) != 5) { fprintf(stderr, "bad --mouse\n"); return 2; }
			inputs.push_back(e);
		}
		else if (a == "--key") {
			InputEv e = {0, false, 0, 0, 0, 0, 0};
			if (sscanf(next(), "%d:%x", &e.frame, &e.key) != 2) { fprintf(stderr, "bad --key\n"); return 2; }
			inputs.push_back(e);
		}
		else if (a == "--sav") sav = next();
		else if (a == "--ss-save") ss_save_at = atoi(next());
		else if (a == "--ss-load") ss_load_at = atoi(next());
		else if (a == "--ss-slot") ss_slot = atoi(next()) & 3;
		else if (a == "--download") {
			std::string v = next();
			size_t c1 = v.find(':'), c2 = v.find(':', c1 + 1);
			if (c1 == std::string::npos || c2 == std::string::npos) { fprintf(stderr, "--download FRAME:INDEX:FILE\n"); return 2; }
			Download d;
			d.frame = atoi(v.substr(0, c1).c_str());
			d.index = atoi(v.substr(c1 + 1, c2 - c1 - 1).c_str());
			d.data = read_file(v.substr(c2 + 1).c_str());
			later.push_back(d);
		}
		else if (a[0] == '+') ;
		else { fprintf(stderr, "unknown argument %s\n", a.c_str()); return 2; }
	}
	if (!image) { fprintf(stderr, "usage: %s --image prog.p8c ...\n", argv[0]); return 2; }

	// save file (hps_io virtual disk 0): its contents, and the transfer in
	// progress: 0 idle, 1 read, 2 write; byte position and pacing
	std::vector<uint8_t> sav_data;
	if (sav) { FILE* f = fopen(sav, "rb"); if (f) { fclose(f); sav_data = read_file(sav); } }
	bool sav_dirty = false;
	int sd_op = 0, sd_pos = 0, sd_delay = 0;
	bool sd_half = false;

	std::vector<Download> dls;
	dls.push_back({read_file(image), 0});
	for (auto& d : later) dls.push_back(d);
	size_t cur = 0;

	Vp8_soc* t = new Vp8_soc;
	t->clk = 0;
	t->reset = 1;
	t->frames = frames;
	t->buttons = buttons;
	t->show_fps = show_fps;
	t->ioctl_download = 0;
	t->ioctl_wr = 0;
	t->img_mounted = 0;
	t->sd_ack = 0;
	t->sd_buff_wr = 0;
	t->eval();

	// video capture
	std::vector<uint32_t> img(128 * 128, 0), last(128 * 128, 0);
	int vx = 0, vy = 0, vframes = 0;
	bool prev_vs = false, prev_hs = false, had_de_line = false;

	uint64_t cyc = 0;
	int exit_code = -1;
	uint64_t last_frame_cyc = 0;
	int nframes = 0;
	int stop_vframe = 0;
	std::string con;
	std::vector<int16_t> samples;
	// cycles per CATEGORY (0 cpu, 1 gfx, 2 system incl. vblank wait, 3 audio), after frame 2
	uint64_t cat_cyc[4] = {0, 0, 0, 0};
	int cat = 2;
	uint64_t acc = 0;

	uint64_t reset_end = 16;
	for (; cyc < max_cycles; cyc++) {
		if (cyc == reset_end) t->reset = 0;
		if (reset_rd && ddr.reads >= reset_rd) { reset_cyc = cyc; reset_rd = 0; }
		t->pause = cyc >= pause_from && cyc < pause_to;
		t->ss_slot = ss_slot;
		t->ss_save = ss_save_at >= 0 && nframes >= ss_save_at;
		t->ss_load = ss_load_at >= 0 && nframes >= ss_load_at;
		if (t->ss_save) ss_save_at = -1;
		if (t->ss_load) ss_load_at = -1;
		if (!inputs.empty() && nframes >= inputs[0].frame && cyc >= next_input) {
			InputEv e = inputs[0];
			inputs.erase(inputs.begin());
			if (e.mouse) {
				// PS/2 packet: status {y sign, x sign, 1, middle, right, left}, x, y (up positive)
				int st = (e.btn & 7) | 8 | (e.dx < 0 ? 0x10 : 0) | (e.dy < 0 ? 0x20 : 0);
				t->ps2_mouse = ((t->ps2_mouse ^ 0x1000000) & 0x1000000) | ((e.dy & 0xff) << 16) | ((e.dx & 0xff) << 8) | st;
				t->mouse_wheel = e.wheel & 0xff;
			} else {
				t->ps2_key = ((t->ps2_key ^ 0x400) & 0x400) | (e.key & 0x3ff);
			}
			next_input = cyc + 4;
		}
		for (auto& sn : snaps)
			if (sn.first == cyc) write_ppm(sn.second.c_str(), last);
		// a reset while the program runs (OSD Reset): it starts again
		if ((reset_at > 0 && nframes == reset_at && reset_end < cyc) || (reset_cyc && cyc == reset_cyc)) {
			fprintf(stderr, "[%llu] reset after frame %d\n", (unsigned long long)cyc, nframes);
			nframes = 0;
			reset_cyc = 0;
			t->reset = 1;
			reset_end = cyc + reset_len;
			reset_at = 0;
		}

		// ---- inputs for this cycle
		t->ioctl_wr = 0;
		if (cur < dls.size() && cyc > 32 && (dls[cur].phase != 0 || nframes >= dls[cur].frame)) {
			Download& d = dls[cur];
			switch (d.phase) {
			case 0: t->ioctl_download = 1; t->ioctl_index = d.index; d.phase = 1; d.delay = 4; break;
			case 1: if (--d.delay <= 0) d.phase = 2; break;
			case 2:
				if (d.delay > 0) { d.delay--; break; }
				if (t->ioctl_wait) break;
				if (d.pos < d.data.size()) {
					t->ioctl_addr = d.pos;
					t->ioctl_dout = d.data[d.pos];
					t->ioctl_wr = 1;
					d.pos++;
					d.delay = 3;
				} else {
					d.phase = 3;
					d.delay = 8;
				}
				break;
			case 3:
				// main mounts the save file before it ends the download
				if (sav && d.delay == 4) { t->img_mounted = 1; t->img_size = sav_data.size(); t->img_readonly = 0; }
				if (d.delay == 3) t->img_mounted = 0;
				if (--d.delay <= 0) {
					t->ioctl_download = 0;
					fprintf(stderr, "[%llu] downloaded index %d (%zu bytes)\n", (unsigned long long)cyc, d.index, d.data.size());
					cur++;
				}
				break;
			}
		}
		// save file block transfers, a byte every 4 cycles
		t->sd_buff_wr = 0;
		if (sav) {
			if (sd_op == 0 && (t->sd_rd || t->sd_wr)) {
				sd_op = t->sd_rd ? 1 : 2; sd_pos = 0; sd_delay = 8; sd_half = false; t->sd_ack = 1;
			} else if (sd_op && --sd_delay <= 0) {
				size_t base = (size_t)t->sd_lba * 512;
				if (sd_pos == 512) { t->sd_ack = 0; sd_op = 0; }
				else if (sd_op == 1) {
					t->sd_buff_addr = sd_pos;
					t->sd_buff_dout = base + sd_pos < sav_data.size() ? sav_data[base + sd_pos] : 0;
					t->sd_buff_wr = 1;
					sd_pos++; sd_delay = 4;
				} else if (!sd_half) {
					// address now, data (sd_buff_din, RAM latency) a few cycles later
					t->sd_buff_addr = sd_pos; sd_half = true; sd_delay = 3;
				} else {
					if (sav_data.size() < base + 512) sav_data.resize(base + 512, 0);
					sav_data[base + sd_pos] = t->sd_buff_din;
					sav_dirty = true;
					sd_pos++; sd_half = false; sd_delay = 1;
				}
			}
		}
		ddr.drive(t, cyc);

		t->clk = 0;
		t->eval();
		// ---- sample outputs (state before the rising edge)
		ddr.edge(t, cyc);

		if (t->ce_pix) {
			bool de = !t->hblank && !t->vblank;
			if (t->vs && !prev_vs) { vy = 0; if (vframes++ > 0) last = img; }
			if (t->hs && !prev_hs) { if (had_de_line) vy++; had_de_line = false; vx = 0; }
			if (de) {
				if (vx < 128 && vy < 128) img[vy * 128 + vx] = (t->r << 16) | (t->g << 8) | t->b;
				vx++;
				had_de_line = true;
			}
			prev_vs = t->vs;
			prev_hs = t->hs;
		}

		// audio output at 22050 Hz (CLK_HZ = 100 MHz)
		acc += 22050;
		if (acc >= 100000000) { acc -= 100000000; samples.push_back((int16_t)t->audio); }
		{
			auto& r = t->rootp;
			if (r->p8_soc__DOT__mmio_wr && r->p8_soc__DOT__mreg == 2) cat = r->p8_soc__DOT__dm_wdata & 3;
			if (nframes >= 2) cat_cyc[cat]++;
			// blitter: operations started and cycles busy
			if (blit_trace && r->p8_soc__DOT__mmio_wr && r->p8_soc__DOT__mreg == 0x041)
				printf("ctrl store %u at %llu (blitter %s)\n", (unsigned)(r->p8_soc__DOT__dm_wdata & 15), (unsigned long long)cyc,
				       r->p8_soc__DOT__blt_busy ? "busy" : "idle");
			if (r->p8_soc__DOT__blit__DOT__start) {
				blt_ops++;
				// --blit-trace: the registers as the operation starts
				if (blit_trace)
					printf("blt op %u dst %x size %x srow %x scol %x tl8 %x\n", (unsigned)(r->p8_soc__DOT__blit__DOT__wdata & 15),
					       (unsigned)r->p8_soc__DOT__blit__DOT__r_dst, (unsigned)r->p8_soc__DOT__blit__DOT__r_size,
					       (unsigned)r->p8_soc__DOT__blit__DOT__r_srow, (unsigned)r->p8_soc__DOT__blit__DOT__r_scol,
					       (unsigned)r->p8_soc__DOT__blit__DOT__r_tl[8]);
			}
			if (r->p8_soc__DOT__blt_busy) blt_cyc++;
		}
		if (t->con_wr) {
			char c = (char)t->con_data;
			if (c == '\n') { printf("console: %s\n", con.c_str()); con.clear(); }
			else con += c;
		}
		if (t->frame_wr) {
			nframes++;
			uint64_t d = cyc - last_frame_cyc;
			if (nframes <= 3 || nframes % 30 == 0)
				fprintf(stderr, "[%llu] frame %d (%llu cycles since last)\n", (unsigned long long)cyc, nframes, (unsigned long long)d);
			last_frame_cyc = cyc;
		}
		if (t->exit_wr && exit_code < 0) { exit_code = (int)t->exit_code; }

		t->clk = 1;
		t->eval();

		if (trace_pc && t->rootp->p8_soc__DOT__cpu__DOT__w_valid)
			fprintf(stderr, "pc %08x\n", t->rootp->p8_soc__DOT__cpu__DOT__d_pc);
		if (exit_code >= 0) {
			// keep running until the last presented frame has been scanned out
			bool video = ppm || check_video || wav;
			if (stop_vframe == 0) stop_vframe = vframes + 2;
			if (!video || vframes >= stop_vframe) break;
		}
	}
	if (!con.empty()) printf("console: %s\n", con.c_str());

	fprintf(stderr, "stopped after %llu cycles, %d frames, exit %d\n", (unsigned long long)cyc, nframes, exit_code);
	if (nframes > 2) {
		double n = nframes - 2;
		fprintf(stderr, "per frame after frame 2: cpu %.0f, gfx %.0f, audio %.0f, system+wait %.0f cycles\n",
			cat_cyc[0] / n, cat_cyc[1] / n, cat_cyc[3] / n, cat_cyc[2] / n);
	}
	if (ddr.errors) fprintf(stderr, "ddr: %llu protocol errors\n", (unsigned long long)ddr.errors);

	// PICO-8 RAM contents
	std::vector<uint8_t> ram(0x10000);
	{
		auto& r = t->rootp;
		for (int a = 0; a < 0x10000; a++) {
			int w = a >> 2;
			switch (a & 3) {
			case 0: ram[a] = r->p8_soc__DOT__p8lane__BRA__0__KET____DOT__ram__DOT__mem[w]; break;
			case 1: ram[a] = r->p8_soc__DOT__p8lane__BRA__1__KET____DOT__ram__DOT__mem[w]; break;
			case 2: ram[a] = r->p8_soc__DOT__p8lane__BRA__2__KET____DOT__ram__DOT__mem[w]; break;
			default: ram[a] = r->p8_soc__DOT__p8lane__BRA__3__KET____DOT__ram__DOT__mem[w]; break;
			}
		}
	}
	if (hash) {
		// FNV-1a 64 of PICO-8 RAM 0x6000-0x7fff
		uint64_t h = 0xcbf29ce484222325ull;
		for (int a = 0x6000; a < 0x8000; a++) { h ^= ram[a]; h *= 0x100000001b3ull; }
		printf("screen-hash %016llx\n", (unsigned long long)h);
	}
	if (blit_stats) printf("blit-stats %llu ops %llu cycles\n", (unsigned long long)blt_ops, (unsigned long long)blt_cyc);
	if (screen_hex) {
		// the screen as 2048 little-endian words, for $readmemh (p8_video splash)
		FILE* f = fopen(screen_hex, "w");
		for (int w = 0; w < 2048; w++) {
			const uint8_t* b = &ram[0x6000 + w * 4];
			fprintf(f, "%02x%02x%02x%02x\n", b[3], b[2], b[1], b[0]);
		}
		fclose(f);
	}
	int video_bad = 0;
	if (check_video) {
		// the last frame scanned out must be the PICO-8 screen at exit
		std::vector<uint32_t> want(128 * 128);
		render(ram.data(), want.data());
		const std::vector<uint32_t>& got = vframes > 1 ? last : img;
		for (int i = 0; i < 128 * 128; i++)
			if (got[i] != want[i] && video_bad++ < 5)
				fprintf(stderr, "video: pixel %d,%d is %06x, want %06x\n", i % 128, i / 128, got[i], want[i]);
		printf("video %s (%d pixels differ)\n", video_bad ? "MISMATCH" : "ok", video_bad);
	}
	if (wav) {
		FILE* f = fopen(wav, "wb");
		uint32_t n = samples.size() * 2;
		auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
		auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
		fwrite("RIFF", 1, 4, f); u32(36 + n); fwrite("WAVEfmt ", 1, 8, f);
		u32(16); u16(1); u16(1); u32(22050); u32(44100); u16(2); u16(16);
		fwrite("data", 1, 4, f); u32(n);
		fwrite(samples.data(), 2, samples.size(), f);
		fclose(f);
		int nz = 0; for (int16_t v : samples) nz += v != 0;
		fprintf(stderr, "audio: %zu samples, %d non-zero\n", samples.size(), nz);
	}
	if (ppm) {
		write_ppm(ppm, vframes > 1 ? last : img);
		fprintf(stderr, "video frames seen: %d\n", vframes);
	}
	if (sav && sav_dirty) {
		FILE* f = fopen(sav, "wb");
		fwrite(sav_data.data(), 1, sav_data.size(), f);
		fclose(f);
	}
	t->final();
	delete t;
	return exit_code < 0 ? 124 : video_bad ? 3 : exit_code;
}
