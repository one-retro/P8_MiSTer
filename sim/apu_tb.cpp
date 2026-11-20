// Testbench for p8_apu alone: replays an operation trace (written by the
// firmware's model of the audio unit) and checks every sample the unit
// pushes to the FIFO or shows on ACC, and the cycles each operation takes.
//
// Trace lines: `op CTRL INC INC2 VOICE NOISE FX CYCLES` (hex, CYCLES
// decimal: from its start to the start of the next queued operation), then
// `s SAMPLE` per sample it pushes; `r SAMPLE` an ACC read, `n` an ACC write.
//
// usage: Vp8_apu TRACE [--stream]
//   default: one operation at a time (waits until the unit is idle), checks
//   the cycles; --stream: operations as fast as the queue takes them, with
//   the FIFO refusing a sample now and then (a CPU write), checks samples only.
#include "Vp8_apu.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

static Vp8_apu *m;
static unsigned long long cyc;
static std::deque<int> expect;
static long errors, checked;
static bool refuse;  // --stream: the FIFO refuses every 7th cycle
static unsigned long long progress;  // cycle of the last trace line
static long lineno;

static void tick() {
	m->push_ok = !(refuse && cyc % 7 == 3);
	m->clk = 0; m->eval();
	if (m->push && m->push_ok) {
		int s = (int16_t)m->push_data;
		if (expect.empty()) {
			if (errors++ < 10) fprintf(stderr, "cycle %llu: pushed %d, none expected\n", cyc, s);
		} else {
			if (expect.front() != s && errors++ < 10)
				fprintf(stderr, "cycle %llu: pushed %d, expected %d (sample %ld)\n", cyc, s, expect.front(), checked);
			expect.pop_front();
			checked++;
		}
	}
	m->clk = 1; m->eval();
	m->wr = 0;
	cyc++;
	if (cyc - progress > 2000000) {
		fprintf(stderr, "stuck at trace line %ld (cycle %llu, status %d)\n", lineno, cyc, m->status);
		exit(1);
	}
}

static void write(int reg, unsigned v) {
	m->wr = 1; m->wreg = reg; m->wdata = v;
	tick();
}

static bool busy() { m->eval(); return m->status & 1; }
static bool full() { m->eval(); return m->status & 2; }

int main(int argc, char **argv) {
	Verilated::commandArgs(argc, argv);
	if (argc < 2) { fprintf(stderr, "usage: Vp8_apu TRACE [--stream]\n"); return 2; }
	bool stream = argc > 2 && !strcmp(argv[2], "--stream");
	refuse = stream;
	FILE *f = fopen(argv[1], "r");
	if (!f) { perror(argv[1]); return 2; }
	m = new Vp8_apu;
	m->reset = 1; tick(); tick(); m->reset = 0;
	while (busy()) tick();

	std::vector<std::string> lines;
	char buf[256];
	while (fgets(buf, sizeof buf, f)) lines.push_back(buf);
	long ops = 0, offset = -1, bad_cycles = 0;
	for (size_t li = 0; li < lines.size(); li++) {
		char line[256];
		strcpy(line, lines[li].c_str());
		lineno = li + 1;
		progress = cyc;
		if (line[0] == 'o') {
			// the samples it pushes
			for (size_t k = li + 1; k < lines.size() && lines[k][0] == 's'; k++) expect.push_back(atoi(lines[k].c_str() + 2));
			unsigned c, inc, inc2, voice, noise, fx, cycles;
			sscanf(line, "op %x %x %x %x %x %x %u", &c, &inc, &inc2, &voice, &noise, &fx, &cycles);
			if (stream) { while (full()) tick(); }
			else { while (busy()) tick(); }
			if ((c >> 8 & 3) == 0) {
				write(2, inc); write(3, inc2); write(4, voice); write(5, noise); write(6, fx);
			}
			write(1, c);
			ops++;
			if (!stream) {
				// from the CTRL write to idle: the operation's cycles plus a
				// constant (the registered write, the idle cycle)
				unsigned long long t0 = cyc;
				while (busy()) tick();
				long d = (long)(cyc - t0) - (long)cycles;
				if (offset < 0) offset = d;
				if (d != offset && bad_cycles++ < 10)
					fprintf(stderr, "op %ld (%s): %llu cycles, model %u (+%ld)\n", ops, strtok(line, "\n"), cyc - t0, cycles, offset);
			}
		} else if (line[0] == 'r') {
			while (busy()) tick();
			tick(); tick();
			int s = (int16_t)m->acc_q;
			if (s != atoi(line + 2) && errors++ < 10) fprintf(stderr, "ACC read %d, expected %d\n", s, atoi(line + 2));
			checked++;
		} else if (line[0] == 'n') {
			while (busy()) tick();
			write(7, 0);
		}
	}
	progress = cyc;
	while (busy() || !expect.empty()) tick();
	if (!expect.empty()) { fprintf(stderr, "%zu samples never pushed\n", expect.size()); errors++; }
	printf("apu %s: %ld ops, %ld samples checked, %ld errors, %ld cycle mismatches (offset %ld), %llu cycles\n",
	       stream ? "stream" : "step", ops, checked, errors, bad_cycles, offset, cyc);
	delete m;
	return errors || bad_cycles ? 1 : 0;
}
