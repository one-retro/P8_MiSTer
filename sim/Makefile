# Verilator model of p8_soc. Needs verilator 5 (or `make docker`, which
# builds inside the mister-p8-verilator image: see README.md).
RTL := ../rtl/rv32_cpu.sv ../rtl/p8_bram.sv ../rtl/p8_icache.sv ../rtl/p8_dcache.sv ../rtl/p8_tunit.sv \
       ../rtl/p8_ddr_arb.sv ../rtl/p8_loader.sv ../rtl/p8_video.sv ../rtl/p8_audio.sv ../rtl/p8_apu.sv \
       ../rtl/p8_blit.sv ../rtl/p8_soc.sv
OBJ := obj_dir

all: $(OBJ)/Vp8_soc $(OBJ)/apu/Vp8_apu

$(OBJ)/Vp8_soc: $(RTL) sim_main.cpp
	verilator --cc --exe --build -O3 -j 0 --top-module p8_soc --public-flat-rw \
		-Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
		+define+P8_SPLASH=\"$(abspath ../rtl/splash.hex)\" \
		-CFLAGS "-O2" --Mdir $(OBJ) $(RTL) sim_main.cpp -o Vp8_soc

# the audio unit alone (apu_tb.cpp replays an operation trace)
$(OBJ)/apu/Vp8_apu: ../rtl/p8_apu.sv ../rtl/p8_bram.sv apu_tb.cpp
	verilator --cc --exe --build -O3 -j 0 --top-module p8_apu \
		-Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
		-CFLAGS "-O2" --Mdir $(OBJ)/apu $(abspath ../rtl/p8_apu.sv ../rtl/p8_bram.sv apu_tb.cpp) -o Vp8_apu

docker:
	docker run --rm -u $$(id -u):$$(id -g) -v $(abspath ..):$(abspath ..) -w $(abspath .) mister-p8-verilator make

clean:
	rm -rf $(OBJ)

.PHONY: all docker clean
