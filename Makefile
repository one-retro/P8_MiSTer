# P8 core. Quartus and Verilator run in Docker (see README.md).
#
#   make          the core: output_files/p8.rbf
#   make sim      the Verilator model: sim/obj_dir/Vp8_soc
#   make test     RV32IM and Zbb self-tests on the model
#   make splash   a new rtl/splash.hex into the last compile, without a new fit
#   make clean    remove Quartus and Verilator output

SIM_IMAGE := mister-p8-verilator

rbf:
	scripts/quartus.sh . quartus_sh --flow compile p8

sim-image:
	docker build -q -t $(SIM_IMAGE) sim > /dev/null

sim: sim-image
	$(MAKE) -C sim docker

test: sim-image
	sim/cpu-tests.sh

splash:
	scripts/update-splash.sh

# Quartus runs as root in its container, so its output is removed there too
clean:
	$(MAKE) -C sim clean
	scripts/quartus.sh . rm -rf db incremental_db output_files greybox_tmp hc_output simulation \
		build_id.v c5_pin_model_dump.txt PLLJ_PLLSPE_INFO.txt jtag.cdf

.PHONY: rbf sim-image sim test splash clean
