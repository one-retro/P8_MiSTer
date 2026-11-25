#!/bin/sh
# RV32IM, Zbb, hazard, performance counter, D$ write-back, divider and
# table-unit lookup cache self-tests (sim/tests) on the Verilator model of
# p8_soc, under several DDR timings. Verilator and clang run in the
# mister-p8-verilator Docker image (docker build -t mister-p8-verilator sim).
#
# usage: sim/cpu-tests.sh
set -e
cd "$(dirname "$0")/.."
img=mister-p8-verilator
out=sim/obj_dir/tests
mkdir -p "$out"
make -s -C sim docker > /dev/null
sim=sim/obj_dir/Vp8_soc
fail=0
for t in isa:rv32im zbb:rv32im_zbb hazard:rv32im_zbb perf:rv32im mem:rv32im div:rv32im hcache:rv32im; do
  n=${t%%:*}
  docker run --rm -u "$(id -u):$(id -g)" -v "$PWD:/w" -w /w $img \
    clang --target=riscv32 -march=${t#*:} -mabi=ilp32 -mno-relax -nostdlib -ffreestanding \
    -fuse-ld=lld -static -O2 -Isim/tests -Wl,-T,sim/tests/link.ld \
    sim/tests/$n.S -o $out/$n.elf
  python3 scripts/flatten-elf.py $out/$n.elf $out/$n.bin > /dev/null
  for cfg in "16 10" "0 0" "2 50" "40 30"; do
    set -- $cfg
    if $sim --image $out/$n.bin --max-cycles 20000000 --ddr-latency $1 --ddr-busy $2 2>/dev/null | grep -q "$n ok"; then
      echo "ok   $n (ddr latency $1, busy $2%)"
    else
      echo "FAIL $n (ddr latency $1, busy $2%)"; fail=1
    fi
  done
done
echo "fail=$fail"
exit $fail
