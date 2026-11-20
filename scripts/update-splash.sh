#!/bin/sh
# Put the current rtl/splash.hex into an already compiled project and
# rebuild only the bitstream: the placement and timing stay those of the
# last full compile, so a new splash needs no new fit (or seed sweep).
#
# usage: scripts/update-splash.sh [DIR]   (default: this directory, after
#        `scripts/quartus.sh . quartus_sh --flow compile p8`)
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
dir=$(cd "${1:-$here}" && pwd)
# the display buffer's init file that synthesis generated from splash.hex
mif=$(grep -l '"rtl/splash.hex"' "$dir"/db/p8.ram0_p8_bram_*.hdl.mif)
[ "$(echo "$mif" | wc -l)" = 1 ] || { echo "update-splash.sh: no single splash init file in $dir/db" >&2; exit 1; }
# written next to it and copied in by the container: Quartus (as root in
# Docker) owns db/
tmp="$dir/splash.mif.tmp"
python3 - "$mif" "$dir/rtl/splash.hex" "$tmp" <<'PY'
import sys
mif, hexf, out = sys.argv[1], sys.argv[2], sys.argv[3]
words = [int(w, 16) for w in open(hexf).read().split()]
assert len(words) == 2048, "splash.hex must have 2048 words"
lines = open(mif).read().split('\n')
i = lines.index('CONTENT BEGIN')
body = [f"\t{a} :\t{words[a]:032b};" for a in range(2047, -1, -1)]
open(out, 'w').write('\n'.join(lines[:i + 1] + body + ['END;', '']))
PY
"$here/scripts/quartus.sh" "$dir" cp splash.mif.tmp "db/$(basename "$mif")"
rm -f "$tmp"
"$here/scripts/quartus.sh" "$dir" quartus_cdb p8 --update_mif
"$here/scripts/quartus.sh" "$dir" quartus_asm p8
echo "$dir/output_files/p8.rbf"
