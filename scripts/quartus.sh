#!/bin/sh
# Run Quartus Prime Lite 17.0.2 (the version MiSTer cores standardise on)
# in Docker. Works on Apple Silicon through Docker Desktop's Rosetta x86
# emulation. The repository is mounted at /build.
#
# usage: scripts/quartus.sh <dir> <quartus command...>
#   scripts/quartus.sh . quartus_sh --flow compile p8
#   scripts/quartus.sh . quartus_sta -t report.tcl
#
# QUARTUS_IMAGE overrides the image.
set -e
IMAGE=${QUARTUS_IMAGE:-theypsilon/quartus-lite-c5:17.0.2.docker1}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
if [ $# -lt 2 ]; then
  sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
fi
dir=$1
shift
abs=$(cd "$dir" && pwd)
case "$abs" in
  "$ROOT"|"$ROOT"/*) rel=${abs#"$ROOT"} ;;
  *) echo "quartus.sh: $dir is not inside $ROOT" >&2; exit 2 ;;
esac
tty=
[ -t 1 ] && tty=-t
exec docker run --rm $tty --platform linux/amd64 \
  -v "$ROOT":/build -w "/build$rel" \
  "$IMAGE" "$@"
