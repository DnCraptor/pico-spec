#!/usr/bin/env bash
# Convert pico-spec framebuffer dump (made by screenshot.gdb) to PNG and open it.
set -euo pipefail

FB=/tmp/picospec_fb.bin
PAL=/tmp/picospec_pal.bin
DIM=/tmp/picospec_dim.txt
OUT=/tmp/picospec_screen.png

if [[ ! -f "$FB" || ! -f "$PAL" || ! -f "$DIM" ]]; then
    echo "Missing dump files. Run screenshot.gdb in an active GDB session first."
    exit 1
fi

# GDB via Cortex-Debug writes file logs in MI format, e.g. ~"320 240\n".
# Strip MI noise and grab the first line that looks like "<num> <num>".
DIMLINE=$(grep -oE '[0-9]+ [0-9]+' "$DIM" | head -1)
if [[ -z "$DIMLINE" ]]; then
    echo "Bad dim file: $DIM"
    cat "$DIM"
    exit 1
fi
read -r W H <<< "$DIMLINE"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/fb2png.py" "$FB" "$PAL" "$W" "$H" "$OUT"

echo "$OUT"
