#!/usr/bin/env bash
# Standalone screenshot: launch our own openocd + gdb, halt, dump, resume, exit.
# Use this when VS Code Debug Console is broken or you don't want to start a
# debug session. Requires the device to be running (powered, no debug session
# currently attached to it).
set -euo pipefail

# Pick the most recent ELF.
ELF=$(ls -t /home/drew/github/pico-spec/build/bin/MinSizeRel/*.elf \
              /home/drew/github/pico-spec/build_picodvi/bin/Release/*.elf 2>/dev/null \
      | grep -v "bs2_default" | head -1)
if [[ -z "${ELF:-}" ]]; then
    echo "No ELF found"
    exit 1
fi
echo "Using ELF: $ELF"

GDB=$HOME/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb
OPENOCD=$HOME/.pico-sdk/openocd/0.12.0+dev/openocd
OPENOCD_SCRIPTS=$HOME/.pico-sdk/openocd/0.12.0+dev/scripts
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Start OpenOCD in background.
"$OPENOCD" -s "$OPENOCD_SCRIPTS" \
    -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
    -c "adapter speed 5000" \
    >/tmp/picospec_openocd.log 2>&1 &
OPENOCD_PID=$!
trap 'kill $OPENOCD_PID 2>/dev/null || true' EXIT

# Wait briefly for openocd to listen.
for _ in 1 2 3 4 5 6 7 8 9 10; do
    if (echo > /dev/tcp/127.0.0.1/3333) 2>/dev/null; then break; fi
    sleep 0.2
done

"$GDB" -batch \
    -ex "file $ELF" \
    -ex "target extended-remote localhost:3333" \
    -ex "set confirm off" \
    -ex "monitor halt" \
    -ex "source $SCRIPT_DIR/screenshot.gdb" \
    -ex "monitor resume" \
    -ex "detach" \
    -ex "quit"

"$SCRIPT_DIR/screenshot.sh"
