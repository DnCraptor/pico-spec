set pagination off
set $w = VIDEO::vga.xres
set $h = VIDEO::vga.yres
set $fb0 = VIDEO::vga.frameBuffer[0]
printf "screenshot: %dx%d fb=%p\n", $w, $h, $fb0
dump binary memory /tmp/picospec_fb.bin $fb0 ($fb0 + $w * $h)
# Dump 1KB at $fb0-aligned dummy zone — NOT a real palette. fb2png.py will
# auto-detect that the palette doesn't look right and fall back to the
# standard ZX palette, which is what we want for HDMI build anyway (real
# palette is optimized away into TMDS conv_color).
dump binary memory /tmp/picospec_pal.bin $fb0 ($fb0 + 1024)
printf "screenshot: palette placeholder dumped (ZX fallback will be used)\n"
set logging file /tmp/picospec_dim.txt
set logging overwrite on
set logging redirect on
set logging enabled on
printf "%d %d\n", $w, $h
set logging enabled off
printf "screenshot: dump done\n"
