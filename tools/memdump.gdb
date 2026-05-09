# memdump.gdb — dump Z80 address space + all 8 physical RAM pages (128K)
# Usage (in Debug Console): source tools/memdump.gdb
# Then run task "Pico: Memory Dump" (Ctrl+Alt+D)

# Logical Z80 address space via current mapping (4x16K window 0x0000-0xFFFF)
dump binary memory /tmp/picospec_mem0.bin MemESP::ramCurrent[0] (MemESP::ramCurrent[0] + 16384)
dump binary memory /tmp/picospec_mem1.bin MemESP::ramCurrent[1] (MemESP::ramCurrent[1] + 16384)
dump binary memory /tmp/picospec_mem2.bin MemESP::ramCurrent[2] (MemESP::ramCurrent[2] + 16384)
dump binary memory /tmp/picospec_mem3.bin MemESP::ramCurrent[3] (MemESP::ramCurrent[3] + 16384)

# All 8 physical RAM pages (128K) via MemESP::ram[N]._int->p
# Note: pages backed by PSRAM_SPI or SWAP read whatever was last synced into the
# cache buffer at _int->p; non-cached PSRAM/SWAP regions cannot be dumped here.
dump binary memory /tmp/picospec_ram0.bin MemESP::ram[0]._int->p (MemESP::ram[0]._int->p + 16384)
dump binary memory /tmp/picospec_ram1.bin MemESP::ram[1]._int->p (MemESP::ram[1]._int->p + 16384)
dump binary memory /tmp/picospec_ram2.bin MemESP::ram[2]._int->p (MemESP::ram[2]._int->p + 16384)
dump binary memory /tmp/picospec_ram3.bin MemESP::ram[3]._int->p (MemESP::ram[3]._int->p + 16384)
dump binary memory /tmp/picospec_ram4.bin MemESP::ram[4]._int->p (MemESP::ram[4]._int->p + 16384)
dump binary memory /tmp/picospec_ram5.bin MemESP::ram[5]._int->p (MemESP::ram[5]._int->p + 16384)
dump binary memory /tmp/picospec_ram6.bin MemESP::ram[6]._int->p (MemESP::ram[6]._int->p + 16384)
dump binary memory /tmp/picospec_ram7.bin MemESP::ram[7]._int->p (MemESP::ram[7]._int->p + 16384)

set logging file /tmp/picospec_regs.txt
set logging overwrite on
set logging redirect on
set logging enabled on

printf "AF=%04X\n",  (unsigned short)((Z80::regA << 8) | (Z80::carryFlag ? Z80::sz5h3pnFlags | 1 : Z80::sz5h3pnFlags))
printf "BC=%04X\n",  (unsigned short)Z80::regBC.word
printf "DE=%04X\n",  (unsigned short)Z80::regDE.word
printf "HL=%04X\n",  (unsigned short)Z80::regHL.word
printf "AFx=%04X\n", (unsigned short)Z80::regAFx.word
printf "BCx=%04X\n", (unsigned short)Z80::regBCx.word
printf "DEx=%04X\n", (unsigned short)Z80::regDEx.word
printf "HLx=%04X\n", (unsigned short)Z80::regHLx.word
printf "IX=%04X\n",  (unsigned short)Z80::regIX.word
printf "IY=%04X\n",  (unsigned short)Z80::regIY.word
printf "SP=%04X\n",  (unsigned short)Z80::regSP.word
printf "PC=%04X\n",  (unsigned short)Z80::regPC.word
printf "I=%02X\n",   (unsigned char)Z80::regI
printf "R=%02X\n",   (unsigned char)Z80::regR
printf "IM=%d\n",    (int)Z80::modeINT
printf "IFF1=%d\n",  (int)Z80::ffIFF1
printf "IFF2=%d\n",  (int)Z80::ffIFF2
printf "halted=%d\n",(int)Z80::halted
printf "bankLatch=%d\n",  (int)MemESP::bankLatch
printf "romLatch=%d\n",   (int)MemESP::romLatch
printf "videoLatch=%d\n", (int)MemESP::videoLatch
printf "romInUse=%d\n",   (int)MemESP::romInUse
printf "pagingLock=%d\n", (int)MemESP::pagingLock
printf "page0ram=%d\n",   (int)MemESP::page0ram
printf "tstates=%u\n",    (unsigned int)CPU::tstates
printf "ram0_type=%d\n",  (int)MemESP::ram[0]._int->mem_type
printf "ram1_type=%d\n",  (int)MemESP::ram[1]._int->mem_type
printf "ram2_type=%d\n",  (int)MemESP::ram[2]._int->mem_type
printf "ram3_type=%d\n",  (int)MemESP::ram[3]._int->mem_type
printf "ram4_type=%d\n",  (int)MemESP::ram[4]._int->mem_type
printf "ram5_type=%d\n",  (int)MemESP::ram[5]._int->mem_type
printf "ram6_type=%d\n",  (int)MemESP::ram[6]._int->mem_type
printf "ram7_type=%d\n",  (int)MemESP::ram[7]._int->mem_type

set logging enabled off
set logging redirect off

echo \nMemory dump files written to /tmp/picospec_mem{0-3}.bin, /tmp/picospec_ram{0-7}.bin and /tmp/picospec_regs.txt\n
