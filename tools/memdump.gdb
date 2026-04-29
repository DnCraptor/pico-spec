# memdump.gdb — dump Z80 address space 0x0000-0xFFFF via MemESP::ramCurrent[]
# Usage (in Debug Console): source tools/memdump.gdb
# Then run task "Pico: Memory Dump" (Ctrl+Alt+D)

dump binary memory /tmp/picospec_mem0.bin MemESP::ramCurrent[0] (MemESP::ramCurrent[0] + 16384)
dump binary memory /tmp/picospec_mem1.bin MemESP::ramCurrent[1] (MemESP::ramCurrent[1] + 16384)
dump binary memory /tmp/picospec_mem2.bin MemESP::ramCurrent[2] (MemESP::ramCurrent[2] + 16384)
dump binary memory /tmp/picospec_mem3.bin MemESP::ramCurrent[3] (MemESP::ramCurrent[3] + 16384)

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

set logging enabled off
set logging redirect off

echo \nMemory dump files written to /tmp/picospec_mem{0-3}.bin and /tmp/picospec_regs.txt\n
