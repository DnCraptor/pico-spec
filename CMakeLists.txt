# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")
cmake_minimum_required(VERSION 3.13)

option(VGA_HDMI "Enable VGA/HDMI driver" ON)
option(SUPPRESS_VGA "Suppresse VGA driver" ON)
option(TFT "Enable TFT display" OFF)
option(ILI9341 "Enable TFT ILI9341 display" OFF)
option(TV "Enable TV composite output" OFF)
option(SOFTTV "Enable TV soft composite output" OFF)
option(MURM "Murmulator 1.0" ON)
option(MURM2 "Murmulator 2.0" OFF)
option(PICO_PC "RP2040-PICO-PC" OFF)
option(PICO_DV "RP2350-PICO-DV" OFF)
option(ZERO "RP2040-PiZero" OFF)
option(ZERO2 "RP2350-PiZero" OFF)
option(ZIFI_NET_CLIENT "Enable FTP/SFTP/SSH client over ZiFi (RP2350 only, uses mbedTLS)" OFF)

# Per-subsystem port-access tracing toggles (debugging). OFF by default — when
# ON, gated `#if XXX_TRACE` Debug::log() calls in the sources become active.
# Add new toggles here following the same option(...) + compile-definition pattern.
option(IDE_PORT_TRACE "Enable PROFI IDE/HDD port access tracing" OFF)
option(FDD_PORT_TRACE "Enable WD1793 (ВГ93) FDD command/port tracing" OFF)
option(RTC_PORT_TRACE "Enable MC146818 Mr Gluk RTC port access tracing" OFF)
option(PROFI_PORT_TRACE "Enable 0x7FFD/0xDFFD paging-port write tracing" OFF)
option(SND_PORT_TRACE "Enable per-port I/O histogram (sound-DAC port hunting)" OFF)
option(PERF_TRACE "Enable per-60-frame CPU/HDMI/FPS performance logging" OFF)
option(MEM_ACCESS_TRACE "Count Z80 accesses per RAM page between load and eviction (accessor-mode feasibility study; slows readbyte/writebyte)" OFF)
option(GS_PERF_TRACE "Enable General Sound per-second perf counters (GS.cpp)" OFF)
option(GS_DEBUG_TRACE "Enable General Sound port-IO trace ring + auto-dumps (~50 KB SRAM)" OFF)
option(ZIFI_TRACE "Enable ZiFi (ESP-01S WiFi NIC) port/UART tracing" OFF)
option(ZIFI_NET_VERBOSE "Per-packet trace of the ZiFi net client (+IPD/chanSend); floods logs" OFF)

# Per-board Debug-Probe UART toggles (logging over UART). Declared here together;
# the board-specific `if(<board>_DBG_UART)` blocks below consume them (they call
# pico_enable_stdio_uart() + remap pins, which needs the target to exist first).
# Each enables stdio over the board's Debug-Probe header — see the block for pins.
set(MURM1_DBG_UART   OFF CACHE BOOL "Enable Debug Probe UART0 on GP0/GP1 (MURM1) — moves KBD to GP16/GP17, disables NESPAD")
set(PICO_PC_DBG_UART OFF CACHE BOOL "Enable Debug Probe UART0 on DBG1 header GP0/GP1 (PICO_PC)")
set(PICO_DV_DBG_UART OFF CACHE BOOL "Enable Debug Probe UART1 on GP20/GP21 (PICO_DV)")
set(ZERO_DBG_UART    OFF CACHE BOOL "Enable Debug Probe UART0 on GP16/GP13 (ZERO)")
set(ZERO2_DBG_UART   OFF CACHE BOOL "Enable Debug Probe UART1 on GP20/GP21 (ZERO2)")

#set(FLASH_SIZE_GREEN ON)
#set(m1p2launcher ON)

# set(TFT ON)
# set(TFT_INV ON)
# set(ILI9341 ON)
# set(ST7789 ON)
# set(SOFTTV ON)
# set(PSRAM OFF)  #For Testing Only

# Display output (default: VGA_HDMI; override with -D flags, e.g. cmake -DSOFTTV=ON)
if(NOT VGA_HDMI AND NOT SOFTTV AND NOT TFT AND NOT ILI9341 AND NOT TV)
    set(VGA_HDMI ON)
endif()
# ALF is always available on RP2350 (4MB) — small 256KB default cart + shared flash
# region — but excluded on flash-constrained 2MB RP2040 builds via the SDK's
# PICO_RP2040 macro directly in the source guards (no separate define needed).

set(CPU_MHZ 378)
set (PORT_VERSION "1.2.28")
string(LENGTH "${PORT_VERSION}" PORT_VERSION_LEN)

# Target board (default: PICO_PC; override with -D flags, e.g. cmake -DMURM2=ON)
if(NOT MURM AND NOT MURM2 AND NOT PICO_PC AND NOT PICO_DV AND NOT ZERO AND NOT ZERO2)
    set(MURM2 ON)
    set(MURM2_P2 ON)
endif()

set(KBDUSB ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pull in Raspberry Pi Pico SDK (must be before project)
IF(ZERO2)
    # For Board RP2350-PiZero
    set(PICO_BOARD_HEADER_DIRS ${CMAKE_SOURCE_DIR}/src/boards)
    set(PICO_BOARD waveshare_rp2350_pizero CACHE STRING "Board type" FORCE)
    set(PICO_PLATFORM rp2350-arm-s)
    set(CPU_MHZ 252)
elseif(ZERO)
    set(PICO_BOARD pico CACHE STRING "Board type" FORCE)
    set(CPU_MHZ 252)
elseif(MURM AND NOT MURM_P2)
    set(PICO_BOARD pico CACHE STRING "Board type" FORCE)
    set(PICO_PLATFORM rp2040)
elseif(MURM2 AND NOT MURM2_P2)
    set(PICO_BOARD pico CACHE STRING "Board type" FORCE)
    set(PICO_PLATFORM rp2040)
else()
    # All remaining RP2350 boards (MURM2_P2, PICO_PC, PICO_DV, MURM1_P2/default)
    # build for the RP2350B (48-GPIO) package so GPIO 30..47 are valid for any
    # peripheral. pico2 forces PICO_RP2350A=1, so we use a local B-variant header
    # (a CMake `set(PICO_RP2350A 0)` is ignored by the SDK — it must be the header).
    set(PICO_BOARD_HEADER_DIRS ${CMAKE_SOURCE_DIR}/src/boards)
    set(PICO_BOARD picospec_rp2350b CACHE STRING "Board type" FORCE)
    set(PICO_PLATFORM rp2350-arm-s)
endif()

message(STATUS "PICO_BOARD: ${PICO_BOARD}")

include(pico_sdk_import.cmake)

if (PICO_BOARD STREQUAL "pico")
    SET(PROJECT_NAME p1-spec)
    set(BOARD_CHIP_TAG p1)
    set(FLASH_SIZE 2048)
    # light .uf2 (reduced number of ROMs)
    set(FLASH_SIZE_GREEN ON)
else ()
    SET(PROJECT_NAME p2-spec)
    set(BOARD_CHIP_TAG p2)
    set(FLASH_SIZE 4096)
endif()

IF(MURM2)
    SET(BUILD_NAME "m2${PROJECT_NAME}")
    set(CONFIG_BOARD_TAG "m2${BOARD_CHIP_TAG}")
    set (BUTTER_PSRAM_GPIO 8)
ELSEIF(PICO_PC)
    SET(BUILD_NAME "PC${PROJECT_NAME}")
    set(CONFIG_BOARD_TAG "PC${BOARD_CHIP_TAG}")
    set (BUTTER_PSRAM_GPIO 8)
ELSEIF(PICO_DV)
    SET(BUILD_NAME "DV${PROJECT_NAME}")
    set(CONFIG_BOARD_TAG "DV${BOARD_CHIP_TAG}")
    set (BUTTER_PSRAM_GPIO 47)
ELSEIF(ZERO)
    SET(BUILD_NAME "z0${PROJECT_NAME}")
    set(CONFIG_BOARD_TAG "z0${BOARD_CHIP_TAG}")
ELSEIF(ZERO2)
    SET(BUILD_NAME "z0${PROJECT_NAME}")
    set(CONFIG_BOARD_TAG "z0${BOARD_CHIP_TAG}")
    set (BUTTER_PSRAM_GPIO 47)
ELSE()
    SET(BUILD_NAME "m1${PROJECT_NAME}")
    set(CONFIG_BOARD_TAG "m1${BOARD_CHIP_TAG}")
    set (BUTTER_PSRAM_GPIO 19)
ENDIF()

# PSRAM kill-switch: only when set(PSRAM OFF) is explicitly uncommented above
# (an undefined/unset PSRAM leaves PSRAM enabled). Forces the butter PSRAM CS to
# GPIO 255 (no valid pin) so the driver treats PSRAM as absent on every board.
if(PSRAM STREQUAL "OFF")
    set(BUTTER_PSRAM_GPIO 255)
endif()

set(OUTPUT_DIR "${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}")

project(${PROJECT_NAME} C CXX ASM)

# Core0 stack lives at the top of SCRATCH_Y (.scratch_y is empty), and SCRATCH_X
# (core1/video stack) sits directly below it. The default 2 KB core0 stack is too
# tight for the OSD menu call chain (file browser → text-edit → file-info/create →
# std::string + format parsers): it overflowed into SCRATCH_X, corrupting core1's
# video stack — manifesting as SIGBUS (MSTKERR) crashes (F1 file info, F9 create
# TRD) and a garbled screen. Grow core0 to 4 KB (fills the otherwise-empty 4 KB
# SCRATCH_Y exactly); pin core1 at 2 KB so SCRATCH_X is unchanged.
#
# PICO_USE_STACK_GUARDS: hw-traced 2026-07-21 — even 4 KB overflowed again in
# usbRoot mode (Tape::LoadTape → FatFs → usb_disk_read → tuh_task pump), and the
# silent descent into SCRATCH_X trashed core1's live IRQ frames: core1 SIGBUS
# storms with garbage PC (values = core0 locals), then double-fault LOCKUP on
# both cores. The guard (MSPLIM on RP2350, MPU canary on RP2040, both cores)
# turns any future overflow into an immediate fault AT the boundary — the
# overflowing core traps with stackOvf=1 instead of corrupting its neighbour.
add_compile_definitions(PICO_STACK_SIZE=0x1000 PICO_CORE1_STACK_SIZE=0x800
                        PICO_USE_STACK_GUARDS=1)

# ZiFi network client (FTP/SFTP/SSH) uses the SDK-vendored mbedTLS as a crypto
# primitive library. Point the SDK's mbedTLS at our trimmed config via an
# ABSOLUTE path BEFORE pico_sdk_init() — the pico_mbedtls CMakeLists reads
# PICO_MBEDTLS_CONFIG_FILE at SDK-init time and propagates it (as
# MBEDTLS_CONFIG_FILE) to both our TUs and the mbedTLS library's own sources.
# Absolute path means it resolves regardless of per-target include dirs. Harmless
# on builds that don't link pico_mbedtls (RP2040 / ZIFI_NET_CLIENT=OFF).
if (ZIFI_NET_CLIENT AND NOT PICO_PLATFORM STREQUAL "rp2040")
    set(PICO_MBEDTLS_CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/src/mbedtls_config_picospec.h")
endif ()

# TinyUSB 0.21.0 vendored at external/tinyusb (subset: LICENSE, src/, hw/bsp/
# rp2040 + family_support — what the SDK's tinyusb glue consumes). Must be set
# BEFORE pico_sdk_init(): the SDK-bundled TinyUSB (0.18 in SDK 2.2.0) paces host
# bulk at ~1 × 64 B packet per 1 ms frame, capping USB MSC/CDC near ~64 KB/s;
# the 0.21 RP2 HCD rework lifts Speed Test to 0.89/0.75 MB/s (hw-confirmed).
set(PICO_TINYUSB_PATH ${CMAKE_CURRENT_LIST_DIR}/external/tinyusb)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(drivers/audio)
add_subdirectory(drivers/picomp3lib)
add_subdirectory(drivers/nespad)
add_subdirectory(drivers/fatfs)
add_subdirectory(drivers/sdcard)
add_subdirectory(drivers/vga-nextgen)
add_subdirectory(drivers/st7789)
add_subdirectory(drivers/hdmi)
add_subdirectory(drivers/tv)
add_subdirectory(drivers/tv-software)
add_subdirectory(drivers/graphics)
add_subdirectory(drivers/psram)

#target_compile_definitions(${PROJECT_NAME} PRIVATE VGA)

# INCLUDE FILES THAT SHOULD BE COMPILED:
file(GLOB_RECURSE SRC
    "src/*.cpp"
    "src/*.c"
    "fabgl/*.cpp"
    "src/roms/*.cpp"
    "src/roms/*.c"
    "src/miniz/*.c"
    "src/GS/*.cpp"
    "src/GS/*.c"
)

message(STATUS "Add source files:")
foreach (SRC_FILE IN LISTS SRC)
    message(STATUS "${SRC_FILE}")
endforeach ()
message(STATUS "")

add_executable(${PROJECT_NAME} ${SRC})

# КРИТИЧНАЯ ОПТИМИЗАЦИЯ: принудительно -O3 для горячего кода независимо от CMAKE_BUILD_TYPE
# Это важно для MinSizeRel (где по умолчанию -Os), чтобы не терять FPS
set_source_files_properties(src/AySound.cpp PROPERTIES
    COMPILE_FLAGS "-O3 -ffast-math -funroll-loops")
set_source_files_properties(src/SAASound.cpp PROPERTIES
    COMPILE_FLAGS "-O3 -ffast-math -funroll-loops")
set_source_files_properties(src/MidiSynth.cpp PROPERTIES
    COMPILE_FLAGS "-O3 -ffast-math -funroll-loops")
set_source_files_properties(src/SoftSynth.cpp PROPERTIES
    COMPILE_FLAGS "-O3 -ffast-math -funroll-loops")
# Wavetable MIDI engine wrapper (external/embeded-midi-synth) — hot audio path.
set_source_files_properties(src/midi_wt.c PROPERTIES
    COMPILE_FLAGS "-O3 -ffast-math -funroll-loops")
set_source_files_properties(src/Video.cpp PROPERTIES
    COMPILE_FLAGS "-O3 -funroll-loops")
# Z80 эмулятор — самый горячий код, критично для производительности
set_source_files_properties(src/Z80_JLS.cpp PROPERTIES
    COMPILE_FLAGS "-O3 -funroll-loops")
set_source_files_properties(src/CPU.cpp PROPERTIES
    COMPILE_FLAGS "-O3")
set_source_files_properties(src/Ports.cpp PROPERTIES
    COMPILE_FLAGS "-O3")
set_source_files_properties(src/wd1793.cpp PROPERTIES
    COMPILE_FLAGS "-O3")
set_source_files_properties(src/ESPectrum.cpp PROPERTIES
    COMPILE_FLAGS "-O3 -funroll-loops")
set_source_files_properties(src/pwm_audio.cpp PROPERTIES
    COMPILE_FLAGS "-O3")

if (PICO_BOARD STREQUAL "pico")
    pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
    target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
    pico_set_boot_stage2(${PROJECT_NAME} slower_boot2)
endif()

# pull in common dependencies
target_link_libraries(${PROJECT_NAME} PRIVATE
    psram
    audio
    graphics
    picomp3lib

    nespad
    sdcard
    fatfs

    pico_runtime
    pico_stdlib
    pico_multicore
    hardware_dma
    hardware_pio
    hardware_i2c
    hardware_interp
    hardware_timer
    hardware_clocks
    hardware_pwm
    hardware_uart
    hardware_flash
    hardware_xip_cache
    hardware_exception
    stdc++

    tinyusb_host tinyusb_board
)

target_include_directories(${PROJECT_NAME} PRIVATE
        src
        src/roms
        external/embeded-midi-synth
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${PROJECT_NAME} pico_cyw43_arch_none)
endif()

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# add url via pico_set_program_url

if (m1p2launcher)
    pico_set_linker_script(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/memmap.ld")
elseif (NOT PICO_PLATFORM STREQUAL "rp2040")
    # Custom linker script reserves a fixed 128K region (RAM_PAGES01234567) for
    # the 8 Z80 RAM pages backing pages 0-7 (m16c_pages: pages0123/pages46/57).
    # All other code/data and the heap (which holds the framebuffer) live in a
    # separate RAM region. The two regions sit in different SRAM banks with
    # independent AHB ports — HDMI DMA reading the framebuffer cannot stall
    # CPU access to Z80 RAM and vice versa.
    pico_set_linker_script(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/rp2350-memmap.ld")
endif ()

target_link_options(${PROJECT_NAME} PRIVATE -Xlinker --print-memory-usage --data-sections)
target_compile_definitions(${PROJECT_NAME} PRIVATE FLASH_SIZE=${FLASH_SIZE})
target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_BOARD_TAG="${CONFIG_BOARD_TAG}")

# IDE/HDD port tracing toggle (see Ports.cpp / IDE.cpp `#if IDE_PORT_TRACE`).
if (IDE_PORT_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE IDE_PORT_TRACE=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE IDE_PORT_TRACE=0)
endif ()

# WD1793 FDD command tracing toggle (see wd1793.cpp `#if FDD_PORT_TRACE`).
if (FDD_PORT_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE FDD_PORT_TRACE=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE FDD_PORT_TRACE=0)
endif ()

# RTC (MC146818 Mr Gluk TimeKeeper) port tracing — logs every IN/OUT with low
# byte 0xF7 (and the EFF7 enable), to discover Gluk's real RTC port/register use.
if (RTC_PORT_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE RTC_PORT_TRACE=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE RTC_PORT_TRACE=0)
endif ()

# Per-60-frame CPU/HDMI/FPS performance logging (see Video.cpp `#if PERF_TRACE`).
if (PERF_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PERF_TRACE=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE PERF_TRACE=0)
endif ()

# Per-page Z80 access counting between page load and eviction (see MemESP.h/.cpp
# `#if MEM_ACCESS_TRACE`, [ACC] log in Video.cpp). Feasibility study for the
# accessor-mode bank window idea: if evicted trampoline pages show only a few
# hundred accesses, serving them per-byte over SPI beats the 16KB page load.
if (MEM_ACCESS_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE MEM_ACCESS_TRACE=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE MEM_ACCESS_TRACE=0)
endif ()

# General Sound perf counters toggle (see GS.cpp `GS_PERF()` / `#if GS_PERF_TRACE`).
if (GS_PERF_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE GS_PERF_TRACE=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE GS_PERF_TRACE=0)
endif ()

# General Sound port-IO trace ring + auto-dump triggers (see GS.cpp
# `#ifdef GS_DEBUG_TRACE`). Costs ~50 KB SRAM — diagnostic builds only;
# the code tests presence with #ifdef, so define it only when ON.
if (GS_DEBUG_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE GS_DEBUG_TRACE=1)
endif ()

# 0x7FFD/0xDFFD paging-port write tracing (see Ports.cpp `#if PROFI_PORT_TRACE`).
if (PROFI_PORT_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PROFI_PORT_TRACE=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE PROFI_PORT_TRACE=0)
endif ()

# ZiFi port/UART tracing (see ZiFi.cpp / ZiFiAT.cpp `#if ZIFI_TRACE`).
if (ZIFI_TRACE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ZIFI_TRACE=1)
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE ZIFI_TRACE=0)
endif ()

if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
target_compile_options(${PROJECT_NAME} PUBLIC
        -Wno-error=suggest-attribute=format
        -Wno-error=cast-qual
        -Wno-error=unused-parameter
        -Wno-error=conversion
        -Wno-error=format=
        -Wno-error=sign-compare
        -Wno-error=missing-field-initializers
        -Wno-error=switch
        -Wno-error=implicit-fallthrough=
        -Wno-error=stringop-truncation
        -Wno-error=restrict
        -w
)
endif ()

IF(FLASH_SIZE_GREEN)
target_compile_definitions(${PROJECT_NAME} PRIVATE
#        NO_SEPARATE_48K_CUSTOM=1
#        NO_SPAIN_ROM_48k=1
#        NO_SPAIN_ROM_128k=1
)
ENDIF()

# ALF is now a standard runtime machine (256KB default cart "Elf-1" in flash; bigger
# cartridges load into the shared flash region from SD/Web). CARTRIDGE_AS_CUSTOM is
# gone — custom ROMs use their own dedicated slots. ALF is excluded on 2MB RP2040
# builds via the PICO_RP2040 macro in the source guards.
# ALF no longer carries a 1.6MB cartridge in the image, so the GM.DLS gm_bank
# partition stays available alongside ALF on RP2350 (they share the region at
# runtime — one occupant at a time; see SharedFlash/RomManager).

IF(KBDUSB)
    add_subdirectory(drivers/ps2kbd)
    target_link_libraries(${PROJECT_NAME} PRIVATE ps2kbd)
    target_compile_definitions(${PROJECT_NAME} PRIVATE KBDUSB)
ELSE()
    add_subdirectory(drivers/ps2)
    target_link_libraries(${PROJECT_NAME} PRIVATE ps2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE KBDPS2)
ENDIF()

if (BUTTER_PSRAM_GPIO)
    target_compile_definitions(${PROJECT_NAME} PRIVATE BUTTER_PSRAM_GPIO=${BUTTER_PSRAM_GPIO})
endif()
# General Sound — needs butter PSRAM (XIP @ 0x11000000) for 2 MB of GS RAM plus
# a second Z80 instance. butter PSRAM is RP2350-only; on RP2040 BUTTER_PSRAM_GPIO
# maps to plain SPI PSRAM which has different semantics. Restrict accordingly.
if (BUTTER_PSRAM_GPIO AND NOT PICO_PLATFORM STREQUAL "rp2040")
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_GS=1)
endif()

# ZiFi (ESP-01S WiFi NIC emulation) — RP2350 only.
# TX/RX pins are chosen at RUNTIME (Network → GPIO picker, stored in NVS) from a
# per-board candidate list defined in src/BoardPins.cpp. No compile-time pin
# define is needed anymore; the old -DZIFI_TX_PIN/-DZIFI_RX_PIN are obsolete.

# ZiFi network client (FTP/SFTP/SSH over the ESP-01S TCP sockets). RP2350 only —
# SSH crypto runs on the Pico via mbedTLS primitives (NOT wolfSSH, NOT mbedTLS's
# TLS stack; see src/mbedtls_config_picospec.h). New sources (ZiFiSock/Ftp/Ssh/
# Sftp.cpp) auto-compile via the GLOB; each file body is wrapped in
# `#if !PICO_RP2040 && ZIFI_NET_CLIENT` so RP2040 builds get zero footprint.
if (ZIFI_NET_CLIENT AND NOT PICO_PLATFORM STREQUAL "rp2040")
    target_compile_definitions(${PROJECT_NAME} PRIVATE ZIFI_NET_CLIENT=1)
    if (ZIFI_NET_VERBOSE)
        target_compile_definitions(${PROJECT_NAME} PRIVATE ZIFI_NET_VERBOSE=1)
    endif ()
    # MBEDTLS_CONFIG_FILE is supplied by the SDK from PICO_MBEDTLS_CONFIG_FILE
    # (set before pico_sdk_init above) — do not redefine it here, that collides
    # with pico_mbedtls_headers' INTERFACE define.
    target_link_libraries(${PROJECT_NAME} PRIVATE
        pico_mbedtls          # crypto primitives from pico-sdk's vendored mbedTLS
        pico_rand)            # hardware RNG for SSH ephemeral keys / padding
    message(STATUS "ZiFi network client (FTP/SFTP/SSH) enabled with mbedTLS crypto")
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE ZIFI_NET_CLIENT=0)
endif ()

# redcode/Z80 build configuration for GS (LGPLv3 library vendored in src/GS/)
set_source_files_properties(src/GS/Z80_redcode.c PROPERTIES
    COMPILE_FLAGS "-O3 -funroll-loops -DZ80_STATIC -DGS_Z80_INSN_IN_SRAM -DZ80_EXTERNAL_HEADER=\\\"Z80_compat.h\\\"")
set_source_files_properties(src/GS/GS.cpp PROPERTIES
    COMPILE_FLAGS "-O3 -funroll-loops -DZ80_STATIC -DZ80_EXTERNAL_HEADER=\\\"Z80_compat.h\\\"")

IF(MURM2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}" PORT_VERSION_LEN=${PORT_VERSION_LEN}
        MURM2=1
        CPU_MHZ=${CPU_MHZ}

        KBD_CLOCK_PIN=2
        KBD_DATA_PIN=3

        LOAD_WAV_PIO=22

        CLK_AY_PIN1=21 # not in use
        LATCH_595_PIN=9
        CLK_595_PIN=10
        DATA_595_PIN=11
        CLK_AY_PIN2=29

        I2S_DATA_PIO=9 #
        I2S_BCK_PIO=10 #
        I2S_LCK_PIO=11

        BEEPER_PIN=9 #
        PWM_PIN0=10 #
        PWM_PIN1=11
        MIDI_TX_PIN=22

        # SDCARD
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=6
        SDCARD_PIN_SPI0_MOSI=7
        SDCARD_PIN_SPI0_MISO=4

        #PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=255
        PSRAM_ASYNC=255

        PSRAM_PIN_CS=255
        PSRAM_PIN_SCK=255
        PSRAM_PIN_MOSI=255
        PSRAM_PIN_MISO=255

        # NES Gamepad
        USE_NESPAD
        NES_GPIO_CLK=20
        NES_GPIO_LAT=21
        NES_GPIO_DATA=26

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=12

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=12


        # TFT
        TFT_CS_PIN=12
        TFT_RST_PIN=14
        TFT_LED_PIN=15
        TFT_DC_PIN=16
        TFT_DATA_PIN=18
        TFT_CLK_PIN=19
        TFT_INV=${TFT_INV}

        DEFAULT_THROTTLING=0
        PICO_DEFAULT_LED_PIN=25
    )
ELSEIF(PICO_PC)
    # Debug-Probe UART toggle PICO_PC_DBG_UART declared at top. Enables UART0 on the
    # on-board DBG1 header (GP0=TX, GP1=RX) of Olimex RP2040-PICO-PC.
    # DBG1 shares pins with KBD_CLOCK/KBD_DATA → keyboard is auto-remapped to FREE
    # pins GP10/GP11 when DBG_UART is enabled.
    if(PICO_PC_DBG_UART)
        pico_enable_stdio_uart(${PROJECT_NAME} 1)
        set(PICO_PC_KBD_CLOCK 10) # GP0 taken by UART TX → move KBD to FREE pin
        set(PICO_PC_KBD_DATA 11)  # GP1 taken by UART RX → move KBD to FREE pin
    else()
        set(PICO_PC_KBD_CLOCK 0)
        set(PICO_PC_KBD_DATA 1)
    endif()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        $<$<BOOL:${PICO_PC_DBG_UART}>:PICO_DEFAULT_UART=0>
        $<$<BOOL:${PICO_PC_DBG_UART}>:PICO_DEFAULT_UART_TX_PIN=0>
        $<$<BOOL:${PICO_PC_DBG_UART}>:PICO_DEFAULT_UART_RX_PIN=1>
        $<$<BOOL:${PICO_PC_DBG_UART}>:DBG_UART_ENABLED=1>

        PORT_VERSION="${PORT_VERSION}" PORT_VERSION_LEN=${PORT_VERSION_LEN}
        CPU_MHZ=${CPU_MHZ}

        PICO_PC=1

        # GP-5  UXT1-10
        # GP-8  UXT1-6
        # GP-9  UXT1-5
        # GP-21 UXT1-4
        # GP-20 UXT1-3

        # QWST1 3/4 - GP2/3

        # DBG1 1/2  (GP0=UART0_TX, GP1=UART0_RX) — used by PICO_PC_DBG_UART
        KBD_CLOCK_PIN=${PICO_PC_KBD_CLOCK}
        KBD_DATA_PIN=${PICO_PC_KBD_DATA}

        LOAD_WAV_PIO=5 # UXT1-10

        CLK_AY_PIN1=21 # not in use
        LATCH_595_PIN=26
        CLK_595_PIN=27
        DATA_595_PIN=28
        CLK_AY_PIN2=29

        I2S_DATA_PIO=26 # not implemented
        I2S_BCK_PIO=26  # I2S_BCK_PIO == I2S_LCK_PIO == I2S_DATA_PIO - unsupported mark
        I2S_LCK_PIO=26

        BEEPER_PIN=26 # not implemented
        PWM_PIN0=27 # R
        PWM_PIN1=28 # L
        MIDI_TX_PIN=26

        # SDCARD
        SDCARD_PIN_SPI0_MISO=4
        SDCARD_PIN_SPI0_SCK=6
        SDCARD_PIN_SPI0_MOSI=7
        SDCARD_PIN_SPI0_CS=22

        #PSRAM             # not implemented
        PSRAM_SPINLOCK=255 # not implemented
        PSRAM_ASYNC=255    # not implemented

        PSRAM_PIN_CS=255   # not implemented
        PSRAM_PIN_SCK=255  # not implemented
        PSRAM_PIN_MOSI=255 # not implemented
        PSRAM_PIN_MISO=255 # not implemented

        # NES Gamepad
        USE_NESPAD
        NES_GPIO_CLK=5    # UXT1-10
        NES_GPIO_LAT=9    # UXT1-5
        NES_GPIO_DATA=20  # UXT1-3
        NES_GPIO_DATA2=21 # UXT1-4

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=12

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=12

        DEFAULT_THROTTLING=0
        PICO_DEFAULT_LED_PIN=25
    )
ELSEIF(PICO_DV)
    # Debug-Probe UART toggle PICO_DV_DBG_UART declared at top. Enables UART1 on GP20/GP21.
    # Note: GP20 is also LOAD_WAV_PIO default — auto-remapped to GP3 when UART enabled.
    if(PICO_DV_DBG_UART)
        pico_enable_stdio_uart(${PROJECT_NAME} 1)
        set(PICO_DV_LOAD_WAV_PIO 255)  # GP20 taken by UART → use FREE pin
        set(MIDI_TX_PIN 255) # GP21 taken by UART → use FREE pin
    else()
        set(PICO_DV_LOAD_WAV_PIO 20)
        set(MIDI_TX_PIN 21)
    endif()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        $<$<BOOL:${PICO_DV_DBG_UART}>:PICO_DEFAULT_UART=1>
        $<$<BOOL:${PICO_DV_DBG_UART}>:PICO_DEFAULT_UART_TX_PIN=20>
        $<$<BOOL:${PICO_DV_DBG_UART}>:PICO_DEFAULT_UART_RX_PIN=21>
        # Console UART intentionally enabled. The board header (pico2.h) ALWAYS
        # defines PICO_DEFAULT_UART=0 on GP0/GP1 — which is exactly the ZiFi UART
        # on PICO_DV — so Debug::log/stdio must gate on this explicit flag, NOT on
        # PICO_DEFAULT_UART, or logs corrupt the ESP AT stream and FTP won't start.
        $<$<BOOL:${PICO_DV_DBG_UART}>:DBG_UART_ENABLED=1>

        PORT_VERSION="${PORT_VERSION}" PORT_VERSION_LEN=${PORT_VERSION_LEN}
        CPU_MHZ=${CPU_MHZ}

        PICO_DV=1

        # PS/2 keyboard (moved from GP0/1 → GP14/15; GP0/1 freed for ZiFi UART0)
        KBD_CLOCK_PIN=14
        KBD_DATA_PIN=15

        LOAD_WAV_PIO=${PICO_DV_LOAD_WAV_PIO}

        CLK_AY_PIN1=21 # not in use
        LATCH_595_PIN=26
        CLK_595_PIN=27
        DATA_595_PIN=28
        CLK_AY_PIN2=29

        I2S_DATA_PIO=26 #
        I2S_BCK_PIO=27 #
        I2S_LCK_PIO=28

        PWM_PIN0=26 # R
        PWM_PIN1=27 # L
        BEEPER_PIN=28 # not implemented
        MIDI_TX_PIN=${MIDI_TX_PIN}

        # SDCARD
        SDCARD_SPI_BUS=spi1
        SDCARD_PIN_SPI0_MISO=19
        SDCARD_PIN_SPI0_SCK=5
        SDCARD_PIN_SPI0_MOSI=18
        SDCARD_PIN_SPI0_CS=22

        SDCARD_PIO=pio1
        SDCARD_PIO_SM=0

        #PSRAM             # not implemented
        PSRAM_SPINLOCK=255 # not implemented
        PSRAM_ASYNC=255    # not implemented

        PSRAM_PIN_CS=255   # not implemented
        PSRAM_PIN_SCK=255  # not implemented
        PSRAM_PIN_MOSI=255 # not implemented
        PSRAM_PIN_MISO=255 # not implemented

        # NES Gamepad
        NES_GPIO_CLK=8    # UXT1-6
        NES_GPIO_LAT=9    # UXT1-5
        NES_GPIO_DATA=20  # UXT1-3
        NES_GPIO_DATA2=21 # UXT1-4

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=6

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=6

        DEFAULT_THROTTLING=0
        PICO_DEFAULT_LED_PIN=25
)
ELSEIF(ZERO)
    # Debug-Probe UART toggle ZERO_DBG_UART declared at top. Enables UART0 on GP16(TX)/GP13(RX).
    # Both pins are FREE on ZERO; UART0 used because UART1 has no free pins on this board.
    if(ZERO_DBG_UART)
        pico_enable_stdio_uart(${PROJECT_NAME} 1)
    endif()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        $<$<BOOL:${ZERO_DBG_UART}>:PICO_DEFAULT_UART=0>
        $<$<BOOL:${ZERO_DBG_UART}>:PICO_DEFAULT_UART_TX_PIN=16>
        $<$<BOOL:${ZERO_DBG_UART}>:PICO_DEFAULT_UART_RX_PIN=13>
        $<$<BOOL:${ZERO_DBG_UART}>:DBG_UART_ENABLED=1>

        PORT_VERSION="${PORT_VERSION}" PORT_VERSION_LEN=${PORT_VERSION_LEN}
        CPU_MHZ=${CPU_MHZ}
        ZERO=1

        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        LOAD_WAV_PIO=17

        CLK_AY_PIN1=8 # not in use
        LATCH_595_PIN=10
        CLK_595_PIN=11
        DATA_595_PIN=12
        CLK_AY_PIN2=7

        I2S_DATA_PIO=10 #
        I2S_BCK_PIO=11 #
        I2S_LCK_PIO=12

        PWM_PIN0=10
        PWM_PIN1=11
        BEEPER_PIN=12

        # SDCARD
        SDCARD_PIN_SPI0_SCK=18
        SDCARD_PIN_SPI0_MOSI=19
        SDCARD_PIN_SPI0_MISO=20
        SDCARD_PIN_SPI0_CS=21

        #PSRAM             # not implemented
        PSRAM_SPINLOCK=255 # not implemented
        PSRAM_ASYNC=255    # not implemented

        PSRAM_PIN_CS=255   # not implemented
        PSRAM_PIN_SCK=255  # not implemented
        PSRAM_PIN_MOSI=255 # not implemented
        PSRAM_PIN_MISO=255 # not implemented

        # NES Gamepad
        USE_NESPAD
        NES_GPIO_CLK=7
        NES_GPIO_LAT=8
        NES_GPIO_DATA=9
        NES_GPIO_DATA2=9

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=22

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=22

        DEFAULT_THROTTLING=0
        PICO_RP2040=1
        PICO_DEFAULT_LED_PIN=25
    )
ELSEIF(ZERO2)
    # Debug-Probe UART toggle ZERO2_DBG_UART declared at top (UART1 on GP20/GP21).
    if(ZERO2_DBG_UART)
        pico_enable_stdio_uart(${PROJECT_NAME} 1)
    endif()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        $<$<BOOL:${ZERO2_DBG_UART}>:PICO_DEFAULT_UART=1>
        $<$<BOOL:${ZERO2_DBG_UART}>:PICO_DEFAULT_UART_TX_PIN=20>
        $<$<BOOL:${ZERO2_DBG_UART}>:PICO_DEFAULT_UART_RX_PIN=21>
        $<$<BOOL:${ZERO2_DBG_UART}>:DBG_UART_ENABLED=1>

        PORT_VERSION="${PORT_VERSION}" PORT_VERSION_LEN=${PORT_VERSION_LEN}
        CPU_MHZ=${CPU_MHZ}
        ZERO2=1

        # PS2 keyboard (moved from GPIO 2/3 for PCM5122 I2C)
    #    KBD_CLOCK_PIN=14
    #    KBD_DATA_PIN=15
        KBD_CLOCK_PIN=2
        KBD_DATA_PIN=3

        LOAD_WAV_PIO=17

        # PCM5122 Audio Board (I2S)
   #     PCM5122_I2S_DATA=21
   #     PCM5122_I2S_BCK=18
   #     PCM5122_I2S_LCK=19
        # PCM5122 Audio Board (I2C control)
   #     PCM5122_I2C_SDA=2
   #     PCM5122_I2C_SCL=3

        CLK_AY_PIN1=8 # not in use
        LATCH_595_PIN=10
        CLK_595_PIN=11
        DATA_595_PIN=12
        CLK_AY_PIN2=7

        I2S_DATA_PIO=10 # DIN
        I2S_BCK_PIO=11 # BCK
        I2S_LCK_PIO=12 # LRCK

        PWM_PIN0=10
        PWM_PIN1=11
        BEEPER_PIN=12
        MIDI_TX_PIN=22

        # SDCARD
        SDCARD_SPI_BUS=spi1
        SDCARD_PIN_SPI0_SCK=30
        SDCARD_PIN_SPI0_MOSI=31
        SDCARD_PIN_SPI0_MISO=40
        SDCARD_PIN_SPI0_CS=43

        #PSRAM             # not implemented
        PSRAM_SPINLOCK=255 # not implemented
        PSRAM_ASYNC=255    # not implemented

        PSRAM_PIN_CS=255   # not implemented
        PSRAM_PIN_SCK=255  # not implemented
        PSRAM_PIN_MOSI=255 # not implemented
        PSRAM_PIN_MISO=255 # not implemented

        # NES Gamepad
        USE_NESPAD
        NES_GPIO_CLK=4
        NES_GPIO_LAT=5
        NES_GPIO_DATA=7
        NES_GPIO_DATA2=8

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=32

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=32

        PICO_DEFAULT_LED_PIN=255
        DEFAULT_THROTTLING=0
    )
ELSE()
    # Debug-Probe UART toggle MURM1_DBG_UART declared at top (UART0 on GP0/GP1;
    # moves KBD to GP16/GP17, disables NESPAD).
    if(MURM1_DBG_UART)
        pico_enable_stdio_uart(${PROJECT_NAME} 1)
        set(MURM1_KBD_CLOCK_PIN 16)
        set(MURM1_KBD_DATA_PIN  17)
    else()
        set(MURM1_KBD_CLOCK_PIN 0)
        set(MURM1_KBD_DATA_PIN  1)
    endif()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        $<$<BOOL:${MURM1_DBG_UART}>:PICO_DEFAULT_UART=0>
        $<$<BOOL:${MURM1_DBG_UART}>:PICO_DEFAULT_UART_TX_PIN=0>
        $<$<BOOL:${MURM1_DBG_UART}>:PICO_DEFAULT_UART_RX_PIN=1>
        $<$<BOOL:${MURM1_DBG_UART}>:DBG_UART_ENABLED=1>

        PORT_VERSION="${PORT_VERSION}" PORT_VERSION_LEN=${PORT_VERSION_LEN}
        CPU_MHZ=${CPU_MHZ}

        KBD_CLOCK_PIN=${MURM1_KBD_CLOCK_PIN}
        KBD_DATA_PIN=${MURM1_KBD_DATA_PIN}

        LOAD_WAV_PIO=22

        I2S_DATA_PIO=26 #
        I2S_BCK_PIO=27 #
        I2S_LCK_PIO=28

        CLK_AY_PIN1=21 # not in use
        LATCH_595_PIN=26
        CLK_595_PIN=27
        DATA_595_PIN=28
        CLK_AY_PIN2=29

        PWM_PIN0=26 #
        PWM_PIN1=27 #
        BEEPER_PIN=28
        MIDI_TX_PIN=22

        # SDCARD
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=2
        SDCARD_PIN_SPI0_MOSI=3
        SDCARD_PIN_SPI0_MISO=4

        PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        # SPI clock ceiling. At 126 MHz (default) sys_clk=378 gives clkdiv=1.5 —
        # a fractional divider that makes SCK jitter and corrupts this board's
        # APS6404 8 MB chip (~930 KB/1 MB bad at boot, size probe flaps 8M↔1M).
        # 94 MHz targets clkdiv=2.0 at 378 MHz (round(378/188)=2 → SCK=94.5 MHz,
        # still an integer divider with a clean waveform — same reason 63 MHz /
        # clkdiv=3.0 was safe, one step faster: +50% bandwidth). init_psram()
        # runs an at-speed write/verify memtest and drops the target back to
        # PSRAM_FALLBACK_SCK_MHZ (63) if this chip fails at 94.5 MHz;
        # psram_update_clkdiv() keeps the chosen target after CPU overclock too.
        PSRAM_MAX_SCK_MHZ=63

        PSRAM_PIN_CS=18
        PSRAM_PIN_SCK=19
        PSRAM_PIN_MOSI=20
        PSRAM_PIN_MISO=21

        # NES Gamepad — disabled when MURM1_DBG_UART=ON (KBD moves to GP16/GP17, conflicts with NES_GPIO_DATA=16)
        $<$<NOT:$<BOOL:${MURM1_DBG_UART}>>:USE_NESPAD>
        NES_GPIO_CLK=14
        NES_GPIO_DATA=16
        NES_GPIO_LAT=15

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=6

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=6

        # TFT
        TFT_CS_PIN=6
        TFT_RST_PIN=8
        TFT_LED_PIN=9
        TFT_DC_PIN=10
        TFT_DATA_PIN=12
        TFT_CLK_PIN=13
        TFT_INV=${TFT_INV}

        DEFAULT_THROTTLING=0
        PICO_DEFAULT_LED_PIN=25
    )
ENDIF()

IF(TFT)
    target_link_libraries(${PROJECT_NAME} PRIVATE st7789)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TFT)
    IF(ILI9341)
        SET(BUILD_NAME "${BUILD_NAME}-ILI9341")
        target_compile_definitions(${PROJECT_NAME} PRIVATE ILI9341)
    ELSE()
        IF(TFT_INV)
            SET(BUILD_NAME "${BUILD_NAME}-ST7789V")
        ELSE()
            SET(BUILD_NAME "${BUILD_NAME}-ST7789")
        ENDIF()
    ENDIF()
ELSEIF(TV)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TV)
    target_link_libraries(${PROJECT_NAME} PRIVATE tv)
    SET(BUILD_NAME "${BUILD_NAME}-TV")
ELSEIF(SOFTTV)
	target_compile_definitions(${PROJECT_NAME} PRIVATE SOFTTV)
	target_link_libraries(${PROJECT_NAME} PRIVATE tv-software)
	SET(BUILD_NAME "${BUILD_NAME}-TV-SOFT")
ELSE()
    target_link_libraries(${PROJECT_NAME} PRIVATE hdmi)
    target_link_libraries(${PROJECT_NAME} PRIVATE vga-nextgen)
    target_compile_definitions(${PROJECT_NAME} PRIVATE VGA_HDMI)
    if(SUPPRESS_VGA)
        target_compile_definitions(${PROJECT_NAME} PRIVATE SUPPRESS_VGA)
        SET(BUILD_NAME "${BUILD_NAME}-HDMI")
    else()
        SET(BUILD_NAME "${BUILD_NAME}-VGA-HDMI")
    endif()
ENDIF()

SET(BUILD_NAME "${BUILD_NAME}-${PORT_VERSION}")

# ALF is now a standard runtime machine on RP2350 (not a separate build), so no
# "-ALF" suffix. The 2MB RP2040 builds exclude ALF (no room).
IF (FLASH_SIZE_GREEN)
    SET(BUILD_NAME "${BUILD_NAME}")
ENDIF()

IF(m1p2launcher)
    IF(MURM2)
        SET(BUILD_NAME "${BUILD_NAME}.m2p2")
    ELSEIF(PICO_PC)
        SET(BUILD_NAME "${BUILD_NAME}.PCp2")
    ELSEIF(PICO_DV)
        SET(BUILD_NAME "${BUILD_NAME}.DVp2")
    ELSEIF(ZERO2)
        SET(BUILD_NAME "${BUILD_NAME}.z0p2")
    ELSE()
        SET(BUILD_NAME "${BUILD_NAME}.m1p2")
    ENDIF()
ENDIF()

message(STATUS "BUILD_NAME: ${BUILD_NAME}")

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${BUILD_NAME}")

execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    PICO_BUILD_NAME="${BUILD_NAME}"
    PICO_GIT_BRANCH="${GIT_BRANCH}"
    PICO_GIT_COMMIT="${GIT_COMMIT_HASH}"
)

if (m1p2launcher)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rename
        "${OUTPUT_DIR}/${BUILD_NAME}.uf2"
        "${OUTPUT_DIR}/${BUILD_NAME}"
    )
endif ()
