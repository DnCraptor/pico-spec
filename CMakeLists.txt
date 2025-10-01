# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

option(HWAY "Enable Hardware AY-3-8910 sound" OFF)
option(VGA_HDMI "Enable VGA/HDMI driver" OFF)
option(TFT "Enable TFT display" OFF)
option(ILI9341 "Enable TFT ILI9341 display" OFF)
option(TV "Enable TV composite output" OFF)
option(SOFTTV "Enable TV soft composite output" OFF)
option(MURM2 "Murmulator 2.0" OFF)
option(PICO_PC "RP2040-PICO-PC" OFF)
option(PICO_DV "RP2350-PICO-DV" OFF)
option(ZERO "RP2040-PiZero" OFF)
option(ZERO2 "RP2350-PiZero" OFF)

#set(m1p2 ON)
#set(HWAY ON)

set(VGA_HDMI ON)
#set(SOFTTV ON)
#set(TFT ON)
#set(TFT_INV 0)
#set(TFT_INV 1)
#set(ILI9341 ON)

#set(CPU_MHZ 327.3)
#set(CPU_MHZ 336)
#set(CPU_MHZ 315)
set(CPU_MHZ 378)
#set(CPU_MHZ 460)
#set(CPU_MHZ 252)
#set(CPU_MHZ 133)
set (PORT_VERSION "1.2.G")

#set(ZERO ON)
set(ZERO2 ON)
#set (PICO_PC ON)
#set (PICO_DV ON)
#set (MURM ON)
#set (MURM2 ON)

set(KBDUSB ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pull in Raspberry Pi Pico SDK (must be before project)
IF(ZERO2)
    # For Board RP2350-PiZero
    set(PICO_BOARD_HEADER_DIRS ${CMAKE_SOURCE_DIR}/src/boards)
    set(PICO_BOARD waveshare_rp2350_pizero CACHE STRING "Board type")
    set(PICO_PLATFORM rp2350-arm-s)
else()
    set(PICO_BOARD pico2 CACHE STRING "Board type")
endif()

include(pico_sdk_import.cmake)

if (PICO_BOARD MATCHES "pico")
    SET(PROJECT_NAME p1-spec)
    set(FLASH_SIZE 2048)
else ()
    SET(PROJECT_NAME p2-spec)
    set(FLASH_SIZE 4096)
endif()

# light .uf2 (reduced number of ROMs)
set(FLASH_SIZE_GREEN ON)

IF(MURM2)
    SET(BUILD_NAME "m2${PROJECT_NAME}")
    set (BUTTER_PSRAM_GPIO 8)
ELSEIF(PICO_PC)
    SET(BUILD_NAME "PC${PROJECT_NAME}")
    set (BUTTER_PSRAM_GPIO 8)
ELSEIF(PICO_DV)
    SET(BUILD_NAME "DV${PROJECT_NAME}")
    set (BUTTER_PSRAM_GPIO 47)
ELSEIF(ZERO)
    SET(BUILD_NAME "z0${PROJECT_NAME}")
ELSEIF(ZERO2)
    SET(BUILD_NAME "z0${PROJECT_NAME}")
    set (BUTTER_PSRAM_GPIO 47)
ELSE()
    SET(BUILD_NAME "m1${PROJECT_NAME}")
    set (BUTTER_PSRAM_GPIO 19)
ENDIF()

set(OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}")

project(${PROJECT_NAME} C CXX ASM)

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
)

message(STATUS "Add source files:")
foreach (SRC_FILE IN LISTS SRC)
    message(STATUS "${SRC_FILE}")
endforeach ()
message(STATUS "")

add_executable(${PROJECT_NAME} ${SRC})

if (PICO_BOARD MATCHES "pico")
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
    hardware_flash
    hardware_exception
    stdc++

    tinyusb_host tinyusb_board
)

target_include_directories(${PROJECT_NAME} PRIVATE
        src
        src/roms
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${PROJECT_NAME} pico_cyw43_arch_none)
endif()

##pico_enable_stdio_uart(${PROJECT_NAME} 0)
##pico_enable_stdio_usb(${PROJECT_NAME} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# add url via pico_set_program_url

if (m1p2)
    pico_set_linker_script(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/memmap.ld")
endif ()

target_link_options(${PROJECT_NAME} PRIVATE -Xlinker --print-memory-usage --data-sections)
target_compile_definitions(${PROJECT_NAME} PRIVATE FLASH_SIZE=${FLASH_SIZE})

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
        NO_SPAIN_ROM_48k=1
        NO_SPAIN_ROM_128k=1
        CARTRIDGE_AS_CUSTOM=1
        NO_SEPARATE_48K_CUSTOM=1
        NO_ALF=1
)
ENDIF()

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

IF(MURM2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        MURM2=1
        CPU_MHZ=${CPU_MHZ}

        KBD_CLOCK_PIN=2
        KBD_DATA_PIN=3

        LOAD_WAV_PIO=22

        I2S_DATA_PIO=9 #
        I2S_BCK_PIO=10 #
        I2S_LCK_PIO=11

        BEEPER_PIN=9 #
        PWM_PIN0=10 #
        PWM_PIN1=11
        SOUND_FREQUENCY=30600

        # SDCARD
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=6
        SDCARD_PIN_SPI0_MOSI=7
        SDCARD_PIN_SPI0_MISO=4

        #PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=18
        PSRAM_PIN_SCK=19
        PSRAM_PIN_MOSI=20
        PSRAM_PIN_MISO=21

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
    )
ELSEIF(PICO_PC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}

        PICO_PC=1

        # GP-5  UXT1-10
        # GP-8  UXT1-6
        # GP-9  UXT1-5
        # GP-21 UXT1-4
        # GP-20 UXT1-3

        # QWST1 3/4 - GP2/3

        # DBG1 1/2
        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        LOAD_WAV_PIO=5 # UXT1-10

        I2S_DATA_PIO=26 # not implemented
        I2S_BCK_PIO=26  # I2S_BCK_PIO == I2S_LCK_PIO == I2S_DATA_PIO - unsupported mark
        I2S_LCK_PIO=26

        BEEPER_PIN=26 # not implemented
        PWM_PIN0=27 # R
        PWM_PIN1=28 # L
        SOUND_FREQUENCY=30600

        # SDCARD
        SDCARD_PIN_SPI0_MISO=4
        SDCARD_PIN_SPI0_SCK=6
        SDCARD_PIN_SPI0_MOSI=7
        SDCARD_PIN_SPI0_CS=22

        #PSRAM           # not implemented
        PSRAM_SPINLOCK=1 # not implemented
        PSRAM_ASYNC=1    # not implemented

        PSRAM_PIN_CS=3   # not implemented
        PSRAM_PIN_SCK=3  # not implemented
        PSRAM_PIN_MOSI=3 # not implemented
        PSRAM_PIN_MISO=3 # not implemented

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
    )
ELSEIF(PICO_DV)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}

        PICO_DV=1

        # GP-5  UXT1-10
        # GP-8  UXT1-6
        # GP-9  UXT1-5
        # GP-21 UXT1-4
        # GP-20 UXT1-3

        # QWST1 3/4 - GP2/3

        # DBG1 1/2
        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        LOAD_WAV_PIO=5 # UXT1-10

        I2S_DATA_PIO=26 #
        I2S_BCK_PIO=27 #
        I2S_LCK_PIO=28

        PWM_PIN0=26 # R
        PWM_PIN1=27 # L
        BEEPER_PIN=28 # not implemented
        SOUND_FREQUENCY=30600

        # SDCARD
        SDCARD_SPI_BUS=spi1
        SDCARD_PIN_SPI0_MISO=19
        SDCARD_PIN_SPI0_SCK=5
        SDCARD_PIN_SPI0_MOSI=18
        SDCARD_PIN_SPI0_CS=22

        SDCARD_PIO=pio1
        SDCARD_PIO_SM=0

        #PSRAM           # not implemented
        PSRAM_SPINLOCK=1 # not implemented
        PSRAM_ASYNC=1    # not implemented

        PSRAM_PIN_CS=3   # not implemented
        PSRAM_PIN_SCK=3  # not implemented
        PSRAM_PIN_MOSI=3 # not implemented
        PSRAM_PIN_MISO=3 # not implemented

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

        DEBUG=0
)
ELSEIF(ZERO)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}
        ZERO=1

        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        LOAD_WAV_PIO=17

        I2S_DATA_PIO=10 #
        I2S_BCK_PIO=11 #
        I2S_LCK_PIO=12

        PWM_PIN0=10
        PWM_PIN1=11
        BEEPER_PIN=12
        SOUND_FREQUENCY=30600

        # SDCARD
        SDCARD_PIN_SPI0_SCK=18
        SDCARD_PIN_SPI0_MOSI=19
        SDCARD_PIN_SPI0_MISO=20
        SDCARD_PIN_SPI0_CS=21

     #   PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=2
        PSRAM_PIN_SCK=3
        PSRAM_PIN_MOSI=4
        PSRAM_PIN_MISO=5

        # NES Gamepad
        USE_NESPAD
        NES_GPIO_CLK=7
        NES_GPIO_LAT=8
        NES_GPIO_DATA=9
        NES_GPIO_DATA2=10

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=22

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=22

        DEFAULT_THROTTLING=0
        PICO_RP2040=1
        PICO_DEFAULT_LED_PIN=25
    )
ELSEIF(ZERO2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}
        ZERO2=1

        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        LOAD_WAV_PIO=17

        I2S_DATA_PIO=10 #
        I2S_BCK_PIO=11 #
        I2S_LCK_PIO=12

        PWM_PIN0=10
        PWM_PIN1=11
        BEEPER_PIN=12
        SOUND_FREQUENCY=30600

        # SDCARD
        SDCARD_SPI_BUS=spi1
        SDCARD_PIN_SPI0_SCK=30
        SDCARD_PIN_SPI0_MOSI=31
        SDCARD_PIN_SPI0_MISO=40
        SDCARD_PIN_SPI0_CS=43

     #   PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=0
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=2
        PSRAM_PIN_SCK=3
        PSRAM_PIN_MOSI=4
        PSRAM_PIN_MISO=5

        # NES Gamepad
        #USE_NESPAD
        NES_GPIO_CLK=7
        NES_GPIO_LAT=8
        NES_GPIO_DATA=9
        NES_GPIO_DATA2=10

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=32

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=32

        PICO_DEFAULT_LED_PIN=25

        DEFAULT_THROTTLING=0
    )
ELSE()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}

        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        LOAD_WAV_PIO=22

        I2S_DATA_PIO=26 #
        I2S_BCK_PIO=27 #
        I2S_LCK_PIO=28

        PWM_PIN0=26 #
        PWM_PIN1=27 #
        BEEPER_PIN=28
        SOUND_FREQUENCY=30600

        # SDCARD
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=2
        SDCARD_PIN_SPI0_MOSI=3
        SDCARD_PIN_SPI0_MISO=4

        PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=18
        PSRAM_PIN_SCK=19
        PSRAM_PIN_MOSI=20
        PSRAM_PIN_MISO=21

        # NES Gamepad
        USE_NESPAD
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
    )
ENDIF()


IF(HWAY)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HWAY)
    SET(BUILD_NAME "${BUILD_NAME}-HWAY")
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
    SET(BUILD_NAME "${BUILD_NAME}-VGA-HDMI")
ENDIF()

SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}MHz-${PORT_VERSION}")

IF(m1p2)
    SET(BUILD_NAME "${BUILD_NAME}.m1p2")
ENDIF()

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${BUILD_NAME}")
