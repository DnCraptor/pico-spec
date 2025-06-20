cmake_minimum_required(VERSION 3.13)
# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(p1-spec C CXX ASM)

option(I2S "Enable I2S sound" OFF)
option(I2S_CS4334 "Enable I2S CS4334 sound" OFF)
option(VGA "Enable VGA" OFF)
option(TFT "Enable TFT display" OFF)
option(ILI9341 "Enable TFT ILI9341 display" OFF)
option(HDMI "Enable HDMI display" OFF)
option(TV "Enable TV composite output" OFF)
option(SOFTTV "Enable TV soft composite output" OFF)
option(MURM20 "Murmulator 2.0" OFF)
option(ZERO "RP2040-PiZero" OFF)
option(PICO_PC "RP2040-PICO-PC" OFF)

set (PORT_VERSION "1.2.D")

#set(I2S ON)

##set(TV ON)
#set(SOFTTV ON)
#set(TFT ON)
#set(ILI9341 ON)
#set(HDMI ON)

#set(ZERO ON)
#set(PICO_PC ON)
#set(MURM2 ON)
#set(MONSTR ON)
set(KBDUSB ON)

#set(FLASH_SIZE 4096)
set(FLASH_SIZE 2048)
set(FLASH_SIZE_GREEN ON)
#set(CPU_MHZ 408)
set(CPU_MHZ 378)
#set(CPU_MHZ 252)

IF(MURM2)
    SET(BUILD_NAME "m2${PROJECT_NAME}-${FLASH_SIZE}-${CPU_MHZ}")
ELSEIF(ZERO)
    SET(BUILD_NAME "z0${PROJECT_NAME}-${FLASH_SIZE}-${CPU_MHZ}")
ELSEIF(PICO_PC)
    SET(BUILD_NAME "PC${PROJECT_NAME}-${FLASH_SIZE}-${CPU_MHZ}")
ELSE()
    SET(BUILD_NAME "m1${PROJECT_NAME}-${FLASH_SIZE}-${CPU_MHZ}")
ENDIF()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}  -funroll-loops -ffast-math -feliminate-unused-debug-types -ffunction-sections -fdata-sections -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funroll-loops -ffast-math -feliminate-unused-debug-types -ffunction-sections -fdata-sections -O2")

pico_sdk_init()

set(OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}")

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

# INCLUDE FILES THAT SHOULD BE COMPILED:
file(GLOB_RECURSE SRC "src/*.cpp" "src/*.c" "fabgl/*.cpp" "src/roms/*.cpp" "src/roms/*.c" "src/miniz/*.c")

message(STATUS "Add source files:")
foreach (SRC_FILE IN LISTS SRC)
    message(STATUS "${SRC_FILE}")
endforeach ()
message(STATUS "")

add_executable(${PROJECT_NAME} ${SRC})

pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(${PROJECT_NAME} slower_boot2)

pico_set_program_name(p1-spec "ESpectrum port for Murmulator - ZX Spectrum emulator by DnCraptor")
pico_set_program_version(p1-spec "test")

target_include_directories(${PROJECT_NAME} PRIVATE
        src
        src/roms
)

IF(NOT I2S)
    SET(BUILD_NAME "${BUILD_NAME}-PWM")
    if(ZERO)
        target_compile_definitions(${PROJECT_NAME} PRIVATE AUDIO_PWM_PIN=11)
    else()
        target_compile_definitions(${PROJECT_NAME} PRIVATE AUDIO_PWM_PIN=26)
    endif()
ELSEIF(I2S_CS4334)
    target_compile_definitions(${PROJECT_NAME} PRIVATE I2S_CS4334)
    SET(BUILD_NAME "${BUILD_NAME}-CS4334")
ELSE()
    SET(BUILD_NAME "${BUILD_NAME}-I2S")
    target_compile_definitions(${PROJECT_NAME} PRIVATE I2S_SOUND=1)
ENDIF()

IF(TFT)
    target_link_libraries(${PROJECT_NAME} PRIVATE st7789)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TFT)
    IF(ILI9341)
        target_compile_definitions(${PROJECT_NAME} PRIVATE ILI9341)
        SET(BUILD_NAME "${BUILD_NAME}-ILI9341-${PORT_VERSION}")
    ELSE()
        SET(BUILD_NAME "${BUILD_NAME}-ST7789-${PORT_VERSION}")
    ENDIF()
ELSEIF(HDMI)
    target_link_libraries(${PROJECT_NAME} PRIVATE hdmi)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HDMI)
    SET(BUILD_NAME "${BUILD_NAME}-HDMI-${PORT_VERSION}")
ELSEIF(TV)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TV)
    target_link_libraries(${PROJECT_NAME} PRIVATE tv)
    SET(BUILD_NAME "${BUILD_NAME}-TV-${PORT_VERSION}")
ELSEIF(SOFTTV)
	target_compile_definitions(${PROJECT_NAME} PRIVATE SOFTTV)
	target_link_libraries(${PROJECT_NAME} PRIVATE tv-software)
	SET(BUILD_NAME "${BUILD_NAME}-TV-SOFT-${PORT_VERSION}")
ELSE()
    target_compile_definitions(${PROJECT_NAME} PRIVATE VGA_DRV)
    target_link_libraries(${PROJECT_NAME} PRIVATE vga-nextgen)
    SET(BUILD_NAME "${BUILD_NAME}-VGA-${PORT_VERSION}")
ENDIF()

IF(FLASH_SIZE_GREEN)
target_compile_definitions(${PROJECT_NAME} PRIVATE
        NO_SPAIN_ROM_48k=1
        NO_SPAIN_ROM_128k=1
        CARTRIDGE_AS_CUSTOM=1
        NO_SEPARATE_48K_CUSTOM=1
        NO_ALF=1
)
ENDIF()

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${BUILD_NAME}")

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

        tinyusb_host tinyusb_board
)

IF(KBDUSB)
    add_subdirectory(drivers/ps2kbd)
    target_link_libraries(${PROJECT_NAME} PRIVATE ps2kbd)
    target_compile_definitions(${PROJECT_NAME} PRIVATE KBDUSB)
ELSE()
    add_subdirectory(drivers/ps2)
    target_link_libraries(${PROJECT_NAME} PRIVATE ps2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE KBDPS2)
ENDIF()

IF(MURM2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}

        KBD_CLOCK_PIN=2
        KBD_DATA_PIN=3

        LOAD_WAV_PIO=22

        BEEPER_PIN=9
        PWM_PIN0=10
        PWM_PIN1=11
        SOUND_FREQUENCY=30600
        I2S_FREQUENCY=30600

        # SDCARD
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=6
        SDCARD_PIN_SPI0_MOSI=7
        SDCARD_PIN_SPI0_MISO=4

        #PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=8
        PSRAM_PIN_SCK=6
        PSRAM_PIN_MOSI=7
        PSRAM_PIN_MISO=4

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
        
        PICO_FLASH_SIZE_BYTES=${FLASH_SIZE}*1024
        DEFAULT_THROTTLING=0
    )
ELSEIF(ZERO)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}
        ZERO=1

        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        LOAD_WAV_PIO=17

        PWM_PIN0=11
        PWM_PIN1=12
        BEEPER_PIN=13
        SOUND_FREQUENCY=30600
        I2S_FREQUENCY=30600

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

        # TFT
        TFT_CS_PIN=22
        TFT_RST_PIN=24
        TFT_LED_PIN=25
        TFT_DC_PIN=26
        TFT_DATA_PIN=28
        TFT_CLK_PIN=29
        
        PICO_FLASH_SIZE_BYTES=${FLASH_SIZE}*1024
        DEFAULT_THROTTLING=0
    )
ELSEIF(PICO_PC)
    set(DVI_DEFAULT_SERIAL_CONFIG "rp240_pico_pc_cfg" CACHE STRING
        "Select a default pin configuration from common_dvi_pin_configs.h")

    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
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

        BEEPER_PIN=26 # not implemented
        PWM_PIN0=27 # R
        PWM_PIN1=28 # L
        SOUND_FREQUENCY=30600
        I2S_FREQUENCY=30600

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
        NES_GPIO_CLK=8    # UXT1-6
        NES_GPIO_LAT=9    # UXT1-5
        NES_GPIO_DATA=20  # UXT1-3
        NES_GPIO_DATA2=21 # UXT1-4

        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=12

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=12
        
        PICO_FLASH_SIZE_BYTES=${FLASH_SIZE}*1024
        DEFAULT_THROTTLING=0
    )
ELSEIF(MONSTR)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}

        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

    #    LOAD_WAV_PIO=22
        PWM_PIN0=26
        PWM_PIN1=27
        BEEPER_PIN=28
        SOUND_FREQUENCY=30600
        I2S_FREQUENCY=30600

        # SDCARD
        SDCARD_PIN_SPI0_CS=23 
        SDCARD_PIN_SPI0_SCK=18 
        SDCARD_PIN_SPI0_MOSI=19 
        SDCARD_PIN_SPI0_MISO=20
        
        #PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=5
        PSRAM_PIN_SCK=2
        PSRAM_PIN_MOSI=3
        PSRAM_PIN_MISO=4

        # NES Gamepad
    #    USE_NESPAD
        NES_GPIO_CLK=14
        NES_GPIO_LAT=15
        NES_GPIO_DATA=16

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

        PICO_FLASH_SIZE_BYTES=${FLASH_SIZE}*1024
        DEFAULT_THROTTLING=0
    )
ELSE()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}

        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        LOAD_WAV_PIO=22
        PWM_PIN0=26
        PWM_PIN1=27
        BEEPER_PIN=28
        SOUND_FREQUENCY=30600
        I2S_FREQUENCY=30600

        # SDCARD
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=2
        SDCARD_PIN_SPI0_MOSI=3
        SDCARD_PIN_SPI0_MISO=4
        
        PSRAM
        PSRAM_NO_FUGE
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=18
        PSRAM_PIN_SCK=19
        PSRAM_PIN_MOSI=20
        PSRAM_PIN_MISO=21

#        PSRAM_PIN_CS=17
#        PSRAM_PIN_SCK=18
#        PSRAM_PIN_MOSI=19
#        PSRAM_PIN_MISO=16

        # NES Gamepad
        USE_NESPAD
        NES_GPIO_CLK=14
        NES_GPIO_LAT=15
        NES_GPIO_DATA=16
    #    NES_GPIO_DATA=28 - instead of BEEPER_PIN

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

	    PICO_FLASH_SIZE_BYTES=${FLASH_SIZE}*1024
        DEFAULT_THROTTLING=0
    )
ENDIF()

pico_set_linker_script(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/memmap.ld")
target_link_options(${PROJECT_NAME} PRIVATE -Xlinker --print-memory-usage --data-sections)
pico_add_extra_outputs(${PROJECT_NAME})
