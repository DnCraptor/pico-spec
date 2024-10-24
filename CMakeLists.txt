cmake_minimum_required(VERSION 3.13)
# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(pico-spec C CXX ASM)
SET(BUILD_NAME "${PROJECT_NAME}")

option(I2S "Enable I2S sound" OFF)
option(I2S_CS4334 "Enable I2S CS4334 sound" OFF)
option(VGA "Enable VGA" OFF)
option(TFT "Enable TFT display" OFF)
option(ILI9341 "Enable TFT ILI9341 display" OFF)
option(HDMI "Enable HDMI display" OFF)
option(TV "Enable TV composite output" OFF)
option(SOFTTV "Enable TV soft composite output" OFF)

set (PORT_VERSION "0.3.0")

###set(I2S ON)
###set(HDMI ON)
###set(TV ON)
###set(SOFTTV ON)
#set(TFT ON)
#set(ILI9341 ON)

set(CPU_MHZ 378)

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
add_subdirectory(drivers/ps2)
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
file(GLOB_RECURSE SRC "src/*.cpp" "src/*.c fabgl/*.cpp" "src/roms/*.cpp" "src/roms/*.c" "src/miniz/*.c")

message(STATUS "Add source files:")
foreach (SRC_FILE IN LISTS SRC)
    message(STATUS "${SRC_FILE}")
endforeach ()
message(STATUS "")

add_executable(${PROJECT_NAME} ${SRC})

pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(${PROJECT_NAME} slower_boot2)

#set(FAMILY rp2040)
#set(BOARD pico_sdk)

pico_set_program_name(pico-spec "ESpectrum port for Murmulator - ZX Spectrum emulator by DnCraptor")
pico_set_program_version(pico-spec "test")

target_include_directories(${PROJECT_NAME} PRIVATE
        src
        src/roms
)

IF(TFT)
    target_link_libraries(${PROJECT_NAME} PRIVATE st7789)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TFT)
    SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-TFT-${PORT_VERSION}")
    IF(ILI9341)
        SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-ILI9341-${PORT_VERSION}")
        target_compile_definitions(${PROJECT_NAME} PRIVATE ILI9341)
    ELSE()
        SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-ST7789-${PORT_VERSION}")
    ENDIF()
ELSEIF(HDMI)
    target_link_libraries(${PROJECT_NAME} PRIVATE hdmi)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HDMI)
    SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-HDMI-${PORT_VERSION}")
ELSEIF(TV)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TV)
    target_link_libraries(${PROJECT_NAME} PRIVATE tv)
    SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-TV-${PORT_VERSION}")
ELSEIF(SOFTTV)
	target_compile_definitions(${PROJECT_NAME} PRIVATE SOFTTV)
	target_link_libraries(${PROJECT_NAME} PRIVATE tv-software)
	SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-TV-SOFT-${PORT_VERSION}")
ELSE()
    target_compile_definitions(${PROJECT_NAME} PRIVATE VGA_DRV)
    target_link_libraries(${PROJECT_NAME} PRIVATE vga-nextgen)
    SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-VGA-${PORT_VERSION}")
ENDIF()

IF(NOT I2S)
    target_compile_definitions(${PROJECT_NAME} PRIVATE AUDIO_PWM_PIN=26)
ELSEIF(I2S_CS4334)
    target_compile_definitions(${PROJECT_NAME} PRIVATE I2S_CS4334)
    SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-I2S-CS4334-${PORT_VERSION}")
ELSE()
    SET(BUILD_NAME "${BUILD_NAME}-${CPU_MHZ}-I2S-TDA1387-${PORT_VERSION}")
    target_compile_definitions(${PROJECT_NAME} PRIVATE I2S_SOUND=1)
ENDIF()

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${BUILD_NAME}")

target_link_libraries(${PROJECT_NAME} PRIVATE
        psram
        audio
        graphics

        ps2
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

target_compile_definitions(${PROJECT_NAME} PRIVATE
        PORT_VERSION="${PORT_VERSION}"
        CPU_MHZ=${CPU_MHZ}

        LOAD_WAV_PIO=22
        PWM_PIN0=26
        PWM_PIN1=27
        BEEPER_PIN=28

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
)

target_link_options(${PROJECT_NAME} PRIVATE -Xlinker --print-memory-usage --data-sections)
pico_add_extra_outputs(${PROJECT_NAME})
