@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

if not defined BUILD_TYPE set "BUILD_TYPE=MinSizeRel"
if not defined JOBS set "JOBS=%NUMBER_OF_PROCESSORS%"

:: Auto-detect Ninja from Pico SDK or system PATH
if not defined CMAKE_GENERATOR (
    set "PICO_NINJA=%USERPROFILE%\.pico-sdk\ninja\v1.12.1\ninja.exe"
    if exist "!PICO_NINJA!" (
        set "CMAKE_GENERATOR=Ninja"
        set "CMAKE_MAKE_PROGRAM=!PICO_NINJA!"
    ) else (
        where ninja >nul 2>&1
        if !errorlevel! equ 0 (
            set "CMAKE_GENERATOR=Ninja"
        ) else (
            where nmake >nul 2>&1
            if !errorlevel! equ 0 (
                set "CMAKE_GENERATOR=NMake Makefiles"
            ) else (
                echo Error: no build tool found (ninja or nmake)
                exit /b 1
            )
        )
    )
)

:: All available targets
set "ALL_TARGETS=MURM_P1 MURM_P2 MURM2 PICO_PC PICO_DV ZERO ZERO2"

:: Targets that support TFT+ILI9341
set "TFT_TARGETS=MURM_P1 MURM_P2 MURM2"

:: Parse arguments
if "%~1"=="" (
    set "TARGETS=%ALL_TARGETS%"
) else (
    set "TARGETS=%*"
)

echo === pico-spec multi-target build ===
echo Targets: %TARGETS%
echo Build type: %BUILD_TYPE%
echo Generator: %CMAKE_GENERATOR%
echo Parallel jobs: %JOBS%
echo.

set "FAILED="
set "SUCCEEDED="
set "OUTPUT_DIR=%SCRIPT_DIR%\firmware"
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

:: Build each target with VGA_HDMI, then TFT_ILI9341 for MURM targets
for %%T in (%TARGETS%) do (
    :: VGA_HDMI build
    call :build_target %%T VGA_HDMI

    :: TFT_ILI9341 build for MURM targets
    for %%M in (%TFT_TARGETS%) do (
        if "%%T"=="%%M" call :build_target %%T TFT_ILI9341
    )
)

echo ========================================
echo  Build summary
echo ========================================

if defined SUCCEEDED (
    echo Succeeded:!SUCCEEDED!
    for %%P in (!SUCCEEDED!) do (
        :: Parse TARGET:DISPLAY pair
        for /f "tokens=1,2 delims=:" %%A in ("%%P") do (
            if "%%B"=="VGA_HDMI" (
                set "BDIR=%SCRIPT_DIR%\build-%%A"
            ) else (
                set "BDIR=%SCRIPT_DIR%\build-%%A-%%B"
            )
        )
        for /r "!BDIR!" %%F in (*.uf2) do (
            copy /y "%%F" "%OUTPUT_DIR%\" >nul
            echo   %%~nxF
        )
    )
    echo.
    echo All .uf2 files copied to: %OUTPUT_DIR%\
)

if defined FAILED (
    echo Failed:!FAILED!
    exit /b 1
)

echo.
echo All builds completed successfully!
goto :eof

:build_target
:: %1 = target name, %2 = display variant (VGA_HDMI or TFT_ILI9341)
set "BT_TARGET=%~1"
set "BT_DISPLAY=%~2"

if "%BT_DISPLAY%"=="VGA_HDMI" (
    set "BT_BUILD_DIR=%SCRIPT_DIR%\build-%BT_TARGET%"
    set "BT_LABEL=%BT_TARGET% (VGA_HDMI)"
) else (
    set "BT_BUILD_DIR=%SCRIPT_DIR%\build-%BT_TARGET%-%BT_DISPLAY%"
    set "BT_LABEL=%BT_TARGET% (%BT_DISPLAY%)"
)

echo ========================================
echo  Building: !BT_LABEL!
echo  Build dir: !BT_BUILD_DIR!
echo ========================================

:: Clean stale CMake cache if generator changed
if exist "!BT_BUILD_DIR!\CMakeCache.txt" (
    for /f "tokens=2 delims==" %%G in ('findstr /b "CMAKE_GENERATOR:INTERNAL=" "!BT_BUILD_DIR!\CMakeCache.txt" 2^>nul') do (
        if not "%%G"=="%CMAKE_GENERATOR%" (
            echo   Generator changed, cleaning cache...
            del /q "!BT_BUILD_DIR!\CMakeCache.txt" 2>nul
            rmdir /s /q "!BT_BUILD_DIR!\CMakeFiles" 2>nul
        )
    )
)

if not exist "!BT_BUILD_DIR!" mkdir "!BT_BUILD_DIR!"

:: Map target to CMake flags
if "%BT_TARGET%"=="MURM_P1" (
    set "BT_TARGET_FLAGS=-DMURM=ON"
) else if "%BT_TARGET%"=="MURM_P2" (
    set "BT_TARGET_FLAGS=-DMURM=ON -DMURM_P2=ON"
) else (
    set "BT_TARGET_FLAGS=-D%BT_TARGET%=ON"
)

:: Display variant flags
set "BT_DISPLAY_FLAGS="
if "%BT_DISPLAY%"=="TFT_ILI9341" (
    set "BT_DISPLAY_FLAGS=-DTFT=ON -DILI9341=ON -DVGA_HDMI=OFF"
)

set "BT_CMAKE_ARGS=-B "!BT_BUILD_DIR!" -S "%SCRIPT_DIR%" -G "%CMAKE_GENERATOR%" !BT_TARGET_FLAGS! !BT_DISPLAY_FLAGS! -DCMAKE_BUILD_TYPE=%BUILD_TYPE%"
if defined CMAKE_MAKE_PROGRAM (
    set "BT_CMAKE_ARGS=!BT_CMAKE_ARGS! -DCMAKE_MAKE_PROGRAM=!CMAKE_MAKE_PROGRAM!"
)

cmake !BT_CMAKE_ARGS! && cmake --build "!BT_BUILD_DIR!" --parallel %JOBS%
if !errorlevel! equ 0 (
    set "SUCCEEDED=!SUCCEEDED! %BT_TARGET%:%BT_DISPLAY%"
    echo.
    echo [OK] !BT_LABEL! build succeeded
) else (
    set "FAILED=!FAILED! %BT_TARGET%:%BT_DISPLAY%"
    echo.
    echo [FAIL] !BT_LABEL! build failed
)
echo.
goto :eof
