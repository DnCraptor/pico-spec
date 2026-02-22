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
set "ALL_TARGETS=MURM MURM2 PICO_PC PICO_DV ZERO ZERO2"

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

for %%T in (%TARGETS%) do (
    set "TARGET=%%T"
    set "BUILD_DIR=%SCRIPT_DIR%\build-%%T"

    echo ========================================
    echo  Building: %%T
    echo  Build dir: !BUILD_DIR!
    echo ========================================

    :: Clean stale CMake cache if generator changed
    if exist "!BUILD_DIR!\CMakeCache.txt" (
        for /f "tokens=2 delims==" %%G in ('findstr /b "CMAKE_GENERATOR:INTERNAL=" "!BUILD_DIR!\CMakeCache.txt" 2^>nul') do (
            if not "%%G"=="%CMAKE_GENERATOR%" (
                echo   Generator changed, cleaning cache...
                del /q "!BUILD_DIR!\CMakeCache.txt" 2>nul
                rmdir /s /q "!BUILD_DIR!\CMakeFiles" 2>nul
            )
        )
    )

    if not exist "!BUILD_DIR!" mkdir "!BUILD_DIR!"

    set "CMAKE_ARGS=-B "!BUILD_DIR!" -S "%SCRIPT_DIR%" -G "%CMAKE_GENERATOR%" -D%%T=ON -DCMAKE_BUILD_TYPE=%BUILD_TYPE%"
    if defined CMAKE_MAKE_PROGRAM (
        set "CMAKE_ARGS=!CMAKE_ARGS! -DCMAKE_MAKE_PROGRAM=!CMAKE_MAKE_PROGRAM!"
    )

    cmake !CMAKE_ARGS! && cmake --build "!BUILD_DIR!" --parallel %JOBS%
    if !errorlevel! equ 0 (
        set "SUCCEEDED=!SUCCEEDED! %%T"
        echo.
        echo [OK] %%T build succeeded
    ) else (
        set "FAILED=!FAILED! %%T"
        echo.
        echo [FAIL] %%T build failed
    )
    echo.
)

echo ========================================
echo  Build summary
echo ========================================

if defined SUCCEEDED (
    echo Succeeded:!SUCCEEDED!
    for %%T in (!SUCCEEDED!) do (
        for /r "%SCRIPT_DIR%\build-%%T" %%F in (*.uf2) do (
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
