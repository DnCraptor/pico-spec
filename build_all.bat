@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

if not defined BUILD_TYPE set "BUILD_TYPE=MinSizeRel"
if not defined MAX_PARALLEL set "MAX_PARALLEL=3"

:: Parse flags (--clean, -p, -j) before positional targets.
set "CLEAN=0"
set "POSARGS="
:parse_args
if "%~1"=="" goto parse_done
if /i "%~1"=="--clean" ( set "CLEAN=1" & shift & goto parse_args )
if /i "%~1"=="-p"      ( set "MAX_PARALLEL=%~2" & shift & shift & goto parse_args )
if /i "%~1"=="-j"      ( set "JOBS_PER_BUILD=%~2" & shift & shift & goto parse_args )
if /i "%~1"=="--help"  goto show_help
if /i "%~1"=="-h"      goto show_help
set "POSARGS=!POSARGS! %~1"
shift
goto parse_args
:parse_done

:: JOBS_PER_BUILD default = ceil(NUMBER_OF_PROCESSORS / MAX_PARALLEL)
if not defined JOBS_PER_BUILD (
    set /a "JOBS_PER_BUILD=(%NUMBER_OF_PROCESSORS% + %MAX_PARALLEL% - 1) / %MAX_PARALLEL%"
)

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

:: ccache if available
set "CCACHE_ARGS="
set "HAVE_CCACHE=0"
where ccache >nul 2>&1
if !errorlevel! equ 0 (
    set "CCACHE_ARGS=-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_ASM_COMPILER_LAUNCHER=ccache"
    if not defined CCACHE_DIR set "CCACHE_DIR=%USERPROFILE%\.ccache"
    ccache --max-size=5G >nul 2>&1
    set "HAVE_CCACHE=1"
)

:: All available targets
set "ALL_TARGETS=MURM_P1 MURM_P2 MURM2_P2 PICO_PC PICO_DV ZERO ZERO2"
set "TFT_TARGETS=MURM_P1 MURM_P2 MURM2_P2"
set "TFT_ST_TARGETS=MURM2_P1"
set "SOFTTV_TARGETS=MURM_P1 MURM_P2 MURM2_P2"

if "%POSARGS%"=="" (
    set "TARGETS=%ALL_TARGETS%"
) else (
    set "TARGETS=%POSARGS%"
)

:: Build PAIRS list as "TARGET:DISPLAY TARGET:DISPLAY ..."
set "PAIRS="
for %%T in (%TARGETS%) do (
    set "PAIRS=!PAIRS! %%T:VGA_HDMI"
    for %%M in (%TFT_TARGETS%) do ( if "%%T"=="%%M" set "PAIRS=!PAIRS! %%T:TFT_ILI9341" )
    for %%M in (%TFT_ST_TARGETS%) do ( if "%%T"=="%%M" set "PAIRS=!PAIRS! %%T:TFT_ST7789" )
    for %%M in (%SOFTTV_TARGETS%) do ( if "%%T"=="%%M" set "PAIRS=!PAIRS! %%T:SOFTTV" )
)

:: Count pairs
set "PAIRS_COUNT=0"
for %%P in (%PAIRS%) do set /a PAIRS_COUNT+=1

echo === pico-spec multi-target build ===
echo Targets:          %TARGETS%
echo Pairs:            %PAIRS_COUNT%
echo Build type:       %BUILD_TYPE%
echo Generator:        %CMAKE_GENERATOR%
echo Parallel targets: %MAX_PARALLEL%  (jobs per target: %JOBS_PER_BUILD%, CPUs=%NUMBER_OF_PROCESSORS%)
if "%HAVE_CCACHE%"=="1" ( echo ccache:           enabled ) else ( echo ccache:           not installed ^(choco install ccache / scoop install ccache^) )
echo Clean first:      %CLEAN%
echo.

set "OUTPUT_DIR=%SCRIPT_DIR%\firmware"
set "LOG_DIR=%SCRIPT_DIR%\build-logs"
set "STATE_DIR=%SCRIPT_DIR%\build-logs\.state"
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"
if not exist "%LOG_DIR%"    mkdir "%LOG_DIR%"
if exist "%STATE_DIR%" rmdir /s /q "%STATE_DIR%"
mkdir "%STATE_DIR%"

:: Optional clean
if "%CLEAN%"=="1" (
    for %%P in (%PAIRS%) do (
        for /f "tokens=1,2 delims=:" %%A in ("%%P") do (
            if "%%B"=="VGA_HDMI" ( set "BD=%SCRIPT_DIR%\build-%%A" ) else ( set "BD=%SCRIPT_DIR%\build-%%A-%%B" )
            if exist "!BD!" (
                echo Cleaning !BD! ...
                rmdir /s /q "!BD!"
            )
        )
    )
    echo.
)

:: Launch pairs with bounded concurrency.
:: Each background worker is `cmd /c "call %0 :worker TARGET DISPLAY"`.
:: We track running count via marker files in %STATE_DIR%.
set "START_TIME=%TIME%"

for %%P in (%PAIRS%) do (
    for /f "tokens=1,2 delims=:" %%A in ("%%P") do (
        :wait_slot
        call :count_running RUNNING
        if !RUNNING! geq %MAX_PARALLEL% (
            :: Sleep 1 sec without PowerShell dependency
            ping -n 2 127.0.0.1 >nul
            goto wait_slot
        )
        echo Starting %%A:%%B ...
        :: Unique marker per job: %%A-%%B.running
        type nul > "%STATE_DIR%\%%A-%%B.running"
        start "" /b cmd /c "call "%~f0" :worker %%A %%B > "%LOG_DIR%\%%A-%%B.log" 2>&1"
    )
)

:: Wait for all to finish
:wait_all
call :count_running RUNNING
if !RUNNING! gtr 0 (
    ping -n 2 127.0.0.1 >nul
    goto wait_all
)

echo.
echo ========================================
echo  Build summary
echo ========================================

set "SUCCEEDED="
set "FAILED="
for %%F in ("%STATE_DIR%\*.ok") do (
    set "N=%%~nF"
    set "SUCCEEDED=!SUCCEEDED! !N!"
)
for %%F in ("%STATE_DIR%\*.fail") do (
    set "N=%%~nF"
    set "FAILED=!FAILED! !N!"
)

if defined SUCCEEDED (
    echo Succeeded:!SUCCEEDED!
    for %%E in (!SUCCEEDED!) do (
        set "ENTRY=%%E"
        :: %%E is "TARGET-DISPLAY"; reconstruct build dir
        for /f "tokens=1,* delims=-" %%A in ("%%E") do (
            set "T=%%A"
            set "D=%%B"
        )
        :: Handle TARGET names containing '-' (e.g. MURM_P1-VGA_HDMI has none, but robustness)
        if "!D!"=="VGA_HDMI" (
            set "BD=%SCRIPT_DIR%\build-!T!"
        ) else (
            set "BD=%SCRIPT_DIR%\build-!T!-!D!"
        )
        for /r "!BD!" %%F in (*.uf2) do (
            copy /y "%%F" "%OUTPUT_DIR%\" >nul
            echo   %%~nxF
        )
    )
    echo.
    echo All .uf2 files copied to: %OUTPUT_DIR%\
)

if defined FAILED (
    echo.
    echo Failed:!FAILED!
    echo   See logs in %LOG_DIR%\
    rmdir /s /q "%STATE_DIR%"
    exit /b 1
)

if "%HAVE_CCACHE%"=="1" (
    echo.
    ccache -s 2>nul | findstr /i "cache"
)

rmdir /s /q "%STATE_DIR%"
echo.
echo All builds completed successfully!
goto :eof

:::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Helpers
:::::::::::::::::::::::::::::::::::::::::::::::::::::

:count_running
set /a _CNT=0
for %%F in ("%STATE_DIR%\*.running") do set /a _CNT+=1
set "%~1=!_CNT!"
goto :eof

:show_help
echo Usage: %~nx0 [--clean] [-j JOBS_PER_BUILD] [-p MAX_PARALLEL] [TARGETS...]
echo.
echo Env vars: BUILD_TYPE, MAX_PARALLEL, JOBS_PER_BUILD, CMAKE_GENERATOR
echo Targets:  MURM_P1 MURM_P2 MURM2_P1 MURM2_P2 PICO_PC PICO_DV ZERO ZERO2 (default: all)
exit /b 0

:::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Worker subroutine: called as `%0 :worker TARGET DISPLAY`
:: Runs one build, drops .ok/.fail marker, removes .running marker.
:::::::::::::::::::::::::::::::::::::::::::::::::::::
:worker
setlocal enabledelayedexpansion
set "W_TARGET=%~1"
set "W_DISPLAY=%~2"
set "W_MARK=%STATE_DIR%\%W_TARGET%-%W_DISPLAY%"

if "%W_DISPLAY%"=="VGA_HDMI" (
    set "W_BUILD_DIR=%SCRIPT_DIR%\build-%W_TARGET%"
    set "W_LABEL=%W_TARGET% (VGA_HDMI)"
) else (
    set "W_BUILD_DIR=%SCRIPT_DIR%\build-%W_TARGET%-%W_DISPLAY%"
    set "W_LABEL=%W_TARGET% (%W_DISPLAY%)"
)

echo ========================================
echo  Building: %W_LABEL%
echo  Build dir: %W_BUILD_DIR%
echo ========================================

:: Clean stale CMake cache if generator or platform changed
if exist "%W_BUILD_DIR%\CMakeCache.txt" (
    set "W_NEED_CLEAN=0"
    for /f "tokens=2 delims==" %%G in ('findstr /b "CMAKE_GENERATOR:INTERNAL=" "%W_BUILD_DIR%\CMakeCache.txt" 2^>nul') do (
        if not "%%G"=="%CMAKE_GENERATOR%" (
            echo   Generator changed, cleaning cache...
            set "W_NEED_CLEAN=1"
        )
    )
    if "%W_TARGET%"=="MURM_P1" ( set "W_EXPECTED=rp2040" ) else if "%W_TARGET%"=="MURM2_P1" ( set "W_EXPECTED=rp2040" ) else if "%W_TARGET%"=="ZERO" ( set "W_EXPECTED=rp2040" ) else ( set "W_EXPECTED=rp2350-arm-s" )
    for /f "tokens=2 delims==" %%G in ('findstr /b "PICO_PLATFORM:" "%W_BUILD_DIR%\CMakeCache.txt" 2^>nul') do (
        if not "%%G"=="!W_EXPECTED!" (
            echo   Platform changed (%%G -^> !W_EXPECTED!), cleaning cache...
            set "W_NEED_CLEAN=1"
        )
    )
    if "!W_NEED_CLEAN!"=="1" (
        del /q "%W_BUILD_DIR%\CMakeCache.txt" 2>nul
        rmdir /s /q "%W_BUILD_DIR%\CMakeFiles" 2>nul
    )
)

if not exist "%W_BUILD_DIR%" mkdir "%W_BUILD_DIR%"

if "%W_TARGET%"=="MURM_P1" (
    set "W_FLAGS=-DMURM=ON"
) else if "%W_TARGET%"=="MURM_P2" (
    set "W_FLAGS=-DMURM=ON -DMURM_P2=ON"
) else if "%W_TARGET%"=="MURM2_P1" (
    set "W_FLAGS=-DMURM2=ON"
) else if "%W_TARGET%"=="MURM2_P2" (
    set "W_FLAGS=-DMURM2=ON -DMURM2_P2=ON"
) else (
    set "W_FLAGS=-D%W_TARGET%=ON"
)

if "%W_DISPLAY%"=="TFT_ILI9341" set "W_FLAGS=!W_FLAGS! -DTFT=ON -DILI9341=ON -DVGA_HDMI=OFF"
if "%W_DISPLAY%"=="TFT_ST7789"  set "W_FLAGS=!W_FLAGS! -DTFT=ON -DST7789=ON -DVGA_HDMI=OFF"
if "%W_DISPLAY%"=="SOFTTV"      set "W_FLAGS=!W_FLAGS! -DSOFTTV=ON -DVGA_HDMI=OFF"

set "W_ARGS=-B "%W_BUILD_DIR%" -S "%SCRIPT_DIR%" -G "%CMAKE_GENERATOR%" !W_FLAGS! %CCACHE_ARGS% -DCMAKE_BUILD_TYPE=%BUILD_TYPE%"
if defined CMAKE_MAKE_PROGRAM set "W_ARGS=!W_ARGS! -DCMAKE_MAKE_PROGRAM=!CMAKE_MAKE_PROGRAM!"

cmake !W_ARGS!
if !errorlevel! neq 0 goto worker_fail
cmake --build "%W_BUILD_DIR%" --parallel %JOBS_PER_BUILD%
if !errorlevel! neq 0 goto worker_fail

echo.
echo [OK] %W_LABEL% build succeeded
type nul > "%W_MARK%.ok"
del /q "%W_MARK%.running" 2>nul
endlocal
exit /b 0

:worker_fail
echo.
echo [FAIL] %W_LABEL% build failed
type nul > "%W_MARK%.fail"
del /q "%W_MARK%.running" 2>nul
endlocal
exit /b 1
