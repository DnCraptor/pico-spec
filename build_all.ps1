param(
    [string[]]$Targets,
    [string]$BuildType = $env:BUILD_TYPE,
    [int]$Jobs = 0
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not $BuildType) { $BuildType = "MinSizeRel" }
if ($Jobs -eq 0) {
    if ($env:JOBS) { $Jobs = [int]$env:JOBS }
    else { $Jobs = (Get-CimInstance Win32_Processor).NumberOfLogicalProcessors }
}

# Auto-detect Ninja from Pico SDK or system PATH
$CMakeGenerator = $env:CMAKE_GENERATOR
$CMakeMakeProgram = $null

if (-not $CMakeGenerator) {
    $PicoNinja = Join-Path $env:USERPROFILE ".pico-sdk\ninja\v1.12.1\ninja.exe"
    if (Test-Path $PicoNinja) {
        $CMakeGenerator = "Ninja"
        $CMakeMakeProgram = $PicoNinja
    } elseif (Get-Command ninja -ErrorAction SilentlyContinue) {
        $CMakeGenerator = "Ninja"
    } elseif (Get-Command nmake -ErrorAction SilentlyContinue) {
        $CMakeGenerator = "NMake Makefiles"
    } else {
        Write-Error "No build tool found (ninja or nmake)"
        exit 1
    }
}

# All available targets
$AllTargets = @("MURM", "MURM2", "PICO_PC", "PICO_DV", "ZERO", "ZERO2")

if ($Targets.Count -eq 0) { $Targets = $AllTargets }

Write-Host "=== pico-spec multi-target build ==="
Write-Host "Targets: $($Targets -join ' ')"
Write-Host "Build type: $BuildType"
Write-Host "Generator: $CMakeGenerator"
Write-Host "Parallel jobs: $Jobs"
Write-Host ""

$Failed = @()
$Succeeded = @()
$OutputDir = Join-Path $ScriptDir "firmware"
if (-not (Test-Path $OutputDir)) { New-Item -ItemType Directory -Path $OutputDir | Out-Null }

foreach ($Target in $Targets) {
    $BuildDir = Join-Path $ScriptDir "build-$Target"

    Write-Host "========================================"
    Write-Host " Building: $Target"
    Write-Host " Build dir: $BuildDir"
    Write-Host "========================================"

    # Clean stale CMake cache if generator changed
    $CacheFile = Join-Path $BuildDir "CMakeCache.txt"
    if (Test-Path $CacheFile) {
        $CachedGen = (Select-String -Path $CacheFile -Pattern "^CMAKE_GENERATOR:INTERNAL=(.*)" |
            Select-Object -First 1).Matches.Groups[1].Value
        if ($CachedGen -and $CachedGen -ne $CMakeGenerator) {
            Write-Host "  Generator changed ($CachedGen -> $CMakeGenerator), cleaning cache..."
            Remove-Item $CacheFile -Force
            $CMakeFilesDir = Join-Path $BuildDir "CMakeFiles"
            if (Test-Path $CMakeFilesDir) { Remove-Item $CMakeFilesDir -Recurse -Force }
        }
    }

    if (-not (Test-Path $BuildDir)) { New-Item -ItemType Directory -Path $BuildDir | Out-Null }

    $CMakeArgs = @(
        "-B", $BuildDir, "-S", $ScriptDir,
        "-G", $CMakeGenerator,
        "-D${Target}=ON",
        "-DCMAKE_BUILD_TYPE=$BuildType"
    )
    if ($CMakeMakeProgram) {
        $CMakeArgs += "-DCMAKE_MAKE_PROGRAM=$CMakeMakeProgram"
    }

    $ok = $true
    & cmake @CMakeArgs
    if ($LASTEXITCODE -ne 0) { $ok = $false }

    if ($ok) {
        & cmake --build $BuildDir --parallel $Jobs
        if ($LASTEXITCODE -ne 0) { $ok = $false }
    }

    if ($ok) {
        $Succeeded += $Target
        Write-Host ""
        Write-Host "[OK] $Target build succeeded"
    } else {
        $Failed += $Target
        Write-Host ""
        Write-Host "[FAIL] $Target build failed"
    }
    Write-Host ""
}

Write-Host "========================================"
Write-Host " Build summary"
Write-Host "========================================"

if ($Succeeded.Count -gt 0) {
    Write-Host "Succeeded: $($Succeeded -join ' ')"
    foreach ($Target in $Succeeded) {
        Get-ChildItem -Path (Join-Path $ScriptDir "build-$Target") -Filter "*.uf2" -Recurse | ForEach-Object {
            Copy-Item $_.FullName -Destination $OutputDir -Force
            Write-Host "  $($_.Name)"
        }
    }
    Write-Host ""
    Write-Host "All .uf2 files copied to: $OutputDir\"
}

if ($Failed.Count -gt 0) {
    Write-Host "Failed: $($Failed -join ' ')"
    exit 1
}

Write-Host ""
Write-Host "All builds completed successfully!"
