param(
    [string[]]$Targets,
    [string]$BuildType = $env:BUILD_TYPE,
    [int]$JobsPerBuild = 0,
    [int]$MaxParallel = 0,
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not $BuildType) { $BuildType = "MinSizeRel" }
$NProc = (Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfLogicalProcessors -Sum).Sum
if ($MaxParallel -le 0) {
    if ($env:MAX_PARALLEL) { $MaxParallel = [int]$env:MAX_PARALLEL } else { $MaxParallel = 3 }
}
if ($JobsPerBuild -le 0) {
    if ($env:JOBS_PER_BUILD) { $JobsPerBuild = [int]$env:JOBS_PER_BUILD }
    else { $JobsPerBuild = [math]::Ceiling($NProc / $MaxParallel) }
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

# ccache: use if available
$CCacheArgs = @()
$HaveCCache = $false
if (Get-Command ccache -ErrorAction SilentlyContinue) {
    $CCacheArgs = @(
        "-DCMAKE_C_COMPILER_LAUNCHER=ccache",
        "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
        "-DCMAKE_ASM_COMPILER_LAUNCHER=ccache"
    )
    if (-not $env:CCACHE_DIR) { $env:CCACHE_DIR = Join-Path $env:USERPROFILE ".ccache" }
    try { & ccache --max-size=5G | Out-Null } catch {}
    $HaveCCache = $true
}

$AllTargets    = @("MURM_P1", "MURM_P2", "MURM2", "PICO_PC", "PICO_DV", "ZERO", "ZERO2")
$TftTargets    = @("MURM_P1", "MURM_P2", "MURM2")
$SofttvTargets = @("MURM_P1", "MURM_P2", "MURM2")

if (-not $Targets -or $Targets.Count -eq 0) { $Targets = $AllTargets }

# Build list of (target, display) pairs
$BuildPairs = @()
foreach ($Target in $Targets) {
    $BuildPairs += ,@{ Target = $Target; Display = "VGA_HDMI" }
    if ($TftTargets -contains $Target) {
        $BuildPairs += ,@{ Target = $Target; Display = "TFT_ILI9341" }
    }
    if ($SofttvTargets -contains $Target) {
        $BuildPairs += ,@{ Target = $Target; Display = "SOFTTV" }
    }
}

Write-Host "=== pico-spec multi-target build ==="
Write-Host "Targets:          $($Targets -join ' ')"
Write-Host "Pairs:            $($BuildPairs.Count)"
Write-Host "Build type:       $BuildType"
Write-Host "Generator:        $CMakeGenerator"
Write-Host "Parallel targets: $MaxParallel  (jobs per target: $JobsPerBuild, nproc=$NProc)"
if ($HaveCCache) { Write-Host "ccache:           enabled" }
else { Write-Host "ccache:           not installed (choco install ccache / scoop install ccache → 2-5x faster rebuilds)" }
Write-Host "Clean first:      $Clean"
Write-Host ""

$OutputDir = Join-Path $ScriptDir "firmware"
$LogDir    = Join-Path $ScriptDir "build-logs"
if (-not (Test-Path $OutputDir)) { New-Item -ItemType Directory -Path $OutputDir | Out-Null }
if (-not (Test-Path $LogDir))    { New-Item -ItemType Directory -Path $LogDir    | Out-Null }

function Get-BuildDir($Target, $Display) {
    if ($Display -eq "VGA_HDMI") { Join-Path $ScriptDir "build-$Target" }
    else { Join-Path $ScriptDir "build-${Target}-${Display}" }
}

# Optional clean
if ($Clean) {
    foreach ($Pair in $BuildPairs) {
        $Bd = Get-BuildDir $Pair.Target $Pair.Display
        if (Test-Path $Bd) {
            Write-Host "Cleaning $Bd ..."
            Remove-Item $Bd -Recurse -Force
        }
    }
    Write-Host ""
}

# ScriptBlock executed per pair (in parallel).
# Writes log to $LogDir and returns [pscustomobject] with result.
$Worker = {
    param($Pair, $ScriptDir, $BuildType, $CMakeGenerator, $CMakeMakeProgram,
          $CCacheArgs, $JobsPerBuild, $LogDir)

    $Target  = $Pair.Target
    $Display = $Pair.Display
    if ($Display -eq "VGA_HDMI") {
        $BuildDir = Join-Path $ScriptDir "build-$Target"
        $Label    = "$Target (VGA_HDMI)"
    } else {
        $BuildDir = Join-Path $ScriptDir "build-${Target}-${Display}"
        $Label    = "$Target ($Display)"
    }
    $LogFile = Join-Path $LogDir "${Target}-${Display}.log"
    $Start   = Get-Date

    try {
        "========================================" | Out-File $LogFile
        " Building: $Label"                        | Out-File $LogFile -Append
        " Build dir: $BuildDir"                    | Out-File $LogFile -Append
        "========================================" | Out-File $LogFile -Append

        # Clean stale CMake cache if generator or platform changed
        $CacheFile = Join-Path $BuildDir "CMakeCache.txt"
        if (Test-Path $CacheFile) {
            $NeedClean = $false
            $CachedGen = (Select-String -Path $CacheFile -Pattern "^CMAKE_GENERATOR:INTERNAL=(.*)" |
                Select-Object -First 1).Matches.Groups[1].Value
            if ($CachedGen -and $CachedGen -ne $CMakeGenerator) {
                "  Generator changed ($CachedGen -> $CMakeGenerator), cleaning cache..." | Out-File $LogFile -Append
                $NeedClean = $true
            }
            $CachedPlatform = (Select-String -Path $CacheFile -Pattern "^PICO_PLATFORM:.*=(.*)" |
                Select-Object -First 1).Matches.Groups[1].Value
            $ExpectedPlatform = if ($Target -in @("MURM_P1","ZERO")) { "rp2040" } else { "rp2350-arm-s" }
            if ($CachedPlatform -and $CachedPlatform -ne $ExpectedPlatform) {
                "  Platform changed ($CachedPlatform -> $ExpectedPlatform), cleaning cache..." | Out-File $LogFile -Append
                $NeedClean = $true
            }
            if ($NeedClean) {
                Remove-Item $CacheFile -Force
                $CMakeFilesDir = Join-Path $BuildDir "CMakeFiles"
                if (Test-Path $CMakeFilesDir) { Remove-Item $CMakeFilesDir -Recurse -Force }
            }
        }

        if (-not (Test-Path $BuildDir)) { New-Item -ItemType Directory -Path $BuildDir | Out-Null }

        switch ($Target) {
            "MURM_P1" { $TargetFlags = @("-DMURM=ON") }
            "MURM_P2" { $TargetFlags = @("-DMURM=ON", "-DMURM_P2=ON") }
            default    { $TargetFlags = @("-D${Target}=ON") }
        }
        if ($Display -eq "TFT_ILI9341") {
            $TargetFlags += @("-DTFT=ON", "-DILI9341=ON", "-DVGA_HDMI=OFF")
        } elseif ($Display -eq "SOFTTV") {
            $TargetFlags += @("-DSOFTTV=ON", "-DVGA_HDMI=OFF")
        }

        $CMakeArgs = @(
            "-B", $BuildDir, "-S", $ScriptDir,
            "-G", $CMakeGenerator
        ) + $TargetFlags + $CCacheArgs + @(
            "-DCMAKE_BUILD_TYPE=$BuildType"
        )
        if ($CMakeMakeProgram) {
            $CMakeArgs += "-DCMAKE_MAKE_PROGRAM=$CMakeMakeProgram"
        }

        & cmake @CMakeArgs *>> $LogFile
        $ok = ($LASTEXITCODE -eq 0)
        if ($ok) {
            & cmake --build $BuildDir --parallel $JobsPerBuild *>> $LogFile
            $ok = ($LASTEXITCODE -eq 0)
        }
    } catch {
        "EXCEPTION: $_" | Out-File $LogFile -Append
        $ok = $false
    }

    $Elapsed = [int]((Get-Date) - $Start).TotalSeconds
    [pscustomobject]@{
        Target  = $Target
        Display = $Display
        Label   = $Label
        Ok      = $ok
        Seconds = $Elapsed
        Log     = $LogFile
    }
}

$StartTotal = Get-Date

# Parallel execution: PS7+ uses ForEach-Object -Parallel; older PS uses ThreadJob/Start-Job
$Results = @()
$IsPS7 = $PSVersionTable.PSVersion.Major -ge 7

if ($IsPS7) {
    $Results = $BuildPairs | ForEach-Object -ThrottleLimit $MaxParallel -Parallel {
        $WorkerSB = [ScriptBlock]::Create($using:Worker)
        & $WorkerSB $_ $using:ScriptDir $using:BuildType $using:CMakeGenerator `
            $using:CMakeMakeProgram $using:CCacheArgs $using:JobsPerBuild $using:LogDir
    }
} else {
    # Fallback: bounded pool via Start-Job
    $Running = @()
    $Queue = [System.Collections.Queue]::new()
    foreach ($P in $BuildPairs) { $Queue.Enqueue($P) }

    while ($Queue.Count -gt 0 -or $Running.Count -gt 0) {
        while ($Running.Count -lt $MaxParallel -and $Queue.Count -gt 0) {
            $P = $Queue.Dequeue()
            $Job = Start-Job -ScriptBlock $Worker -ArgumentList `
                $P, $ScriptDir, $BuildType, $CMakeGenerator, $CMakeMakeProgram, `
                $CCacheArgs, $JobsPerBuild, $LogDir
            $Running += $Job
        }
        $Finished = Wait-Job -Job $Running -Any
        $Results += Receive-Job $Finished
        Remove-Job $Finished
        $Running = $Running | Where-Object { $_.Id -ne $Finished.Id }
    }
}

# Print per-pair results
foreach ($R in $Results) {
    if ($R.Ok) {
        Write-Host ("[OK]   {0}:{1} ({2}s)" -f $R.Target, $R.Display, $R.Seconds)
    } else {
        Write-Host ("[FAIL] {0}:{1} ({2}s)  see {3}" -f $R.Target, $R.Display, $R.Seconds, $R.Log) -ForegroundColor Red
    }
}

$TotalSec = [int]((Get-Date) - $StartTotal).TotalSeconds
Write-Host ""
Write-Host "========================================"
Write-Host " Build summary (${TotalSec}s total)"
Write-Host "========================================"

$Succeeded = @($Results | Where-Object { $_.Ok })
$Failed    = @($Results | Where-Object { -not $_.Ok })

if ($Succeeded.Count -gt 0) {
    Write-Host "Succeeded ($($Succeeded.Count)):"
    foreach ($R in $Succeeded) {
        $BuildDir = Get-BuildDir $R.Target $R.Display
        Write-Host ("  {0}:{1} ({2}s)" -f $R.Target, $R.Display, $R.Seconds)
        Get-ChildItem -Path $BuildDir -Filter "*.uf2" -Recurse -ErrorAction SilentlyContinue | ForEach-Object {
            Copy-Item $_.FullName -Destination $OutputDir -Force
            Write-Host "    -> $($_.Name)"
        }
    }
    Write-Host ""
    Write-Host "All firmware files copied to: $OutputDir\"
}

if ($Failed.Count -gt 0) {
    Write-Host ""
    Write-Host "Failed ($($Failed.Count)):" -ForegroundColor Red
    foreach ($R in $Failed) {
        Write-Host ("  {0}:{1}  (see {2})" -f $R.Target, $R.Display, $R.Log) -ForegroundColor Red
    }
    exit 1
}

if ($HaveCCache) {
    Write-Host ""
    try { & ccache -s | Select-String -Pattern 'cache hit|cache miss|cache size' } catch {}
}

Write-Host ""
Write-Host "All builds completed successfully!"
