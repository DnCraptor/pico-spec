#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_TYPE="${BUILD_TYPE:-MinSizeRel}"
NPROC="$(nproc)"

# Parallelism: MAX_PARALLEL targets build concurrently, each with JOBS_PER_BUILD threads.
# Total threads ≈ MAX_PARALLEL * JOBS_PER_BUILD; aim near nproc.
MAX_PARALLEL="${MAX_PARALLEL:-3}"
JOBS_PER_BUILD="${JOBS_PER_BUILD:-$(( (NPROC + MAX_PARALLEL - 1) / MAX_PARALLEL ))}"

# Clean behavior: by default keep build dirs for incremental rebuilds.
# Pass --clean to wipe them first. CMake cache invalidation for generator/platform
# changes still runs per-target below.
CLEAN=false

# Parse flags then positional targets
POSITIONAL=()
while [ $# -gt 0 ]; do
    case "$1" in
        --clean) CLEAN=true; shift ;;
        -j)       JOBS_PER_BUILD="$2"; shift 2 ;;
        -p)       MAX_PARALLEL="$2"; shift 2 ;;
        --)       shift; while [ $# -gt 0 ]; do POSITIONAL+=("$1"); shift; done ;;
        -h|--help)
            cat <<EOF
Usage: $0 [--clean] [-j JOBS_PER_BUILD] [-p MAX_PARALLEL] [TARGETS...]

Env vars: BUILD_TYPE, MAX_PARALLEL, JOBS_PER_BUILD, CMAKE_GENERATOR
Targets:  MURM_P1 MURM_P2 MURM2_P1 MURM2_P2 PICO_PC PICO_DV ZERO ZERO2 (default: all)
EOF
            exit 0 ;;
        *) POSITIONAL+=("$1"); shift ;;
    esac
done
set -- "${POSITIONAL[@]}"

# Auto-detect Ninja from Pico SDK or system PATH
if [ -z "$CMAKE_GENERATOR" ]; then
    PICO_NINJA="$HOME/.pico-sdk/ninja/v1.12.1/ninja"
    if [ -x "$PICO_NINJA" ]; then
        CMAKE_GENERATOR="Ninja"
        CMAKE_MAKE_PROGRAM="$PICO_NINJA"
    elif command -v ninja &>/dev/null; then
        CMAKE_GENERATOR="Ninja"
    elif command -v make &>/dev/null; then
        CMAKE_GENERATOR="Unix Makefiles"
    else
        echo "Error: no build tool found (ninja or make)"
        exit 1
    fi
fi

# ccache: use if available, shared across all per-target build dirs
CCACHE_ARGS=()
if command -v ccache &>/dev/null; then
    CCACHE_ARGS=(
        -DCMAKE_C_COMPILER_LAUNCHER=ccache
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
        -DCMAKE_ASM_COMPILER_LAUNCHER=ccache
    )
    export CCACHE_DIR="${CCACHE_DIR:-$HOME/.ccache}"
    # Enlarge cache; arm-none-eabi objects are small but numerous across 13 builds.
    ccache --max-size=5G >/dev/null 2>&1 || true
    HAVE_CCACHE=1
else
    HAVE_CCACHE=0
fi

# All available targets
ALL_TARGETS="MURM_P1 MURM_P2 MURM2_P2 PICO_PC PICO_DV ZERO ZERO2"

# Targets that support TFT+ILI9341 display variant
TFT_TARGETS="MURM_P1 MURM_P2 MURM2_P2"

# Targets that support TFT+ST7789 display variant
TFT_ST_TARGETS="MURM2_P1"

# Targets that support SOFTTV display variant
SOFTTV_TARGETS="MURM_P1 MURM_P2 MURM2_P2"

# Parse arguments: pass target names to build specific ones, or nothing for all
if [ $# -gt 0 ]; then
    TARGETS="$*"
else
    TARGETS="$ALL_TARGETS"
fi

# Build list of (target, display) pairs
BUILD_PAIRS=()
for TARGET in $TARGETS; do
    BUILD_PAIRS+=("${TARGET}:VGA_HDMI")
    for TFT_T in $TFT_TARGETS; do
        if [ "$TARGET" = "$TFT_T" ]; then
            BUILD_PAIRS+=("${TARGET}:TFT_ILI9341")
            break
        fi
    done
    for TFT_ST_T in $TFT_ST_TARGETS; do
        if [ "$TARGET" = "$TFT_ST_T" ]; then
            BUILD_PAIRS+=("${TARGET}:TFT_ST7789")
            break
        fi
    done
    for STV_T in $SOFTTV_TARGETS; do
        if [ "$TARGET" = "$STV_T" ]; then
            BUILD_PAIRS+=("${TARGET}:SOFTTV")
            break
        fi
    done
done

echo "=== pico-spec multi-target build ==="
echo "Targets:          $TARGETS"
echo "Pairs:            ${#BUILD_PAIRS[@]}"
echo "Build type:       $BUILD_TYPE"
echo "Generator:        $CMAKE_GENERATOR"
echo "Parallel targets: $MAX_PARALLEL  (jobs per target: $JOBS_PER_BUILD, nproc=$NPROC)"
echo "ccache:           $( [ $HAVE_CCACHE = 1 ] && echo enabled || echo "not installed (apt install ccache → ~2-5x faster rebuilds)" )"
echo "Clean first:      $CLEAN"
echo ""

OUTPUT_DIR="$SCRIPT_DIR/firmware"
LOG_DIR="$SCRIPT_DIR/build-logs"
mkdir -p "$OUTPUT_DIR" "$LOG_DIR"

build_dir_for() {
    local target="$1" display="$2"
    if [ "$display" = "VGA_HDMI" ]; then
        echo "$SCRIPT_DIR/build-$target"
    else
        echo "$SCRIPT_DIR/build-${target}-${display}"
    fi
}

# Optional clean
if $CLEAN; then
    for PAIR in "${BUILD_PAIRS[@]}"; do
        TARGET="${PAIR%%:*}"; DISPLAY="${PAIR##*:}"
        BUILD_DIR="$(build_dir_for "$TARGET" "$DISPLAY")"
        if [ -d "$BUILD_DIR" ]; then
            echo "Cleaning $BUILD_DIR ..."
            rm -rf "$BUILD_DIR"
        fi
    done
    echo ""
fi

# Build one (target, display) pair. Writes log to $LOG_DIR/<label>.log.
# Exits 0 on success, non-zero on failure.
build_one() {
    local pair="$1"
    local target="${pair%%:*}"
    local display="${pair##*:}"
    local build_dir label log
    build_dir="$(build_dir_for "$target" "$display")"
    if [ "$display" = "VGA_HDMI" ]; then
        label="$target (VGA_HDMI)"
    else
        label="$target ($display)"
    fi
    log="$LOG_DIR/${target}-${display}.log"

    {
        echo "========================================"
        echo " Building: $label"
        echo " Build dir: $build_dir"
        echo "========================================"

        # Clean stale CMake cache if generator or platform changed
        if [ -f "$build_dir/CMakeCache.txt" ]; then
            local cached_gen cached_platform need_clean=false expected_platform
            cached_gen=$(grep -m1 'CMAKE_GENERATOR:INTERNAL=' "$build_dir/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
            cached_platform=$(grep -m1 'PICO_PLATFORM:' "$build_dir/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
            if [ -n "$cached_gen" ] && [ "$cached_gen" != "$CMAKE_GENERATOR" ]; then
                echo "  Generator changed ($cached_gen -> $CMAKE_GENERATOR), cleaning cache..."
                need_clean=true
            fi
            if [ -n "$cached_platform" ]; then
                case "$target" in
                    MURM_P1|MURM2_P1|ZERO) expected_platform="rp2040" ;;
                    *) expected_platform="rp2350-arm-s" ;;
                esac
                if [ "$cached_platform" != "$expected_platform" ]; then
                    echo "  Platform changed ($cached_platform -> $expected_platform), cleaning cache..."
                    need_clean=true
                fi
            fi
            if $need_clean; then
                rm -rf "$build_dir/CMakeCache.txt" "$build_dir/CMakeFiles"
            fi
        fi

        mkdir -p "$build_dir"

        local target_flags=()
        case "$target" in
            MURM_P1)  target_flags=(-DMURM=ON) ;;
            MURM_P2)  target_flags=(-DMURM=ON -DMURM_P2=ON) ;;
            MURM2_P1) target_flags=(-DMURM2=ON) ;;
            MURM2_P2) target_flags=(-DMURM2=ON -DMURM2_P2=ON) ;;
            *)        target_flags=(-D"${target}"=ON) ;;
        esac
        if [ "$display" = "TFT_ILI9341" ]; then
            target_flags+=(-DTFT=ON -DILI9341=ON -DVGA_HDMI=OFF)
        elif [ "$display" = "TFT_ST7789" ]; then
            target_flags+=(-DTFT=ON -DST7789=ON -DVGA_HDMI=OFF)
        elif [ "$display" = "SOFTTV" ]; then
            target_flags+=(-DSOFTTV=ON -DVGA_HDMI=OFF)
        fi

        local cmake_args=(
            -B "$build_dir" -S "$SCRIPT_DIR"
            -G "$CMAKE_GENERATOR"
            "${target_flags[@]}"
            "${CCACHE_ARGS[@]}"
            -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
        )
        if [ -n "$CMAKE_MAKE_PROGRAM" ]; then
            cmake_args+=(-DCMAKE_MAKE_PROGRAM="$CMAKE_MAKE_PROGRAM")
        fi

        cmake "${cmake_args[@]}" \
          && cmake --build "$build_dir" --parallel "$JOBS_PER_BUILD"
    } >"$log" 2>&1
}

# Run builds with bounded concurrency via xargs -P.
# xargs spawns one subshell per pair, at most MAX_PARALLEL concurrently.
export -f build_one build_dir_for
export SCRIPT_DIR BUILD_TYPE CMAKE_GENERATOR CMAKE_MAKE_PROGRAM LOG_DIR JOBS_PER_BUILD
export CCACHE_ARGS_STR="${CCACHE_ARGS[*]}"
# Re-export CCACHE_ARGS as a string and reconstruct in subshell (arrays don't export cleanly).
build_one_wrapper() {
    local pair="$1"
    IFS=' ' read -r -a CCACHE_ARGS <<<"$CCACHE_ARGS_STR"
    local start=$SECONDS
    if build_one "$pair"; then
        echo "[OK]   $pair ($((SECONDS - start))s)"
    else
        echo "[FAIL] $pair ($((SECONDS - start))s)"
        return 1
    fi
}
export -f build_one_wrapper

START_TIME=$SECONDS
RESULTS_FILE="$(mktemp)"
trap 'rm -f "$RESULTS_FILE"' EXIT

printf '%s\n' "${BUILD_PAIRS[@]}" \
    | xargs -I{} -P "$MAX_PARALLEL" -n 1 \
        bash -c 'build_one_wrapper "$@"' _ {} \
    | tee "$RESULTS_FILE" || true

TOTAL_TIME=$((SECONDS - START_TIME))

echo ""
echo "========================================"
echo " Build summary (${TOTAL_TIME}s total)"
echo "========================================"

SUCCEEDED=()
FAILED=()
while IFS= read -r line; do
    case "$line" in
        "[OK]"*)   SUCCEEDED+=("${line#\[OK\]   }") ;;
        "[FAIL]"*) FAILED+=("${line#\[FAIL\] }") ;;
    esac
done <"$RESULTS_FILE"

if [ ${#SUCCEEDED[@]} -gt 0 ]; then
    echo "Succeeded (${#SUCCEEDED[@]}):"
    for ENTRY in "${SUCCEEDED[@]}"; do
        PAIR="${ENTRY%% (*}"
        TARGET="${PAIR%%:*}"; DISPLAY="${PAIR##*:}"
        BUILD_DIR="$(build_dir_for "$TARGET" "$DISPLAY")"
        echo "  $ENTRY"
        while IFS= read -r FW; do
            cp "$FW" "$OUTPUT_DIR/"
            echo "    -> $(basename "$FW")"
        done < <(find "$BUILD_DIR" -name "*.uf2" 2>/dev/null)
    done
    echo ""
    echo "All firmware files copied to: $OUTPUT_DIR/"
fi

if [ ${#FAILED[@]} -gt 0 ]; then
    echo ""
    echo "Failed (${#FAILED[@]}):"
    for ENTRY in "${FAILED[@]}"; do
        PAIR="${ENTRY%% (*}"
        TARGET="${PAIR%%:*}"; DISPLAY="${PAIR##*:}"
        echo "  $ENTRY    (see $LOG_DIR/${TARGET}-${DISPLAY}.log)"
    done
    exit 1
fi

if [ $HAVE_CCACHE = 1 ]; then
    echo ""
    ccache -s 2>/dev/null | grep -E 'cache hit|cache miss|cache size' || true
fi

echo ""
echo "All builds completed successfully!"
