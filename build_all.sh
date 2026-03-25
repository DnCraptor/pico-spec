#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_TYPE="${BUILD_TYPE:-MinSizeRel}"
PARALLEL_JOBS="${JOBS:-$(nproc)}"

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

# All available targets
ALL_TARGETS="MURM_P1 MURM_P2 MURM2 PICO_PC PICO_DV ZERO ZERO2"

# Parse arguments: pass target names to build specific ones, or nothing for all
if [ $# -gt 0 ]; then
    TARGETS="$*"
else
    TARGETS="$ALL_TARGETS"
fi

echo "=== pico-spec multi-target build ==="
echo "Targets: $TARGETS"
echo "Build type: $BUILD_TYPE"
echo "Generator: $CMAKE_GENERATOR"
echo "Parallel jobs: $PARALLEL_JOBS"
echo ""

FAILED=""
SUCCEEDED=""
OUTPUT_DIR="$SCRIPT_DIR/firmware"
mkdir -p "$OUTPUT_DIR"

# # Expand ZERO and ZERO2 into two variants (252MHz and 378MHz)
# EXPANDED_TARGETS=""
# for TARGET in $TARGETS; do
#     if [ "$TARGET" = "ZERO2" ]; then
#         EXPANDED_TARGETS="$EXPANDED_TARGETS ZERO2@252 ZERO2@378"
#     elif [ "$TARGET" = "ZERO" ]; then
#         EXPANDED_TARGETS="$EXPANDED_TARGETS ZERO@252 ZERO@378"
#     else
#         EXPANDED_TARGETS="$EXPANDED_TARGETS $TARGET"
#     fi
# done
# TARGETS="$EXPANDED_TARGETS"

# Clean build directories before building
for ENTRY in $TARGETS; do
    TARGET="${ENTRY%@*}"
    SUFFIX="${ENTRY#*@}"
    if [ "$TARGET" = "$SUFFIX" ]; then
        BUILD_DIR="$SCRIPT_DIR/build-$TARGET"
    else
        BUILD_DIR="$SCRIPT_DIR/build-${TARGET}-${SUFFIX}"
    fi
    if [ -d "$BUILD_DIR" ]; then
        echo "Cleaning $BUILD_DIR ..."
        rm -rf "$BUILD_DIR"
    fi
done
echo ""

for ENTRY in $TARGETS; do
    TARGET="${ENTRY%@*}"
    MHZ_OVERRIDE="${ENTRY#*@}"
    if [ "$TARGET" = "$MHZ_OVERRIDE" ]; then
        MHZ_OVERRIDE=""
        BUILD_DIR="$SCRIPT_DIR/build-$TARGET"
        LABEL="$TARGET"
    else
        BUILD_DIR="$SCRIPT_DIR/build-${TARGET}-${MHZ_OVERRIDE}"
        LABEL="${TARGET} (${MHZ_OVERRIDE}MHz)"
    fi

    echo "========================================"
    echo " Building: $LABEL"
    echo " Build dir: $BUILD_DIR"
    echo "========================================"

    # Clean stale CMake cache if generator or platform changed
    if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
        CACHED_GEN=$(grep -m1 'CMAKE_GENERATOR:INTERNAL=' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
        CACHED_PLATFORM=$(grep -m1 'PICO_PLATFORM:' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
        NEED_CLEAN=false
        if [ -n "$CACHED_GEN" ] && [ "$CACHED_GEN" != "$CMAKE_GENERATOR" ]; then
            echo "  Generator changed ($CACHED_GEN -> $CMAKE_GENERATOR), cleaning cache..."
            NEED_CLEAN=true
        fi
        if [ -n "$CACHED_PLATFORM" ]; then
            case "$TARGET" in
                MURM_P1|ZERO) EXPECTED_PLATFORM="rp2040" ;;
                *) EXPECTED_PLATFORM="rp2350-arm-s" ;;
            esac
            if [ "$CACHED_PLATFORM" != "$EXPECTED_PLATFORM" ]; then
                echo "  Platform changed ($CACHED_PLATFORM -> $EXPECTED_PLATFORM), cleaning cache..."
                NEED_CLEAN=true
            fi
        fi
        if $NEED_CLEAN; then
            rm -rf "$BUILD_DIR/CMakeCache.txt" "$BUILD_DIR/CMakeFiles"
        fi
    fi

    mkdir -p "$BUILD_DIR"

    # Map target name to CMake flags
    case "$TARGET" in
        MURM_P1) TARGET_FLAGS=(-DMURM=ON) ;;
        MURM_P2) TARGET_FLAGS=(-DMURM=ON -DMURM_P2=ON) ;;
        *)       TARGET_FLAGS=(-D"${TARGET}"=ON) ;;
    esac

    # # Add CPU_MHZ override if specified
    # if [ -n "$MHZ_OVERRIDE" ]; then
    #     TARGET_FLAGS+=(-DCPU_MHZ_OVERRIDE="$MHZ_OVERRIDE")
    # fi

    CMAKE_ARGS=(
        -B "$BUILD_DIR" -S "$SCRIPT_DIR"
        -G "$CMAKE_GENERATOR"
        "${TARGET_FLAGS[@]}"
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    )
    if [ -n "$CMAKE_MAKE_PROGRAM" ]; then
        CMAKE_ARGS+=(-DCMAKE_MAKE_PROGRAM="$CMAKE_MAKE_PROGRAM")
    fi

    if cmake "${CMAKE_ARGS[@]}" \
      && cmake --build "$BUILD_DIR" --parallel "$PARALLEL_JOBS"; then
        SUCCEEDED="$SUCCEEDED $ENTRY"
        echo ""
        echo "[OK] $LABEL build succeeded"
    else
        FAILED="$FAILED $ENTRY"
        echo ""
        echo "[FAIL] $LABEL build failed"
    fi
    echo ""
done

echo "========================================"
echo " Build summary"
echo "========================================"

if [ -n "$SUCCEEDED" ]; then
    echo "Succeeded:$SUCCEEDED"
    for ENTRY in $SUCCEEDED; do
        TARGET="${ENTRY%@*}"
        SUFFIX="${ENTRY#*@}"
        if [ "$TARGET" = "$SUFFIX" ]; then
            BUILD_DIR="$SCRIPT_DIR/build-$TARGET"
        else
            BUILD_DIR="$SCRIPT_DIR/build-${TARGET}-${SUFFIX}"
        fi
        for FW in $(find "$BUILD_DIR" -name "*.uf2" 2>/dev/null); do
            cp "$FW" "$OUTPUT_DIR/"
            echo "  $(basename "$FW")"
        done
    done
    echo ""
    echo "All firmware files copied to: $OUTPUT_DIR/"
fi

if [ -n "$FAILED" ]; then
    echo "Failed:$FAILED"
    exit 1
fi

echo ""
echo "All builds completed successfully!"
