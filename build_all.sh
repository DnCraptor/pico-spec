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
ALL_TARGETS="MURM MURM2 PICO_PC PICO_DV ZERO ZERO2"

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

for TARGET in $TARGETS; do
    BUILD_DIR="$SCRIPT_DIR/build-$TARGET"

    echo "========================================"
    echo " Building: $TARGET"
    echo " Build dir: $BUILD_DIR"
    echo "========================================"

    # Clean stale CMake cache if generator changed
    if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
        CACHED_GEN=$(grep -m1 'CMAKE_GENERATOR:INTERNAL=' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
        if [ -n "$CACHED_GEN" ] && [ "$CACHED_GEN" != "$CMAKE_GENERATOR" ]; then
            echo "  Generator changed ($CACHED_GEN -> $CMAKE_GENERATOR), cleaning cache..."
            rm -rf "$BUILD_DIR/CMakeCache.txt" "$BUILD_DIR/CMakeFiles"
        fi
    fi

    mkdir -p "$BUILD_DIR"

    CMAKE_ARGS=(
        -B "$BUILD_DIR" -S "$SCRIPT_DIR"
        -G "$CMAKE_GENERATOR"
        -D"${TARGET}"=ON
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    )
    if [ -n "$CMAKE_MAKE_PROGRAM" ]; then
        CMAKE_ARGS+=(-DCMAKE_MAKE_PROGRAM="$CMAKE_MAKE_PROGRAM")
    fi

    if cmake "${CMAKE_ARGS[@]}" \
      && cmake --build "$BUILD_DIR" --parallel "$PARALLEL_JOBS"; then
        SUCCEEDED="$SUCCEEDED $TARGET"
        echo ""
        echo "[OK] $TARGET build succeeded"
    else
        FAILED="$FAILED $TARGET"
        echo ""
        echo "[FAIL] $TARGET build failed"
    fi
    echo ""
done

echo "========================================"
echo " Build summary"
echo "========================================"

if [ -n "$SUCCEEDED" ]; then
    echo "Succeeded:$SUCCEEDED"
    for TARGET in $SUCCEEDED; do
        for UF2 in $(find "$SCRIPT_DIR/build-$TARGET" -name "*.uf2" 2>/dev/null); do
            cp "$UF2" "$OUTPUT_DIR/"
            echo "  $(basename "$UF2")"
        done
    done
    echo ""
    echo "All .uf2 files copied to: $OUTPUT_DIR/"
fi

if [ -n "$FAILED" ]; then
    echo "Failed:$FAILED"
    exit 1
fi

echo ""
echo "All builds completed successfully!"
