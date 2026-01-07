#!/bin/bash

export PICO_SDK_PATH=$(cd "$(dirname "$0")" && pwd)/../../pico-sdk
export PICOTOOL_FETCH_FROM_GIT_PATH=$(cd "$(dirname "$0")" && pwd)/../../picotool

cmake -DCMAKE_BUILD_TYPE=MinSizeRel -DPICO_COPY_TO_RAM=0 -DPICO_BOARD=pico2 ..
make -j10 all
