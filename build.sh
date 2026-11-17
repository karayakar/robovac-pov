#!/bin/bash
# compiles the renderer host (the Metal shader is compiled at run time from src/render.metal)
set -e
cd "$(dirname "$0")"
SDK=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk   # the default 27.0 SDK does not link with this toolchain
[ -d "$SDK" ] || SDK=$(xcrun --show-sdk-path)
clang++ -std=c++17 -O2 -fobjc-arc -Wno-deprecated-declarations -isysroot "$SDK" src/main.mm -framework Metal -framework Foundation -framework CoreGraphics -o robovac
