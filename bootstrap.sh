#!/bin/sh

rm -rf build
mkdir build
cd build
cmake -GNinja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain.cmake -DTOOLCHAIN_PATH=/opt/arm ..
