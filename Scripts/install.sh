#!/bin/bash -e
#
# This is a build script for Co-Fusion.
#
# To install all dependencies as well, invoke:
#   ./build.sh --fresh-install
#
#   which will create:
#   - ./deps/densecrf
#   - ./deps/gSLICr
#   - ./deps/OpenNI2
#   - ./deps/Pangolin
#   - ./deps/opencv-3.1.0
#   - ./deps/opencv_contrib
#   - ./deps/boost (unless env BOOST_ROOT is defined)

cd $(dirname `realpath $0`)/..
mkdir -p deps
cd deps

# build gSLICr, see: http://www.robots.ox.ac.uk/~victor/gslicr/
cd gSLICr
git pull
mkdir -p build
cd build
cmake \
  -DOpenCV_DIR="${OpenCV_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCUDA_HOST_COMPILER=/usr/bin/gcc-5 \
  -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -D_FORCE_INLINES" \
  ..
make -j8
cd ../..

# build Co-Fusion
cd ..
mkdir -p build
cd build
cmake \
  -DBOOST_ROOT="${BOOST_ROOT}" \
  -DOpenCV_DIR="${OpenCV_DIR}" \
  -DPangolin_DIR="${Pangolin_DIR}" \
  ..
make -j8
cd ..
