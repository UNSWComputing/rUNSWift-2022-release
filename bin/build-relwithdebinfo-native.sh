#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

cd ${RUNSWIFT_CHECKOUT_DIR}
# extracted from setup-build.sh, so it's done on-demand
setupnative
CTC_VERSION=native
i=relwithdebinfo
BUILD_DIR=build-$i-$CTC_VERSION
mkdir -p $BUILD_DIR
cd $(readlink -f $BUILD_DIR)
if [[ ! -f Makefile ]]; then
    echo "CMAKE!!!"
    cmake "$RUNSWIFT_CHECKOUT_DIR" -DCMAKE_BUILD_TYPE=$i
fi
make "$@"
