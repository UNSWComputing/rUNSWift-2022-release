#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

cd ${RUNSWIFT_CHECKOUT_DIR}
cd $(readlink -f build-relwithdebinfo-$CTC_VERSION_2_8)
cmake -L . | grep FULL_DEB_INFO:BOOL=OFF || cmake -DFULL_DEB_INFO=OFF .
make "$@"
