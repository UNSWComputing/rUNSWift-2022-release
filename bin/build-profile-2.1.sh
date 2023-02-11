#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

cd ${RUNSWIFT_CHECKOUT_DIR}
cd $(readlink -f build-profile-$CTC_VERSION_2_1)
make "$@"
