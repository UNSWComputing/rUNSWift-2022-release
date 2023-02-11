#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

cd ${RUNSWIFT_CHECKOUT_DIR}
bin/nao_sync -b build-relwithdebinfo-$CTC_VERSION_2_8 -v 2.8 "$@"
