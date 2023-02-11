#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

export LD_LIBRARY_PATH=$RUNSWIFT_CHECKOUT_DIR/softwares/ctc-linux64-atom-$CTC_VERSION_2_8/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib/

${RUNSWIFT_CHECKOUT_DIR}/build-relwithdebinfo-$CTC_VERSION_2_8/utils/state-estimation-simulator/state-estimation-simulator.bin "$@"
