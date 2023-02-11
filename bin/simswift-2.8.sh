#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

setup_simswift

LD_LIBRARY_PATH="$RUNSWIFT_CHECKOUT_DIR/softwares/ctc-linux64-atom-$CTC_VERSION_2_8/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib/"
LD_LIBRARY_PATH="$LD_LIBRARY_PATH" "$RUNSWIFT_CHECKOUT_DIR/build-relwithdebinfo-$CTC_VERSION_2_8/robot/runswift" --simulation "$@"
