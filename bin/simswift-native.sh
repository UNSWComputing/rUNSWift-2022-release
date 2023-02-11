#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

setup_simswift

"$RUNSWIFT_CHECKOUT_DIR/build-relwithdebinfo-native/robot/runswift" --simulation "$@"
