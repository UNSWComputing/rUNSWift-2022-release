#!/bin/bash

REALPATH=`realpath "$0"`
BIN_DIR=`dirname "$REALPATH"`
source "$BIN_DIR/source.sh"

${RUNSWIFT_CHECKOUT_DIR}/build-relwithdebinfo-native/utils/offnao/offnao.bin "$@"
