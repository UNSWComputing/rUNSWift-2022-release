#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

setup_simswift
GDB_CMDS="$RUNSWIFT_CHECKOUT_DIR/image/home/nao/data/gdb_cmds.txt"
RUNSWIFT_EXEC="${RUNSWIFT_CHECKOUT_DIR}/build-relwithdebinfo-$CTC_VERSION_2_8/robot/runswift"
gdb -x "$GDB_CMDS" --return-child-result --quiet --args "$RUNSWIFT_EXEC" --simulation "$@"
