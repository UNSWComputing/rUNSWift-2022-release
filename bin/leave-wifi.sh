#!/usr/bin/env bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

for r in ${!robots[@]}; do
    ssh $r sudo bin/changeField.py R && myecho $r done &
done
