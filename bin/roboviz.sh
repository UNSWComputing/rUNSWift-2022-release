#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

# Get machine type (32bit / 64bit)
export MACHINE_TYPE=$(getmachinetype)
if [[ ${MACHINE_TYPE} == 'x86_64' ]]; then
    # 64-bit
    ROBOVIZ_PATH="$RUNSWIFT_CHECKOUT_DIR/softwares/roboviz/bin/linux-amd64/"
    PATH=/usr/lib/jvm/java-8-openjdk-amd64/jre/bin/:"$PATH"
else
    # 32-bit
    ROBOVIZ_PATH="$RUNSWIFT_CHECKOUT_DIR/softwares/roboviz/bin/linux-i586/"
    PATH=/usr/lib/jvm/java-8-openjdk-i386/jre/bin/:"$PATH"
fi
PATH="$PATH" "$ROBOVIZ_PATH/roboviz.sh" "$@"
