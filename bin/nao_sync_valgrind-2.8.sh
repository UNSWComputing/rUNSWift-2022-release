#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

cd ${RUNSWIFT_CHECKOUT_DIR}
bin/nao_sync -b build-valgrind-$CTC_VERSION_2_8 -g -v 2.8 "$@"

echo -e "=================================================================="
echo -e "If you are now looking to run valgrind on the nao, run on the nao:"
echo
echo -e "\trunswift_valgrind.sh"
echo
echo -e "=================================================================="
