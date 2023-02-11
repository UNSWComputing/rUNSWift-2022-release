#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

# use this for gmon.out, which can be read by gprof
[[ -d ${RUNSWIFT_CHECKOUT_DIR}/build-profile ]] || setup-build-profile.sh
${RUNSWIFT_CHECKOUT_DIR}/build-profile/utils/offnao/offnao "$@"

# use this for callgrind.out.$pid, which can be read by kcachegrind
#${RUNSWIFT_CHECKOUT_DIR}/build-relwithdebinfo/utils/offnao/cgoffnao "$@"
