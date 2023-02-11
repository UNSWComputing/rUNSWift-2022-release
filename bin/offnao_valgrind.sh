#!/bin/bash
[[ -d ${RUNSWIFT_CHECKOUT_DIR}/build-valgrind ]] || setup-build-valgrind.sh
${RUNSWIFT_CHECKOUT_DIR}/build-valgrind/utils/offnao/voffnao "$@"
