#!/bin/bash
CTC_DIR=@CTC_DIR@
if [[ -n "$CTC_DIR" ]]; then
  #Needed for 18.04, need to double check if it breaks on 14.04 - if we even care
  export LD_LIBRARY_PATH="$CTC_DIR/../sysroot_legacy/usr/lib:$CTC_DIR/.."
fi
`dirname $0`/vatnao-legacy.bin "$@"
