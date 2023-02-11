#!/bin/bash
CTC_DIR=@CTC_DIR@
if [[ -n "$CTC_DIR" ]]; then
  #Needed for 18.04, need to double check if it breaks on 14.04 - if we even care
  export LD_LIBRARY_PATH="$CTC_DIR/../sysroot_legacy/usr/lib:$CTC_DIR/.."
fi
GDB_CMDS="$RUNSWIFT_CHECKOUT_DIR/image/home/nao/data/gdb_cmds.txt"
gdb -x "$GDB_CMDS" --return-child-result --quiet --args `dirname $0`/vatnao.bin "$@"
