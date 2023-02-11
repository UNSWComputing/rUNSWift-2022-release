#!/bin/bash
CTC_DIR=@CTC_DIR@
if [[ -n "$CTC_DIR" ]]; then
  export QT_PLUGIN_PATH=$CTC_DIR/../sysroot_legacy/usr/lib/qt4/plugins/
  LD_LIBRARY_PATH="$CTC_DIR/../sysroot_legacy/usr/lib/qt4/"
#Needed for 18.04, need to double check if it breaks on 14.04 - if we even care
  LD_LIBRARY_PATH="$LD_LIBRARY_PATH;$CTC_DIR/../sysroot_legacy/usr/lib:$CTC_DIR/.."
fi
GDB_CMDS="$RUNSWIFT_CHECKOUT_DIR/image/home/nao/data/gdb_cmds.txt"
gdb -silent -ex "set env LD_LIBRARY_PATH $LD_LIBRARY_PATH" -x "$GDB_CMDS" --args `dirname $0`/offnao.bin "$@"
