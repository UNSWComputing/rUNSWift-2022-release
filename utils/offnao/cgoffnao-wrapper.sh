#!/bin/bash
CTC_DIR=@CTC_DIR@
if [[ -n "$CTC_DIR" ]]; then
  export QT_PLUGIN_PATH=$CTC_DIR/../sysroot_legacy/usr/lib/qt4/plugins/
  export LD_LIBRARY_PATH="$CTC_DIR/../sysroot_legacy/usr/lib/qt4/"
#Needed for 18.04, need to double check if it breaks on 14.04 - if we even care
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH;$CTC_DIR/../sysroot_legacy/usr/lib:$CTC_DIR/.."
fi

valgrind --tool=callgrind --suppressions="$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/data/valgrind.supp `dirname $0`/offnao.bin "$@"
