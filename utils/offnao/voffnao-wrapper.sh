#!/bin/bash
CTC_DIR=@CTC_DIR@
if [[ -n "$CTC_DIR" ]]; then
  export QT_PLUGIN_PATH=$CTC_DIR/../sysroot_legacy/usr/lib/qt4/plugins/
  export LD_LIBRARY_PATH="$CTC_DIR/../sysroot_legacy/usr/lib/qt4/"
#Needed for 18.04, need to double check if it breaks on 14.04 - if we even care
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH;$CTC_DIR/../sysroot_legacy/usr/lib:$CTC_DIR/.."
fi

# use this most of the time
valgrind --track-origins=yes --suppressions="$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/data/valgrind.supp `dirname $0`/offnao.bin "$@"

# use this when you want to edit the supression file
#valgrind --track-origins=yes --suppressions="$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/data/valgrind.supp --gen-suppressions=all --log-file=/tmp/valgrind.txt `dirname $0`/offnao.bin "$@"

# use this when you want to connect valgrind and gdb
# you may have to stop using ctrl /
# be sure to clean up after yourself
#pkill -9 memcheck-x86-li
#sed 's/run/continue/' "$RUNSWIFT_CHECKOUT_DIR/image/home/nao/data/gdb_cmds.txt" > /tmp/gdb_cmds.txt
#(sleep 1; x-terminal-emulator -e gdb --quiet -ex "target remote | vgdb" -x /tmp/gdb_cmds.txt) &
#valgrind --error-limit=no --track-origins=yes --suppressions="$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/data/valgrind.supp --vgdb=yes --vgdb-error=0 `dirname $0`/offnao.bin "$@"
#pkill -9 memcheck-x86-li
