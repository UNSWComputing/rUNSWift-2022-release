#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

setup_simswift
[[ -d ${RUNSWIFT_CHECKOUT_DIR}/build-valgrind ]] || setup-build-valgrind.sh

RUNSWIFT_EXEC="${RUNSWIFT_CHECKOUT_DIR}/build-valgrind/robot/runswift"

# use this most of the time
valgrind --error-limit=no --track-origins=yes --suppressions="$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/data/valgrind.supp "$RUNSWIFT_EXEC" --simulation "$@"

# use this when you want to edit the supression file
#valgrind --error-limit=no --track-origins=yes --suppressions="$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/data/valgrind.supp --gen-suppressions=all --log-file=/tmp/valgrind.txt "$RUNSWIFT_EXEC" --simulation "$@"

# use this when you want to connect valgrind and gdb
# you may have to stop using ctrl /
# be sure to clean up after yourself
#pkill -9 memcheck-x86-li || true
#sed 's/run/continue/' "$RUNSWIFT_CHECKOUT_DIR/image/home/nao/data/gdb_cmds.txt" > /tmp/gdb_cmds.txt
#(sleep 5; x-terminal-emulator -e gdb --quiet -ex "target remote | vgdb" -x /tmp/gdb_cmds.txt) &
#valgrind --error-limit=no --track-origins=yes --suppressions="$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/data/valgrind.supp --vgdb=yes --vgdb-error=0 "$RUNSWIFT_EXEC" --simulation "$@"
#pkill -9 memcheck-x86-li || true
