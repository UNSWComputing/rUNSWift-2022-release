#!/bin/bash
set -e
set -u

# use this most of the time
valgrind --error-limit=no --track-origins=yes --suppressions=/home/nao/data/valgrind.supp `which runswift` "$@"

# use this when you want to edit the supression file
#valgrind --error-limit=no --track-origins=yes --suppressions=/home/nao/data/valgrind.supp --gen-suppressions=all --log-file=/tmp/valgrind.txt `which runswift` "$@"

# you need two terminals when you want to connect valgrind and
# gdb, so this has not been ported over from simswift_valgrind.sh
