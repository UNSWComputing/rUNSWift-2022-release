#!/usr/bin/env bash
# this script sets up a CSE machine for TEMPORARY use

DEST_DIR=/tmp/runswift.$USER

cd /tmp
mkdir -p $DEST_DIR

## 7z did not work well.  did not store things properly
#curl -O runswift2/runswift.7z
#7z e -o$DEST_DIR runswift.7z

curl -O runswift2/runswift.tar.gz
pv runswift.tar.gz | tar --extract --file - --directory $DEST_DIR --gz

source $DEST_DIR/bin/source.sh
myecho "You should be all set now"
myecho "  To open the simulator, use $DEST_DIR/bin/sim_swiftstart"
myecho "  To open a recording, use $DEST_DIR/bin/offnao"
myecho
myecho "Once you are ready to do more, contact robocup.spl@cse.unsw.edu.au for access"
