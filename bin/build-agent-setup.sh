#!/bin/bash

# this script isn't used right now.  but it's here in case we want to build
# runswift with a non-ctc process, or if we want to copy this for building
# runswift

# WARNING:
## qibuild writes to the CTC_DIR so you can't share the CTC_DIR between
## a docker host and container, using different users or different paths.
## you may have wonky errors like:
##   fatal error: stddef.h: No such file or directory
##   -bash: $CTC_DIR/yocto-sdk/share/cmake/toolchain/linux64_cross_x86/../../../../sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-gcc: Permission denied

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

setupbash
setupctc
setupdocker
aptinstall python-pip cmake

CTC_DIR="$RUNSWIFT_CHECKOUT_DIR"/softwares/ctc-linux64-atom-$CTC_VERSION_2_8

myecho installing qibuild
pip -q install qibuild --user
PATH=$PATH:$HOME/.local/bin
myecho 'setting up ctc with executables (?)'
qitoolchain create cross-atom $CTC_DIR/toolchain.xml > /dev/null
qibuild add-config cross-atom --toolchain cross-atom

cd $RUNSWIFT_CHECKOUT_DIR/robot/libagent
# for testing
rm -rf build-cross-atom .qi
qibuild init
qibuild configure -c cross-atom > /dev/null
cd build-cross-atom
make
