#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

############ Building ###########
myecho Generating Makefiles and doing the initial build
for CTC_VERSION in "${CTC_VERSIONS[@]}"; do
    TOOLCHAIN_FILE="$RUNSWIFT_CHECKOUT_DIR"/toolchain-${CTC_VERSION%.*.*}.cmake
    for i in profile; do
        cd "$RUNSWIFT_CHECKOUT_DIR"
        BUILD_DIR=build-$i-$CTC_VERSION
        mkdir -p $BUILD_DIR
        cd $BUILD_DIR
        myecho "CMAKE!!!"
        cmake  --debug-trycompile .. -DBoost_NO_BOOST_CMAKE=1 -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE} -DCMAKE_BUILD_TYPE=$i -DCMAKE_MAKE_PROGRAM=make
        if [[ ${CTC_VERSION} = ${CTC_VERSION_2_1} ]]; then
            # run cmake again to switch sysroot from legacy to 2.1 (for qt apps like offnao)
            cmake .
        fi
    done
done
# Build!
#only builds for one CTC.  change it by changing CTC_VERSIONS
for i in profile; do
    cd "$RUNSWIFT_CHECKOUT_DIR"
    # we used to not have version numbers in build dirs, then aldebaran released ctc 2.8 without support for old robots
    if [[ -e build-$i ]] && [[ ! -L build-$i ]]; then
        mv build-$i build-$i.old
    fi
    ln -sfn build-$i-$CTC_VERSION build-$i
    cd $(readlink -f build-$i)
    myecho "MAKE!!!"
    make
done

myecho All done! To build, type build-profile-2.1.sh to compile
