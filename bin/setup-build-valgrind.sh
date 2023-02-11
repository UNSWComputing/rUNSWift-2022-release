#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

# for simswift
if ! (dpkg --print-architecture; dpkg --print-foreign-architectures) | grep -q i386; then
    sudo dpkg --add-architecture i386
    myecho "resynchronizing the package index files from their sources (i386)"
    sudo apt-get -qq update
fi
aptinstall valgrind libc6-dbg:i386
# for nao os 2.8
# use subshell so we don't mess with the cwd or env
(
    # change working directory
    cd "$RUNSWIFT_CHECKOUT_DIR/softwares"
    # get it
    [[ -f valgrind-3.14.0.tar.bz2 ]] || wget http://www.valgrind.org/downloads/valgrind-3.14.0.tar.bz2
    # unzip it
    [[ -d valgrind-3.14.0 ]] || tar xf valgrind-3.14.0.tar.bz2
    # change dir
    cd valgrind-3.14.0
    # setup to compile for nao
    CCACHE_PATH=
    source "$RUNSWIFT_CHECKOUT_DIR/softwares/ctc-linux64-atom-$CTC_VERSION_2_8/yocto-sdk/environment-setup-core2-32-sbr-linux"
    # compile only 32-bit and set everything up for running from /home/nao
    [[ -f stamp-h1 ]] || ./configure --prefix=/home/nao/2.8 --enable-only32bit
    # install to /home/nao on the robot
    make install DESTDIR=$RUNSWIFT_CHECKOUT_DIR/image/home/nao/2.8
)
############ Building ###########
myecho Generating Makefiles and doing the initial build
for CTC_VERSION in "${CTC_VERSIONS[@]}"; do
    TOOLCHAIN_FILE="$RUNSWIFT_CHECKOUT_DIR"/toolchain-${CTC_VERSION%.*.*}.cmake
    for i in valgrind; do
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
for i in valgrind; do
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

myecho All done! To build, type build-valgrind-2.1.sh or build-valgrind-2.8.sh to compile
