#!/bin/bash

# jayen isn't a fan of `sudo pip install`, so please use `aptinstall` where
# possible.  see flake8 below for an example.  if you absolutely must use `sudo
# pip install`, make a function like `aptinstall`

# TODO: move non-build stuff out of here (e.g., ssh, game controller, pip for nao_sync -s)

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

echo RUNSWIFT_CHECKOUT_DIR is $RUNSWIFT_CHECKOUT_DIR

setupdocker
aptinstall git

# Set up git
cat << USER_CONFIG
If the user info is incorrect, please configure it like:
  git config user.name Jayen
  git config user.email jayen@cse.unsw.edu.au
USER_CONFIG
cd "$RUNSWIFT_CHECKOUT_DIR"
echo -e 'Your user name: \033[33;1m' $(git config user.name) '\033[0m'
echo -e 'Your email:     \033[33;1m' $(git config user.email) '\033[0m'

setupbash

aptinstall wget unzip
cd "$RUNSWIFT_CHECKOUT_DIR"

# Set up Python code autoformatter
aptinstall python3-pip
tempvirtualenvinstallblack

# Set up Python flake8 linter
aptinstall python-flake8

# Set up git pre-commit hook
if [[ ! -L .git/hooks/pre-commit ]]; then
  myecho "Creating .git/hooks/pre-commit symlink..."
  ln -sf "$RUNSWIFT_CHECKOUT_DIR/.githooks/pre-commit" .git/hooks/pre-commit
fi

########### Toolchain ##########
setupctc
setupnative

# for nao os 2.1 (2.8 already has protobuf 2.6.1)
# use protobuf as it's way more future proof than boost serialization
(
  if [[ ! -x "$RUNSWIFT_CHECKOUT_DIR/softwares/protobuf-2.6.1/bin/protoc" ]]; then
    myecho Downloading/extracting/compiling protobuf for Nao OS 2.1
    cd "$RUNSWIFT_CHECKOUT_DIR/softwares"
    # s3 seems to have a weird issue with --content-disposition and
    # either of --continue or --timestamping, so we explicitly disable it
    # https://github.com/UNSWComputing/rUNSWift/pull/1861#discussion_r263960572
    wget --content-disposition=off https://github.com/protocolbuffers/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.bz2
    [[ -d protobuf-2.6.1 ]] || tar xf protobuf-2.6.1.tar.bz2
    cd protobuf-2.6.1
    export PATH="$RUNSWIFT_CHECKOUT_DIR/softwares/ctc-linux64-atom-$CTC_VERSION_2_1/cross/bin:$PATH"
    [[ -f config.status ]] || ./configure --prefix=$RUNSWIFT_CHECKOUT_DIR/softwares/protobuf-2.6.1 --host=i686-aldebaran-linux-gnu --quiet --enable-silent-rules
    make --quiet install V=0 CXXFLAGS=-w > /dev/null
  fi
)

############ Building ###########
aptinstall cmake patchelf
myecho Generating Makefiles and doing the initial build
for CTC_VERSION in "${CTC_VERSIONS[@]}"; do
    TOOLCHAIN_FILE="$RUNSWIFT_CHECKOUT_DIR"/toolchain-${CTC_VERSION%.*.*}.cmake
    for i in relwithdebinfo; do
        cd "$RUNSWIFT_CHECKOUT_DIR"
        BUILD_DIR=build-$i-$CTC_VERSION
        mkdir -p $BUILD_DIR
        cd $(readlink -f $BUILD_DIR)
        echo "CMAKE!!!"
        cmake --debug-trycompile "$RUNSWIFT_CHECKOUT_DIR" -DBoost_NO_BOOST_CMAKE=1 -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE} -DCMAKE_BUILD_TYPE=$i -DCMAKE_MAKE_PROGRAM=make
        if [[ ${CTC_VERSION} = ${CTC_VERSION_2_1} ]]; then
            # run cmake again to switch sysroot from legacy to 2.1 (for qt apps like offnao)
            cmake .
        fi
    done
done
# Build!
#only builds for one CTC.  change it by changing CTC_VERSIONS
for i in relwithdebinfo; do
    cd "$RUNSWIFT_CHECKOUT_DIR"
    # we used to not have version numbers in build dirs, then aldebaran released ctc 2.8 without support for old robots
    if [[ -e build-$i ]] && [[ ! -L build-$i ]]; then
        mv build-$i build-$i.old
    fi
    ln -sfn build-$i-$CTC_VERSION build-$i
    cd $(readlink -f build-$i)
    echo "MAKE!!!"
    make
done

# Adding the offnao patch for ubuntu versions 16.04 and above
#   s3 seems to have a weird issue with --content-disposition and
#   either of --continue or --timestamping, so we explicitly disable it
#   https://github.com/UNSWComputing/rUNSWift/pull/1861#discussion_r263960572
mv $RUNSWIFT_CHECKOUT_DIR/softwares/sysroot_legacy/usr/lib/libGL.so.1 $RUNSWIFT_CHECKOUT_DIR/softwares/sysroot_legacy/usr/lib/libGL.so.1.backup
wget --content-disposition=off https://github.com/UNSWComputing/rUNSWift-assets/releases/download/v2017.1/libGL.so.1 --directory-prefix=$RUNSWIFT_CHECKOUT_DIR/softwares/sysroot_legacy/usr/lib


echo
echo All done! To build, type build-relwithdebinfo.sh to compile
echo

# Finish
myecho Please close all shells.  Only new shells will have RUNSWIFT_CHECKOUT_DIR set to $RUNSWIFT_CHECKOUT_DIR
myecho 'Alternatively, type `source ~/.runswift.bash` in existing shells.'
echo
