#!/bin/bash

# for nao os 2.8
# use flite because naoqi waits 10 mins before loading ALTextToSpeech

REALPATH=`realpath "$0"`
BIN_DIR=`dirname "$REALPATH"`
source "$BIN_DIR/source.sh"

# change working directory
cd "$RUNSWIFT_CHECKOUT_DIR/softwares"
# get it
[[ -d flite ]] || git clone https://github.com/festvox/flite
# change dir
cd flite
# setup to compile for nao
CCACHE_PATH=
source "$RUNSWIFT_CHECKOUT_DIR/softwares/ctc-linux64-atom-$CTC_VERSION_2_8/yocto-sdk/environment-setup-core2-32-sbr-linux"
# set everything up for running from /home/nao
[[ -f config.status ]] || ./configure --prefix=/home/nao/2.8
# compile
[[ -f bin/flite_cmu_us_slt ]] || make
# get the voices
[[ -f voices/cmu_us_slt.flitevox ]] || make get_voices
# install
[[ -f $RUNSWIFT_CHECKOUT_DIR/image/home/nao/2.8/lib/libflite.a ]] || make install DESTDIR=$RUNSWIFT_CHECKOUT_DIR/image
# create tarball for upload
aptinstall pv pigz
[[ -f "$RUNSWIFT_CHECKOUT_DIR/softwares/flite.tar.gz" ]] || tar --create --file - --directory="$RUNSWIFT_CHECKOUT_DIR/image/home/nao/2.8" . | pv --size 100823040 | pigz > "$RUNSWIFT_CHECKOUT_DIR/softwares/flite.tar.gz"
# tell user to upload
myecho "Please upload $RUNSWIFT_CHECKOUT_DIR/softwares/flite.tar.gz to https://github.com/UNSWComputing/rUNSWift-assets/releases/edit/v2019.1"
