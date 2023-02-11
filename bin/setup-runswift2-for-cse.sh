#!/usr/bin/env bash

# This file sets up a compiled tarball of our executables on runswift2's webserver so new team
# members can run our executables on CSE.  Only offnao and sim_swiftstart have been tested.

cd /tmp
git clone --depth=1 git@github.com:UNSWComputing/rUNSWift.git runswift.jayen

runswift.jayen/bin/setup-build.sh
runswift.jayen/bin/setup-simulation.sh
patchelf --set-rpath $(patchelf --print-rpath runswift.jayen/build-relwithdebinfo-2.1.4.13/robot/runswift | sed 's@/tmp/runswift.jayen@$ORIGIN/../..@g') runswift.jayen/build-relwithdebinfo-2.1.4.13/robot/runswift
patchelf --set-rpath $(patchelf --print-rpath runswift.jayen/build-relwithdebinfo-2.1.4.13/utils/offnao/offnao.bin | sed 's@/tmp/runswift.jayen@$ORIGIN/../../..@g') runswift.jayen/build-relwithdebinfo-2.1.4.13/utils/offnao/offnao.bin

rm -rf runswift.jayen/build-relwithdebinfo-2.8.1.33 runswift.jayen/softwares/ctc-linux64-atom-2.8.1.33
rm -rf runswift.jayen/softwares/*.{zip,gz}
rm -rf runswift.jayen/softwares/flite
rm -rf runswift.jayen/.git

## 7z did not work well.  did not store things properly
#7z a -mx=9 -myx=9 -mqs=on -mmt=$(nproc) runswift.7z ./runswift.jayen/.??* ./runswift.jayen/*
#sudo mv runswift.7z /var/www/html/

## just a quick speed test, and also to get the size
#tar --create --file - --directory=runswift.jayen . | pv > runswift.tar
#3.49GiB 0:00:51 [69.8MiB/s] [                                             <=>  ]

## another speed test
#tar --create --file - --directory=runswift.jayen . | pv -s 3743406080 | pigz > runswift.tar.gz
#3.49GiB 0:01:19 [44.9MiB/s] [================================>] 100%

tar --create --file - --directory=runswift.jayen . | pv -s 3743406080 | pigz > runswift.tar.gz
sudo mv runswift.tar.gz /var/www/html/
