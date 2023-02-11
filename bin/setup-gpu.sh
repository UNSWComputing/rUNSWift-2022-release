#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

robot=$1

drivers=(
    acpi/video.ko
    char/agp/intel-gtt.ko
    gpu/drm/drm.ko
    gpu/drm/drm_kms_helper.ko
    gpu/drm/i915/i915.ko
    video/backlight/backlight.ko
    video/backlight/generic_bl.ko
    video/backlight/lcd.ko
)

# Jayen's magic sauce 2020
cd $RUNSWIFT_CHECKOUT_DIR/softwares
if [[ ! -d linux-aldebaran ]]; then
    git clone --depth=1 https://github.com/UNSWComputing/linux-aldebaran
fi
cd linux-aldebaran
if [[ ! -f arch/x86_64/boot/bzImage ]]; then
    (
        CCACHE_PATH=
        source "$RUNSWIFT_CHECKOUT_DIR/softwares/ctc-linux64-atom-$CTC_VERSION_2_8/yocto-sdk/environment-setup-core2-32-sbr-linux"
        make
    )
fi

# copy the compiled kernel and modules, overwriting the stock ones
$SSH $robot sudo mount -o remount,rw /
RSYNC_VERBOSE --rsync-path="sudo rsync" --copy-links arch/x86_64/boot/bzImage $robot:/boot/bzImage-4.4.86-rt99-aldebaran
cd drivers
RSYNC_CONCISE --rsync-path="sudo rsync" --relative ${drivers[@]} $robot:/lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/
$SSH $robot sudo depmod

# moment of truth (if the kernel boots)
$SSH $robot reboot

# moment of truth (if the video card kernel module loads)
while sleep 1; do
    if $SSH $robot sudo modprobe i915; then
        break
    fi
done

if [[ $(dpkg --print-architecture) == "amd64" ]]; then
    cd $RUNSWIFT_CHECKOUT_DIR/softwares
    #https://github.com/intel/beignet
    aptinstall cmake pkg-config python ocl-icd-dev libegl1-mesa-dev ocl-icd-opencl-dev libdrm-dev libxfixes-dev libxext-dev libtinfo-dev libedit-dev zlib1g-dev
    # because we manually install packages below, we need to specify some dependencies here
    aptinstall binfmt-support libobjc-6-dev
    # 3.6 from the instructions is old.  3.7 is supported, but still old.  Get 3.7 from the debian archives.
    for package in libllvm3.7 llvm-3.7-runtime llvm-3.7 llvm-3.7-dev libclang1-3.7 libclang-common-3.7-dev clang-3.7 libclang-3.7-dev; do
        dpkgurlinstall ${package} \
                       http://snapshot.debian.org/archive/debian/20170328T032449Z/pool/main/l/llvm-toolchain-3.7/${package}_3.7.1-3%2Bb2_amd64.deb \
                       ${package}_3.7.1-3+b2_amd64.deb
    done
    if [[ ! -d beignet ]]; then
        git clone --depth=1 https://github.com/intel/beignet
    fi
    mkdir -p beignet/build
    cd beignet/build
    if [[ ! -f CMakeCache.txt ]]; then
        # does not work with gcc 8.3
        CC=clang-3.7 CXX=clang++-3.7 cmake ..
    fi
    if [[ ! -f src/libcl.so ]]; then
        make
    fi
    if [[ ! -f utests/utest_run ]]; then
        make utest
    fi
#    # if you want to run tests locally.  jayen's T430s fails at compiler_subgroup_buffer_block_read_ui1
#    (
#        cd utests
#        source setenv.sh
#        ./utest_run
#    )
    if [[ ! -f Beignet-1.4-Linux.tar.gz ]]; then
        make package
    fi
    cd $RUNSWIFT_CHECKOUT_DIR/image/home/nao/2.8
    if [[ ! -d Beignet-1.4-Linux ]]; then
        tar xf $OLDPWD/Beignet-1.4-Linux.tar.gz
    fi
else
    mywarning not building beignet on your non-amd64 machine.  we can build this in the chroot in the future, but for now let someone else do it
fi

# copy to the robot
nao_sync $robot

# run test
RSYNC_CONCISE $RUNSWIFT_CHECKOUT_DIR/softwares/beignet/build /home/nao/2.8/Beignet-1.4-Linux/
$SSH $robot sudo mount --bind /home/ /data/2.8/ubuntu-18.04/home/
$SSH $robot sudo chroot /data/2.8/ubuntu-18.04 bash -c "cd /home/nao/2.8/Beignet-1.4-Linux/utests && sed -i 's@$RUNSWIFT_CHECKOUT_DIR/softwares/beignet@/home/nao/2.8/Beignet-1.4-Linux@' setenv.sh utest_run && source setenv.sh && ./utest_run"
