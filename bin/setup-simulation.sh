#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

myecho This script downloads and installs the required software to run simswift.

cd "$RUNSWIFT_CHECKOUT_DIR/softwares"

# Check operating system
if [[ "$OSTYPE" != "linux-gnu" ]]; then
    myerror "simswift is only supported on Linux!"
    exit
fi

# Get machine type (32bit / 64bit)
export MACHINE_TYPE=$(getmachinetype)

# Download prerequisites
myecho "Downloading/installing simulation software pre-requisites..."
aptinstall g++ subversion cmake libfreetype6-dev libode-dev libsdl1.2-dev ruby ruby-dev libdevil-dev libboost-dev libboost-thread-dev libboost-regex-dev libboost-system-dev qt4-default libqt4-opengl-dev # requirements for simspark
aptinstall gcc-multilib g++-multilib
# roboviz doesn't work with java 11
# https://stackoverflow.com/questions/55847497/how-do-i-troubleshoot-inconsistency-detected-dl-lookup-c-111-java-result-12
aptinstall openjdk-8-jdk openjdk-8-jre
aptinstall python2.7

# Git clone modified roboviz / simspark
myecho "Cloning SimSpark and RoboViz (rUNSWift modification)"

if [[ ! -d roboviz ]]; then
    myecho "Shallow cloning modified roboviz"
    git clone --depth=1 git@github.com:UNSWComputing/RoboViz.git roboviz
    robovizupdated=true
else
    myecho "Pulling roboviz"
    robovizupdated=false
    if (cd roboviz && git pull) | grep -q -v 'Already up-to-date.'; then
        robovizupdated=true
    fi
fi

if [[ ! -d simspark ]]; then
    myecho "Shallow cloning modified simspark"
    git clone --depth=1 git@github.com:UNSWComputing/SimSpark.git simspark
    simsparkupdated=true
else
    myecho "Pulling simspark"
    simsparkupdated=false
    if (cd simspark && git pull) | grep -q -v 'Already up-to-date.'; then
        simsparkupdated=true
    fi
fi

if $simsparkupdated; then
    # Install simspark(spark and rcssserver3d)
    SPARK_INSTALL_PATH="$RUNSWIFT_CHECKOUT_DIR"/softwares/simspark/spark/installed
    RCSSSERVER3D_INSTALL_PATH="$RUNSWIFT_CHECKOUT_DIR"/softwares/simspark/rcssserver3d/installed
    myecho "Building and installing modified simspark..."
    myecho "Building simspark - spark"
    cd simspark/spark
    mkdir -p build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX="$SPARK_INSTALL_PATH" \
          -DCMAKE_INSTALL_RPATH="$SPARK_INSTALL_PATH"/lib/simspark \
          -DRCSSSERVER3D_INSTALL_PATH="$RCSSSERVER3D_INSTALL_PATH" \
          ..
    make install
    cd ../../../

    myecho "Building simspark - rcssserver3d"
    cd simspark/rcssserver3d
    mkdir -p build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=$RCSSSERVER3D_INSTALL_PATH -DCMAKE_PREFIX_PATH="$SPARK_INSTALL_PATH" ..
    make install
    cd ../../../
fi

if $robovizupdated; then
    # Install roboviz
    myecho "Building and installing modified roboviz..."
    if [[ ${MACHINE_TYPE} == 'x86_64' ]]; then
        PATH=/usr/lib/jvm/java-8-openjdk-amd64/bin/:"$PATH" ./roboviz/scripts/build-linux64.sh
    else
        PATH=/usr/lib/jvm/java-8-openjdk-i386/bin/:"$PATH" ./roboviz/scripts/build-linux32.sh
    fi
fi

# Prepare system to use simulation build

cd $RUNSWIFT_CHECKOUT_DIR/softwares

myecho "Preparing system for use of simulation build..."

# Apply CTC patch
if [[ ! -f ctc_patch.tar.gz ]]; then
    myecho "Downloading and installing CTC patch..."
    if ! wget http://robocup.web.cse.unsw.edu.au/2018/ctc_patch.tar.gz; then
        rsync -aP repository@runswift2.cse.unsw.edu.au:/var/www/html/simulation/ctc_patch.tar.gz .
    fi
fi
# ctc_patch.tar.gz/ctc_patch/sysroot_legacy => ./sysroot_legacy
tar -zxf ctc_patch.tar.gz ctc_patch/sysroot_legacy --strip-components=1
# Delete old files
rm -f sysroot_legacy/usr/lib/python2.7/random.py[co]

# Fix python bug - https://github.com/pypa/virtualenv/issues/410
if [[ ! -L /usr/lib/python2.7/_sysconfigdata_nd.py ]]; then
    myecho "Fixing ubuntu python bug..."
    sudo ln -s /usr/lib/python2.7/plat-*/_sysconfigdata_nd.py /usr/lib/python2.7/
fi


# Finish
 myecho "All done. You can start roboviz by typing 'roboviz.sh' or rcssserver by typing 'rcssserver3d'."
