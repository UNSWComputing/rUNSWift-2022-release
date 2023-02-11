#!/bin/bash

REALPATH=`realpath "$0"`
BIN_DIR=`dirname "$REALPATH"`
source "$BIN_DIR/source.sh"

# Set up ssh_config
mkdir -p ~/.ssh
chmod 700 ~/.ssh
[[ -f ~/.ssh/config ]] || touch ~/.ssh/config
chmod 600 ~/.ssh/config
# it's ok if old ones are there.  ssh seems to use rules in
# both.  if both have the same rule, the first takes precedence
if ! grep -q "Host ${!robots[*]}" ~/.ssh/config ; then (
  echo
  echo "Host ${!robots[*]}"
  echo "  Hostname %h.local"
  echo "  CheckHostIP no"
  echo "  User nao"
  echo "  StrictHostKeyChecking no"
) >> ~/.ssh/config
fi

# SSH keys
aptinstall openssh-client

# If one has a private key only, skip this part. We assume private-key only means you are using the runswift VM which
# has pull access to GitHub but should not otherwise be used. Once a new team member is onboarded, they should install
# linux bare metal, but they can run `ssh-keygen` manually if they wish to keep using the VM.
if ! ssh-keygen -l -f ~/.ssh/id_rsa &> /dev/null || ssh-keygen -l -f ~/.ssh/id_rsa.pub &> /dev/null; then
  ssh-keygen -l -f ~/.ssh/id_rsa.pub &> /dev/null || ssh-keygen -f ~/.ssh/id_rsa -N '' -q
  if ! grep -qf ~/.ssh/id_rsa.pub "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys; then
    echo >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
    echo "# $(git config user.name)'s key" >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
    cat ~/.ssh/id_rsa.pub >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
  fi
fi

# Set up local copy of pip packages needed on robots
aptinstall python-pip
if [[ ! -f "$RUNSWIFT_CHECKOUT_DIR/softwares/pip/msgpack-$MSGPACK_VERSION-cp27-cp27m-manylinux1_i686.whl" ]]; then
  pip download msgpack==$MSGPACK_VERSION --dest "$RUNSWIFT_CHECKOUT_DIR/softwares/pip" --platform manylinux1_i686 --python-version 27 --implementation cp --abi cp27m --only-binary=:all:
fi

# run setup-flite.sh manually if this doesn't work
if [[ ! -f "$RUNSWIFT_CHECKOUT_DIR/image/home/nao/2.8/bin/flite" ]]; then
    aptinstall pv
    if [[ ! -f "$RUNSWIFT_CHECKOUT_DIR/softwares/flite.tar.gz" ]]; then
        myecho "Downloading pre-built flite..."
        wget https://github.com/UNSWComputing/rUNSWift-assets/releases/download/v2019.1/flite.tar.gz --directory-prefix=$RUNSWIFT_CHECKOUT_DIR/softwares/
    fi
    mkdir -p "$RUNSWIFT_CHECKOUT_DIR/image/home/nao/2.8"
    pv "$RUNSWIFT_CHECKOUT_DIR/softwares/flite.tar.gz" | tar --extract --file - --directory "$RUNSWIFT_CHECKOUT_DIR/image/home/nao/2.8" --gz
fi

# setup ubuntu locally for the robot
mkdir -p $RUNSWIFT_CHECKOUT_DIR/image/data/2.8
cd $RUNSWIFT_CHECKOUT_DIR/image/data/2.8
aptinstall debootstrap
if [[ ! -d ubuntu-18.04 ]]; then
    sudo debootstrap --arch=amd64 bionic ubuntu-18.04 http://mirror.aarnet.edu.au/ubuntu/
    # for the gpu
    sudo chroot ubuntu-18.04 apt-get -qq install libxfixes3 libdrm-intel1 libgl1 libegl1 libsm6
fi
#mkdir -p $RUNSWIFT_CHECKOUT_DIR/image/data/2.1
#cd $RUNSWIFT_CHECKOUT_DIR/image/data/2.1
#if [[ ! -d ubuntu-18.04 ]]; then
#    sudo debootstrap --arch=i386  bionic ubuntu-18.04 http://mirror.aarnet.edu.au/ubuntu/
#fi
