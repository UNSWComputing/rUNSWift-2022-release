# use this file in other rUNSWift bash scripts like:
#   REALPATH=$(realpath "$0")
#   BIN_DIR=$(dirname "$REALPATH")
#   source "$BIN_DIR/source.sh"

set -e # bail out when something goes wrong rather than screwing up the nao
set -u

shopt -s expand_aliases
# don't use --timestamping as then it doesn't try to resume
alias wget='wget --continue --no-verbose --show-progress --progress=bar:force --connect-timeout=1 --tries=1 '

function myerror() {
    echo -n " [!] "       >&2
    echo -ne '\033[31;1m' >&2
    echo -n "$@"          >&2
    echo -e '\033[0m'     >&2
    echo                  >&2
}

function myecho() {
    echo -n " [+] "
    echo -ne '\033[32;1m'
    echo -n "$@"
    echo -e '\033[0m'
}

function mywarning() {
    echo -n " [?] "       >&2
    echo -ne '\033[33;1m' >&2
    echo -n "$@"          >&2
    echo -e '\033[0m'     >&2
    echo                  >&2
}

function progress {
  myecho $(printf "[%+3s%%]" $2) $1
}

function setupbash() {
    if ! grep -q "runswift.bash" ~/.bashrc ; then
        echo source ~/.runswift.bash >> ~/.bashrc
    fi
    echo "# Robocup stuff" > ~/.runswift.bash
    echo export RUNSWIFT_CHECKOUT_DIR=\"$RUNSWIFT_CHECKOUT_DIR\" >> ~/.runswift.bash
    echo export PATH=\"\$RUNSWIFT_CHECKOUT_DIR/bin:\$PATH\" >> ~/.runswift.bash

    # save this if we ran setupbash() before
    if [[ -v GH_TOKEN ]]; then
        echo export GH_TOKEN=\"$GH_TOKEN\" >> ~/.runswift.bash
    fi

    # so we have the environment setup for later in the setup-build process
    source ~/.runswift.bash

    if [[ x"$OLD_RCD" != x"$RUNSWIFT_CHECKOUT_DIR" ]]; then
        trap "myecho RUNSWIFT_CHECKOUT_DIR has changed from \'$OLD_RCD\' to \'$RUNSWIFT_CHECKOUT_DIR\'.  please \`source ~/.runswift.bash\` before fixing things manually" ERR
    fi
}

function setupctc() {
    mkdir -p "$RUNSWIFT_CHECKOUT_DIR"/softwares
    cd "$RUNSWIFT_CHECKOUT_DIR"/softwares

    ASSETS_LOCATION="https://github.com/UNSWComputing/rUNSWift-assets/releases/download/v2017.1/"
    BOOST_HEADERS=boostheaders.zip
    LIBUUID=libuuid.so.1.3.0

    aptinstall wget unzip

    for CTC_VERSION in "${CTC_VERSIONS[@]}"; do
        LINUX_CTC_ZIP=ctc-linux64-atom-$CTC_VERSION.zip
        if [ ! -f ${LINUX_CTC_ZIP} ]; then
            myecho "Downloading toolchain file: ${LINUX_CTC_ZIP}"
            echo "Setup failed: Please provide the toolchain zip file: $LINUX_CTC_ZIP in $RUNSWIFT_CHECKOUT_DIR/softwares"
            # Aldebaran should provide a direct download link !!!
            exit 1
        fi
        CTC_DIR="$RUNSWIFT_CHECKOUT_DIR"/softwares/ctc-linux64-atom-$CTC_VERSION
        [[ -d "$CTC_DIR" ]] || ( myecho Extracting cross toolchain ${LINUX_CTC_ZIP}, this may take a while... && unzip -q ${LINUX_CTC_ZIP} )

        if [[ " ${CTC_VERSION} " = " ${CTC_VERSION_2_1} " ]]; then
            if [ ! -f ${BOOST_HEADERS} ]; then
                echo "Downloading modified boost headers"
                wget ${ASSETS_LOCATION}${BOOST_HEADERS}
            fi
            BOOST_HEADER_DIR="$RUNSWIFT_CHECKOUT_DIR"/softwares/${LINUX_CTC_ZIP/.zip/}/boost/include/boost-1_55/boost/type_traits/detail/
            unzip -j -q -o ${BOOST_HEADERS} -d ${BOOST_HEADER_DIR}
            if [[ $(dpkg --print-architecture) == "i386" ]]; then
                # for running the Qt executables in sysroot_legacy
                # need :i386 in case you have the :amd64 version also installed
                aptinstall libc6:i386 zlib1g:i386 libstdc++6:i386 libgcc1:i386
            fi
            if [[ $(dpkg --print-architecture) == "amd64" ]]; then
                # for running the cross-compiled Qt executables in sysroot_legacy
                aptinstall libc6-i386 lib32z1 lib32stdc++6 lib32gcc1
            fi
        fi
    done

    if [ ! -f ${LIBUUID} ]; then
        echo "Downloading libuuid.so.1.3.0"
        wget ${ASSETS_LOCATION}${LIBUUID}
    fi

    ln -sf ${LIBUUID} libuuid.so.1

    if [[ " ${CTC_VERSIONS[@]} " =~ " ${CTC_VERSION_2_1} " ]]; then
        # boost libs used to come in two flavors: thread-unsafe (no -mt) &
        # thread-safe (-mt). they've only had one (thread-safe) version for a
        # long time but kept both filenames (-mt and no -mt) for a while for
        # ease of backward compatibility.  they made no -mt the default well
        # before 1.55 so i'm not sure why someone explicitly enabled it for the
        # ctc, nor why the ctc does not match the v5
        cd "$RUNSWIFT_CHECKOUT_DIR"/softwares/ctc-linux64-atom-$CTC_VERSION_2_1/boost/lib/
        for lib in libboost_*-mt-1_55.so.1.55.0; do
            ln -sf $lib ${lib/-mt-1_55.so.1.55.0/.so.1.55.0}
        done
        cd -
    fi

    if [[ " ${CTC_VERSIONS[@]} " =~ " ${CTC_VERSION_2_1} " ]]; then
        echo "Changing permission on ctc dir"
        chmod -R 755 "$RUNSWIFT_CHECKOUT_DIR"/softwares/ctc-linux64-atom-$CTC_VERSION_2_1/cross/bin
        chmod -R 755 "$RUNSWIFT_CHECKOUT_DIR"/softwares/ctc-linux64-atom-$CTC_VERSION_2_1/cross/i686-aldebaran-linux-gnu/bin
        chmod -R 755 "$RUNSWIFT_CHECKOUT_DIR"/softwares/ctc-linux64-atom-$CTC_VERSION_2_1/cross/libexec/
    fi

    # This is needed for the v6 stuff
    aptinstall python
    "$RUNSWIFT_CHECKOUT_DIR"/softwares/ctc-linux64-atom-$CTC_VERSION_2_8/yocto-sdk/relocate_qitoolchain.sh

    # Jayen's magic sauce
    SYSROOT_ARCHIVE="sysroot_legacy.tar.gz"
    if [ ! -f ${SYSROOT_ARCHIVE} ]; then
        myecho Downloading/extracting sysroot_legacy/usr, this may take a *long* time...
        wget ${ASSETS_LOCATION}${SYSROOT_ARCHIVE}
    fi
    if [ ! -e sysroot_legacy ]; then
        tar -zxf ${SYSROOT_ARCHIVE}
    fi

    INTEL_BIN=/opt/intel/oneapi/compiler/2021.1.2/linux/bin/ia32/
    if [[ ! -e ${INTEL_BIN} ]]; then
        mywarning 'Please install the intel compiler intel-oneapi-compiler-dpcpp-cpp-and-cpp-classic-32bit:i386 from https://software.intel.com/content/www/us/en/develop/articles/installing-intel-oneapi-toolkits-via-apt.html#aptpkg'
    fi
}

function setupnative() {
    aptinstall libboost-python-dev libboost-program-options-dev libboost-test-dev libqglviewer-dev-qt4 libqwt-dev libmsgpack-dev libprotobuf-dev libbz2-dev protobuf-compiler cmake g++ libpng-dev libjpeg-dev libboost-system-dev libboost-thread-dev libboost-serialization-dev libboost-regex-dev git
}

function aptinstall() {
    for package in "$@"; do
        if ! dpkg --status $package | grep -q 'Status: install ok installed'; then
            myecho installing $package
            # need to pipe to /dev/null for dpkg
            sudo apt-get -qq install $package > /dev/null
        fi
    done
}

# 1st argument: package name
# 2nd argument: url
# 3rd argument: filename
function dpkgurlinstall() {
    package="$1"
    url="$2"
    filename="$3"
    if ! dpkg --status $package | grep -q 'Status: install ok installed'; then
        wget $url
        myecho installing $package
        sudo dpkg --install $filename > /dev/null
    fi
}

# Can be replaced by "aptinstall python-black" when Ubuntu 20.04 LTS comes out
function tempvirtualenvinstallblack() {
    # Given so many supported Linux distributions, not sure which Python 3 we will get
    BLACK_VIRTUALENV_PATH="$RUNSWIFT_CHECKOUT_DIR/.virtualenvs/runswift-black-py3k"

    if [[ ! -x "$BLACK_VIRTUALENV_PATH/bin/black" ]]; then
      myecho "Installing black (a Python code autoformatter)"
      # need --upgrade in case the user has a broken virtualenv installed
      pip3 --quiet install --user --upgrade virtualenv
      python3 -m virtualenv "$BLACK_VIRTUALENV_PATH" --python="$(command -v python3)" --quiet
      source "$BLACK_VIRTUALENV_PATH/bin/activate"
      pip --quiet install black==19.10b0
      deactivate
    fi
}

# if in docker, make it like normal ubuntu (apt, sudo, etc)
function setupdocker() {
    # if root, assume docker
    if [[ `id -u` -eq 0 ]]; then
        export DEBIAN_FRONTEND=noninteractive
        # if sudo not found, update package list
        if ! apt-cache show sudo &> /dev/null; then
            myecho "resynchronizing the package index files from their sources (Updating software listings)..."
            apt-get -qq update
        fi
        # don't need this for the scripts since we have a sudo function, but install it in case someone types it
        aptinstall apt-utils sudo
    fi
}

function sudo() {
    if [[ `id -u` -eq 0 ]]; then
        "$@"
    else
        `which sudo` "$@"
    fi
}

function getosversion() {
    local name=$1
    if [[ -v "robots[$name]" ]]; then
        echo "${robots[$name]}"
    fi
}

function getmachinetype() {
    local MACHINE_TYPE=`uname -m`
    # in case the kernel is 64-bit but the system is 32-bit, use 32-bit for roboviz
    if [[ ${MACHINE_TYPE} == 'x86_64' && $(dpkg --print-architecture) == "i386" ]]; then
        MACHINE_TYPE='i586'
    fi
    echo "$MACHINE_TYPE"
}

function setup_simswift() {
    # setup a symlink so we save logs.  much easier than making debug.logpath relative to $RUNSWIFT_CHECKOUT_DIR
    mkdir -p "$RUNSWIFT_CHECKOUT_DIR/logs/localhost-simswift"
    if [[ ! -d /tmp/runswift ]]; then
        ln -sf "$RUNSWIFT_CHECKOUT_DIR/logs/localhost-simswift" /tmp/runswift
    fi
}

REALPATH=$(realpath "$BASH_SOURCE")
BIN_DIR=$(dirname "$REALPATH")
OLD_RCD="${RUNSWIFT_CHECKOUT_DIR-}"
# Allow to be run as either `cd bin;./setup-build.sh` OR `./bin/setup-build.sh`
export RUNSWIFT_CHECKOUT_DIR=`dirname "$BIN_DIR"`
export PATH="$RUNSWIFT_CHECKOUT_DIR/bin:$PATH"

declare -A robots
# https://github.com/UNSWComputing/rUNSWift/wiki/Setting-up-a-brand-new-robot#places-to-add-name
#v5s
for robot in robot1 robot2; do
    robots[$robot]=2.1
done
#2018 v6s
# https://github.com/UNSWComputing/rUNSWift/wiki/Setting-up-a-brand-new-robot#places-to-add-name
for robot in robot3 robot4; do
    robots[$robot]=2.8
done
#2019-05-28 v6s
# https://github.com/UNSWComputing/rUNSWift/wiki/Setting-up-a-brand-new-robot#places-to-add-name
for robot in robot5 robot6; do
    robots[$robot]=2.8
done

# TOOLCHAIN_FILE= expects these to be four numbers each
CTC_VERSION_2_1=2.1.4.13
CTC_VERSION_2_8=2.8.1.33

NPROC=1
if [[ "$OSTYPE" == "linux-gnu" ]]; then
    NPROC=$(nproc)
    MEM_KB=$(awk '/^MemTotal:/{print $2}' /proc/meminfo)
    # RobotModule.cpp takes 2GB to compile, so limit number of processors to use to 1 per 2GB MEM
    ((NPROC_FOR_MEM = MEM_KB / 2 / 1024 / 1024))
    if [[ $NPROC_FOR_MEM -lt $NPROC ]]; then
        NPROC=$NPROC_FOR_MEM
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    # Mac OSX
    NPROC=$(sysctl -n hw.ncpu)
fi

export MAKEFLAGS=-j$NPROC

# last one in this list gets set up
CTC_VERSIONS=($CTC_VERSION_2_1 $CTC_VERSION_2_8)

# reuse the same ssh connection
SSH="ssh -o ControlMaster=auto -o ControlPath=/tmp/control_%C -o ControlPersist=5s"

# rsync helpers (use aliases as they aren't affected by issues
# with spaces like shell variables and are easier than functions)
export RSYNC_RSH="$SSH" # so we don't have to specify it on the rsync command lines
alias RSYNC_CONCISE="rsync --archive --compress --partial --out-format='%i %n%L'"
alias SUDO_RSYNC_CONCISE="sudo rsync --archive --compress --partial --out-format='%i %n%L' --rsync-path='sudo rsync' --rsh='ssh -i $HOME/.ssh/id_rsa'"
alias RSYNC_VERBOSE="rsync --archive --compress --partial --progress"

# this shouldn't change often as the next version (v1.0.0) is for python >= 3
MSGPACK_VERSION=0.6.2
