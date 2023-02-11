#!/bin/bash

# Sets up venue-specific things like static IP addresses

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

function usage {
    echo "usage: setup-venue.sh [-hs] robot [robot]..."
    echo "	-h --help      	Get this helpful message"
    echo "	-s --skipconfig	Skip configuration"
    exit $LINENO
}

# Definitions
CONFIG_DIR="$RUNSWIFT_CHECKOUT_DIR/bin"
CONFIG_FILE="$CONFIG_DIR/setup-venue.config.sh"
RUNSWIFT_CFG="$RUNSWIFT_CHECKOUT_DIR/image/home/nao/data/runswift.cfg"

# Set default flag values
FLAG_SKIPCONFIG=0

# Get options and store

OPTS=`getopt -o hs --long help,skipconfig -- "$@"`
eval set -- "$OPTS"
while true ; do
  case "$1" in
    -h|--help) usage;;
    -s|--skipconfig) FLAG_SKIPCONFIG=1; shift 1;;
    --) shift; break;;
    *) echo "Type -h or --help for instructions."; exit $LINENO;;
  esac
done

if [[ $# -eq 0 ]]; then
    myerror I think you forgot to list robots
    exit $LINENO
fi

# Clear the window
clear

# Ensure the configuration directory exists
mkdir -p "$CONFIG_DIR"

if [ $FLAG_SKIPCONFIG -ne 1 ]
then
  INFO=""
  yesno="y"
  if [ -f "$CONFIG_FILE" ]
  then
    INFO=`cat "$CONFIG_FILE"`
  fi
  if [ "$INFO" == "" ]
  then
    myerror "No configuration file exists!  Check the git history and try to restore $CONFIG_FILE"
    exit $LINENO
  else
    echo "Please read the following settings."
    echo "==============================================================="
    echo "$INFO"
    echo "==============================================================="
    echo "Are the following configuration settings OK? [Y/n]"
    read yesno
    if [[ "$yesno" != [yY]* ]] && [[ -n "$yesno" ]]
    then
      nano "$CONFIG_FILE" # &> /dev/null
      INFO=`cat "$CONFIG_FILE"`
      echo "Using the following"
      echo "==============================================================="
      echo "$INFO"
      echo "==============================================================="
    fi
  fi
  echo "==============================================================="
fi
source "$CONFIG_FILE"

myecho "Setting runswiftwireless for static IPs, ok?  [Y/n]"
read yesno
if [[ "$yesno" != [yY]* ]] && [[ -n "$yesno" ]]; then
    true # this is just here as we have a standard for checking the non-default answer
else
    if grep transmitter_address "$RUNSWIFT_CFG"; then
        # replace the broadcast address in runswift.cfg
        sed -i "/transmitter_address/s/.*/transmitter_address=$broadcast/" "$RUNSWIFT_CFG"
    else
        myerror "add transmitter_address to [network] section of runswift.cfg"
        exit $LINENO
    fi

    count=0
    for robot in "$@"; do
        percentage=$(echo $(($count * 100 / $#)))
        progress "updateWlanSetup.py $robot" $percentage
        "$RUNSWIFT_CHECKOUT_DIR/utils/wifitools/updateWlanSetup.py" --gateway $gateway --netmask $netmask --prefix $prefix $robot
        count=$(expr $count + 1)
    done
fi

# need the `git status` because somehow the sed above causes diff-index to think it's changed and git status to unthink it
if (cd "$RUNSWIFT_CHECKOUT_DIR" && git status > /dev/null && git diff-index --quiet HEAD "$RUNSWIFT_CFG"); then
    myecho "It looks like you're good to go, but please nao_sync if the broadcast address has changed"
else
    mywarning "It looks like you need to nao_sync as the broadcast address has changed, and also commit to git"
fi
