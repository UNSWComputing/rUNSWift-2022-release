#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

DATE=`date +%F`
DEFAULT_DIR=$DATE-practice

function usage {
# same usage format as getopt itself, but with optional arguments
cat << USAGE

Usage:
 nao-extract.sh <robots>
 nao-extract.sh [options] <robots>

Extract logs from nao.

Options:
 -d,      --disconnect-wifi  disconnect wifi
 -l,      --logs             only extract logs
 -r,      --regions          only extract regions
 -u[dir], --upload=[dir]     upload to runswift2:logs/<dir>.
                             <dir> is "$DEFAULT_DIR" by default
 -w,      --whistles         only extract whistles

 -h, --help     display this help and exit

For more details see jayen.
USAGE
  exit $LINENO
}

# Set default flag values
FLAG_DISCONNECT_WIFI=
FLAG_LOGS=
FLAG_REGIONS=
FLAG_UPLOAD=
FLAG_UPLOAD_DIR=
FLAG_WHISTLES=

FLAG_ONLY=0

# Get options and store
OPTS=`getopt -o dlru::wh --long disconnect-wifi,logs,regions,upload::,whistles,help -- "$@"`
eval set -- "$OPTS"
while true ; do
  case "$1" in
    -d|--disconnect-wifi ) FLAG_DISCONNECT_WIFI=1; shift 1;;
    -l|--logs            ) FLAG_LOGS=1; FLAG_ONLY=1; shift 1;;
    -r|--regions         ) FLAG_REGIONS=1; FLAG_ONLY=1; shift 1;;
    -u|--upload          ) FLAG_UPLOAD=1; FLAG_UPLOAD_DIR="$2"; shift 2;;
    -w|--whistles        ) FLAG_WHISTLES=1; FLAG_ONLY=1; shift 1;;
    -h|--help) usage;;
    --) shift; break;;
    *) echo "Type -h or --help for instructions."; exit $LINENO;;
  esac
done

if [[ $# -eq 0 ]]; then
    mywarning I think you forgot to list robots
fi

for robot in "$@"; do
    myecho $robot

    if [[ "$FLAG_LOGS" = 1 ]] || [[ $FLAG_ONLY -eq 0 ]]; then
        myecho "extracting logs"
        mkdir -p "$RUNSWIFT_CHECKOUT_DIR/logs/$DATE"
        RSYNC_CONCISE $robot:/var/volatile/runswift/ "$RUNSWIFT_CHECKOUT_DIR/logs/$DATE/$robot/" ||
        mywarning check output for error
        RSYNC_CONCISE $robot:output_logs.txt "$RUNSWIFT_CHECKOUT_DIR/logs/$DATE/$robot/" ||
        mywarning check output for error
        RSYNC_CONCISE $robot:/tmp/runswift.std???.txt "$RUNSWIFT_CHECKOUT_DIR/logs/$DATE/$robot/" ||
        mywarning check output for error
    fi

    if [[ "$FLAG_REGIONS" = 1 ]] || [[ $FLAG_ONLY -eq 0 ]]; then
        myecho "extracting regions"
        myerror "TODO"
    fi

    if [[ "$FLAG_WHISTLES" = 1 ]] || [[ $FLAG_ONLY -eq 0 ]]; then
        myecho "extracting whistles"
        mkdir -p "$RUNSWIFT_CHECKOUT_DIR/test/audio/$DATE"
        RSYNC_CONCISE $robot:whistle/heard_whistles/ "$RUNSWIFT_CHECKOUT_DIR/test/audio/$DATE/$robot/" ||
        mywarning check output for error
    fi

    if [[ "$FLAG_DISCONNECT_WIFI" = 1 ]]; then
        myecho "disconnecting wifi"
        $SSH -tt $robot sudo /home/nao/bin/changeField.py R
    fi
done

if [[ "$FLAG_UPLOAD" = 1 ]]; then
    myecho "uploading logs"
    if [[ ! -n "$FLAG_UPLOAD_DIR" ]]; then
        FLAG_UPLOAD_DIR=$DEFAULT_DIR
    fi
    RSYNC_CONCISE "$RUNSWIFT_CHECKOUT_DIR/logs/$DATE/" "runswift@runswift2.cse.unsw.edu.au:/home/runswift/logs/$FLAG_UPLOAD_DIR/"
    RSYNC_CONCISE "$RUNSWIFT_CHECKOUT_DIR/test/audio/$DATE/" "runswift@runswift2.cse.unsw.edu.au:/home/runswift/logs/$FLAG_UPLOAD_DIR/"
fi
