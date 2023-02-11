#!/bin/bash

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

function usage {
  echo "usage: setup-game.sh [-csb] [robot] [robot]..."
  echo "  -c --compile        Make/build before syncing"
  echo "  -s --skipconfig     Skip team/player number configuration"
  echo "  -b --behaviouronly  Do not do a -rad sync"
  echo "  -e --noexit         Do not exit early if can't connect to robot"
  echo "  -l --logging        Turn on logging flags on robots (logging will be"
  echo "                      enabled and cout and cerr will be piped to file)"
  exit 1
}

# Definitions
CONFIG_DIR="$RUNSWIFT_CHECKOUT_DIR/bin"
CONFIG_FILE="$CONFIG_DIR/setup-game.config.sh"

# Set default flag values
FLAG_BUILD=0
FLAG_SKIPCONFIG=0
FLAG_BEHAVIOURONLY=0
FLAG_NOEXIT=0
FLAG_LOGGING=0
FLAG_NO_WIFI=0
FLAG_DEBUG=0

# so it's easy to add more player numbers later
FIRST_PLAYER_NUM=1
LAST_PLAYER_NUM=10

# Get options and store

OPTS=`getopt -o csbelhfkd --long compile,skipconfig,behaviouronly,noexit,logging,fieldstay,keephosts,debug,help -- "$@"`
eval set -- "$OPTS"
while true ; do
  case "$1" in
    -c|--compile) FLAG_BUILD=1; shift 1;;
    -s|--skipconfig) FLAG_SKIPCONFIG=1; shift 1;;
    -d|--debug) FLAG_DEBUG=1; shift 1;;
    -b|--behaviouronly) FLAG_BEHAVIOURONLY=1; shift 1;;
    -e|--noexit) FLAG_NOEXIT=1; shift 1;;
    -l|--logging) FLAG_LOGGING=1; shift 1;;
    -f|--fieldstay) FLAG_NO_WIFI=1; shift 1;;
    -h|--help) usage;;
    --) shift; break;;
    *) echo "Type -h or --help for instructions."; exit 1;;
  esac
done

branch=$(cd $RUNSWIFT_CHECKOUT_DIR && git rev-parse --abbrev-ref HEAD)
if [[ "$branch" != master ]]; then
  mywarning "You are on branch $branch not master.  Do you wish to continue? [y/N]"
  read yesno
  if [[ "$yesno" != [nN]* ]] && [[ -n "$yesno" ]]; then
    true # this is just here as we have a standard for checking the non-default answer
  else
    exit $LINENO
  fi
fi

# Build if --nobuild was not set
if [ $FLAG_BUILD -eq 1 ]
then
  cd $RUNSWIFT_CHECKOUT_DIR/build-relwithdebinfo
  make
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
    echo "No configuration file exists."
    echo ""
    echo "Press any key to create the file..."
    read yesno
    cp "$CONFIG_DIR/setup-game.template.sh" $CONFIG_FILE
    editor "$CONFIG_FILE" # &> /dev/null
  else
    echo "Please read the following settings."
    echo "==============================================================="
    echo "$INFO"
    echo "==============================================================="
    echo "Are the following configuration settings OK? [Y/n]"
    read yesno
    if [[ "$yesno" != [yY]* ]] && [[ -n "$yesno" ]]
    then
      editor "$CONFIG_FILE" # &> /dev/null
      INFO=`cat "$CONFIG_FILE"`
      echo "Using the following"
      echo "==============================================================="
      echo "$INFO"
      echo "==============================================================="
    fi
  fi
  echo "==============================================================="
fi

progress "Updating configuration files" 0

declare -a players=()
declare -A playersTeam
declare -A playersNumb

progress "Checking available connections to robots" 1

source "$CONFIG_FILE"
error=0
for numb in `seq $FIRST_PLAYER_NUM $LAST_PLAYER_NUM`
do
  declare -n pns=p${numb}s
  if [[ ${#pns[@]} != ${#teams[@]} ]]
  then
    myerror "There are ${#teams[@]} teams and ${#pns[@]} player ${numb}'s"
    exit $LINENO
  fi
  for index in ${!teams[@]}
  do
    name=${pns[$index]}
    if [[ ! -n "$name" ]]
    then
      continue
    fi

    # if i specified robots on the command line, skip the other robots
    if [[ $# -gt 0 ]]; then
        if [[ ! " $@ " =~ " $name " ]]; then
            continue
        fi
    fi

    team=${teams[$index]}
    playersTeam[$name]=$team
    playersNumb[$name]=$numb
    players+=($name)
    if ! ssh "nao@$name.local" true &> /dev/null
    then
      error=1
      echo "Unable to ping $name.local"
    fi
  done
done

if [ $error -eq 1 ]
then
  if [ $FLAG_NOEXIT -ne 1 ]
  then
    echo
    myerror "Fatal error. Unable to connect to robots above."
    echo
    exit 1
  fi
fi

for name in "$@"; do
    if [[ ! " ${players[@]} " =~ " $name " ]]; then
        echo
        myerror $name not found in $CONFIG_FILE
        exit $LINENO
    fi
done

echo "Have you rebooted all the nao's? [Y/n]"
read yesno
if [[ "$yesno" != [yY]* ]] && [[ -n "$yesno" ]]
  then
  echo "Please do it properly"
  exit 1
fi

echo "Have you made sure the jersey is on the goalie properly? [Y/n]"
read yesno
if [[ "$yesno" != [yY]* ]] && [[ -n "$yesno" ]]
  then
  echo "Please do it properly"
  exit 1
fi

# during competition, or when the lab emulates competition conditions
# if [ $FLAG_NO_WIFI -eq 0 ]
# then
#   field_letter="runswift"
#   echo "Are you setting up on a competition field? [Y/n]"
#   read yesno
#   if [[ "$yesno" != [yY]* ]] && [[ -n "$yesno" ]]; then
#     true # this is just here as we have a standard for checking the non-default answer
#   else
#     field_letter=
#     while [[ "$field_letter" == "" ]]; do
#         echo "What is the field letter? [ABCDEFG] Just a single letter please"
#         read field_letter
#     done
#   fi
#   sed --in-place "/^ssid=/s/=.*/=$field_letter/" $RUNSWIFT_CHECKOUT_DIR/image/home/nao/data/runswift.cfg
# fi

# in the lab when comp is far away
if [ $FLAG_NO_WIFI -eq 0 ]
then
 field_letter="runswift"
 echo "Are you setting up on a competition field? [y/N]"
 read yesno
 if [[ "$yesno" != [nN]* ]] && [[ -n "$yesno" ]]
   then
   echo "What is the field letter? [ABCDEFG] Just a single letter please"
   read field_letter
 fi
fi

SUCCESSARRAY=()
count=0

for name in "${players[@]}"
do
  team=${playersTeam[$name]}
  numb=${playersNumb[$name]}
  if [ "$team" != "" -a "$numb" != "" ]
  then
    # TODO (jayen): use $name everywhere
    # TODO (jayen): somehow detect if it's changed before removing
    ssh-keygen -R "$name" &> /dev/null
    ssh-keygen -R "$name.local" &> /dev/null
    ssh -o StrictHostKeyChecking=no "nao@$name" true
    ssh -o StrictHostKeyChecking=no "nao@$name.local" true &> /dev/null

    percentage=$(echo $(($count * 100 / ${#players[@]})))
    progress "$name: Setting up on team $team and player $numb" $percentage
    path="$RUNSWIFT_CHECKOUT_DIR/image/home/nao/data/configs"
    osversion="$(getosversion $name)"
    if [[ ! -n "$osversion" ]]
    then
      myerror "unknown robot $name. Can't determine version of $name"
      myerror known robots are "${!robots[@]}"
      exit $LINENO
    fi
    file="$path/$name.cfg"
    if ! sed -i "2s/.*/number=$numb/" $file
    then
      echo ""
      # fail
    fi
    if ! sed -i "3s/.*/team=$team/" $file
    then
      echo ""
      # fail
    fi

    if [ $FLAG_BEHAVIOURONLY -eq 1 ]
    then
      if [ $FLAG_DEBUG -eq 1 ]
      then
        nao_sync-$osversion.sh "$name" < /dev/null
      else
        nao_sync-$osversion.sh "$name" < /dev/null > /tmp/fuck2 2> /tmp/fuck
      fi
    else
      if [ $FLAG_DEBUG -eq 1 ]
      then
        nao_sync-$osversion.sh -ra "$name" < /dev/null
      else
        nao_sync-$osversion.sh -ra "$name" < /dev/null > /tmp/fuck2 2> /tmp/fuck
      fi
    fi
    errorResponse=`cat /tmp/fuck`

    if cat /tmp/fuck | grep -q "not resolve"
    then
      if [ $FLAG_NOEXIT -ne 1 ]
      then
        echo "==="
      fi
      echo "Error. Unable to resolve hostname $name"
      if [ $FLAG_NOEXIT -ne 1 ]
      then
        echo "==="
        exit 1
      fi
    else
      SUCCESSARRAY+=("$name")
    fi

    if [ $FLAG_NO_WIFI -eq 0 ]
    then
      progress "$name: Restarting wifi" $percentage
      command="sudo bin/changeField.py $field_letter"
      if [ $FLAG_DEBUG -eq 1 ]
      then
        ssh "nao@$name" "$command" < /dev/null
      else
        ssh "nao@$name" "$command" < /dev/null &> /dev/null
      fi
    fi

    if [ $FLAG_LOGGING -eq 1 ]
    then
      command="
        file=\"data/configs/${name}.cfg\";
        log_count=\"\$(grep -c 'log=' \$file)\";
        if [ \$log_count -eq 0 ];
        then
          echo -e \"\n\n[debug]\nlog=1\" >> \$file;
        else
          sed -i \"s/log=[0-1]/log=1/\" \$file;
        fi
      "

      ssh "nao@$name" $command < /dev/null &>> /dev/null
      xterm -T "$name" -e ssh -t "nao@$name" "bin/runswift | tee output_logs.txt" &
    fi

    count=$(expr $count + 1)
  fi
done

function join { local IFS="$1"; shift; echo "$*"; }
success_players=`join " , " "${SUCCESSARRAY[@]}"`
progress "Setup complete" 100

echo
echo "${#SUCCESSARRAY[@]}/${#players[@]} players synced: $success_players"
echo "Ready to play."
echo
exit 0
