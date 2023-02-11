#!/bin/bash
set -e
set -u

# Idempotently download and install game controller and TCM
export GAME_CONTROLLER=GameController2019
cd ${RUNSWIFT_CHECKOUT_DIR}/softwares
mkdir -p ${GAME_CONTROLLER}

# Perhaps this the gamecontroller directory shouldn't have a specific year?
cd ${GAME_CONTROLLER}
if [ ! -f ${GAME_CONTROLLER}.zip ]; then
    echo "Downloading ${GAME_CONTROLLER}.zip"
    wget --continue --timestamping http://spl.robocup.org/wp-content/uploads/downloads/${GAME_CONTROLLER}.zip
    unzip -q ${GAME_CONTROLLER}.zip
fi
