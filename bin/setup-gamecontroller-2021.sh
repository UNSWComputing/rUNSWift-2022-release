#!/bin/bash
set -e
set -u

# Idempotently download and install game controller and TCM
GAME_CONTROLLER=GameController2021
cd ${RUNSWIFT_CHECKOUT_DIR}/softwares
mkdir -p ${GAME_CONTROLLER}

# Perhaps this the gamecontroller directory shouldn't have a specific year?
cd ${GAME_CONTROLLER}
if [ ! -f ${GAME_CONTROLLER}.zip ]; then
    echo "Downloading ${GAME_CONTROLLER}.zip"
    wget --continue --timestamping https://github.com/RoboCup-SPL/GameController/releases/download/RoboCup2021/GameController-RoboCup2021.zip -O ${GAME_CONTROLLER}.zip
    unzip -q ${GAME_CONTROLLER}.zip
    mv GameController/* .
    rm -rf GameController
fi
