#!/bin/bash

# This script downloads runsweb (AppImage version) and runs runsweb
# We choose AppImage because the deb would add another runsweb to your path, and it will work on non-deb linuxes
# We set GH_TOKEN so runsweb auto-updates, so GH_TOKEN is not used only for downloading

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

# creates a github token if it doesn't already exist
if [[ ! -v GH_TOKEN ]]; then
    # load script so we don't have to ask the user to restart bash in case they just set it up in this shell session
    source ~/.runswift.bash
    if [[ ! -v GH_TOKEN ]]; then
        myecho "Generate a token at https://github.com/settings/tokens/new?scopes=repo&description=Downloading+rUNSWeb and paste it here."

        # sleep so people have time to notice above message before the page opens
        sleep 3
        xdg-open "https://github.com/settings/tokens/new?scopes=repo&description=Downloading+rUNSWeb"

        myecho "Paste token here:"
        read
        # use GH_TOKEN so electron-updater can auto-update
        export GH_TOKEN="$REPLY"
        echo export GH_TOKEN=\"$GH_TOKEN\" >> ~/.runswift.bash
    fi
fi

if [[ ! -f "$RUNSWIFT_CHECKOUT_DIR/softwares/rUNSWeb.AppImage" ]]; then
    # download runsweb
    python3 "$BIN_DIR/download-runsweb.py"
    chmod +x "$RUNSWIFT_CHECKOUT_DIR/softwares/rUNSWeb.AppImage"
fi

"$RUNSWIFT_CHECKOUT_DIR/softwares/rUNSWeb.AppImage" "$@"
