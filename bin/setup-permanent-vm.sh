#!/bin/bash

green=`tput setaf 2`
reset=`tput sgr0`

# creates a public key if it doesn't already exist
if ! ssh-keygen -l -f ~/.ssh/id_rsa.pub &> /dev/null; then
  if ssh-keygen -l -f ~/.ssh/id_rsa &> /dev/null; then
    echo "Deleting existing private key"
    rm ~/.ssh/id_rsa
  fi
  ssh-keygen -f ~/.ssh/id_rsa -N '' -q
  if ! grep -qf ~/.ssh/id_rsa.pub "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys; then
    echo "${green}Enter your full name:${reset}"
    read name
    echo >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
    echo "# $name's key" >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
    cat ~/.ssh/id_rsa.pub >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
  fi
fi

echo "Copy and paste the text below in the 'key' section in https://github.com/settings/ssh/new"
echo "${green}#########################################################################################"
cat ~/.ssh/id_rsa.pub
echo "#########################################################################################${reset}"

# sleep so people have time to notice above message before the page opens
sleep 1
xdg-open "https://github.com/settings/ssh/new"

echo "All done! Please create a pull request with your public key in /image/home/nao/.ssh/authorized_keys"
