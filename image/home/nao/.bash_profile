# /etc/skel/.bash_profile

# This file is sourced by bash for login shells.  The following line
# runs your .bashrc and is recommended by the bash info pages.
[[ -f ~/.bashrc ]] && . ~/.bashrc

if [ -d /home/nao/bin ]; then
   export PATH="/home/nao/bin:$PATH"
fi
