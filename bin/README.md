style guide for shell scripts
-----------------------------
don't always use exit 1 for failure.  try to use fairly unique exit codes, like the current line number
  doesn't need to be updated when the line number changes
  https://github.com/UNSWComputing/rUNSWift/pull/1803#discussion_r252173481
don't use control operators like || or &&.  use if and if ! for readability.
don't use one-hyphen command line options like -q.  use --quiet for reability.
consider squelching non-error output
use myecho before a command that takes a while
try not to use/nest $().  use several variables and `` for reability
try not to use ${NAME}.  use $NAME for readbility
