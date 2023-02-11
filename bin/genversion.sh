#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")
source "$BIN_DIR/source.sh"

cat << VERSION_CPP
#include "utils/version.hpp"

std::string getVersion() {
  return $(version.sh | sed -r 's/^/"/;s/$/\\n"/');
}
VERSION_CPP
