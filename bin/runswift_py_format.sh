#!/usr/bin/env bash


# Also make it work in SourceTree =/
# https://community.atlassian.com/t5/Bitbucket-questions/SourceTree-Hook-failing-because-paths-don-t-seem-to-be-set/qaq-p/274792
export PATH=/usr/local/bin:$PATH


cd ${RUNSWIFT_CHECKOUT_DIR}
PYTHON_FILES_WC=$(git status -s | grep ".py" | wc -l | tr -d '[:space:]')
if [ "$PYTHON_FILES_WC" -ne "0" ] || [ "$1" != "--python-files-only" ]; then

    # (Peter) This could be "$BLACK_VIRTUALENV_PATH/bin/activate" though I can't presently recall how to
    # export bash variables such that they'll work in this git hook, and it all disappears
    # once python-black makes it into an Ubuntu LTS (likely 20.04).
    source "$RUNSWIFT_CHECKOUT_DIR/.virtualenvs/runswift-black-py3k/bin/activate"

    black --version >/dev/null 2>&1 || {
        echo >&2 "You must install black, but it's not installed. Try:"
        echo >&2 "    sudo pip3 install python-black"
        echo >&2 "Or follow - https://github.com/psf/black#installation-and-usage"
        exit 1;
    }
    # Note that black is configured under the
    # [tool.black] heading of the "pyproject.toml" file
    black --quiet image/home/

    deactivate
# else
#    echo "No python changes."
fi
