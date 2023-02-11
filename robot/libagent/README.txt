# 2018-08-11: how jayen got libagent building with v2.8
## make a test project
sudo apt-get install cmake build-essential cmake-curses-gui
sudo apt install python-pip
pip install qibuild --user
PATH=$PATH:$HOME/.local/bin
qibuild config --wizard
qitoolchain create cross-atom /var/www/html/opennao2/build-2.8.1.33/ctc-linux64-atom-2.8.1.33/toolchain.xml 
qibuild add-config cross-atom --toolchain cross-atom
qibuild init
qisrc create agent
cd agent/
qibuild configure -c cross-atom
# qibuild runs: "/usr/bin/cmake" "-GUnix Makefiles" "-DCMAKE_TOOLCHAIN_FILE=/home/runswift/.local/share/qi/toolchains/cross-atom/toolchain-cross-atom.cmake" "-DCMAKE_BUILD_TYPE=Debug" "-DQI_VIRTUALENV_PATH=/home/runswift/jayen/.qi/venvs/py-cross-atom" "-Dqibuild_DIR=/home/runswift/.local/share/cmake/qibuild" "/home/runswift/jayen/agent"
# jayen thinks we should use: /usr/bin/cmake -DCMAKE_TOOLCHAIN_FILE=/home/runswift/.local/share/qi/toolchains/cross-atom/toolchain-cross-atom.cmake -DCMAKE_BUILD_TYPE=Release ..
qibuild make -c cross-atom
cd build-cross-atom/
make
ccmake ..
## copy over CMakeLists.txt & qiproject.xml to libagent and make small changes
qibuild configure -c cross-atom
qibuild make -c cross-atom
