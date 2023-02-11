#!/bin/bash

# This script is for testing various networks' inference times on a robot
# Use the notebook to create the weights first

REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")/../../bin
source "$BIN_DIR/source.sh"

rsync -aP *.png nao@10.0.0.13:

#CCACHE_PATH=
#source "$RUNSWIFT_CHECKOUT_DIR/softwares/ctc-linux64-atom-2.8.1.33/yocto-sdk/environment-setup-core2-32-sbr-linux"

for net in NeuralNetwork JNN1 JNN2 JNN3 JNN4 JNN5 JNN6 JNN7 JNN8 JNN9 JNN10 JNN11; do
#for net in NeuralNetwork JNN3 JNN5; do
#for net in JNN5; do
    ([[ convert_models.py -nt $net.models/test.cpp ]] || [[ test-template.cpp -nt $net.models/test.cpp ]] || [[ conv2d_op_internal_template.hpp -nt $net.models/test.cpp ]] || [[ convolutional_layers_header.hpp -nt $net.models/test.cpp ]]) && ./convert_models.py "marvin/models/$net.pth" $net.models marvin.basic_models.$net
    cd $net.models
    if [[ -v CROSS_COMPILE ]]; then
#        # for debugging
#        make test CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP" LDFLAGS="-lpthread $LDFLAGS" CXXFLAGS="-g -O3 --std=gnu++14 -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
#        # for profiling
#        make test CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP" LDFLAGS="-lpthread $LDFLAGS" CXXFLAGS="-pg -O3 --std=gnu++14 -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
        # for timing
        make test CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP" LDFLAGS="-lpthread $LDFLAGS" CXXFLAGS="-O3 --std=gnu++14 -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
    else
#        # for profiling
#        make test CXX="g++ -static -m64 -march=silvermont -mtune=silvermont" CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP" LDFLAGS=-pthread CXXFLAGS="-pg -O3 --std=gnu++17 -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
        # for timing
#        # 0.0743899
#        make test CXX="g++ -static -m64 -march=silvermont -mtune=silvermont" CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP" LDFLAGS=-pthread CXXFLAGS="-O3 --std=gnu++17 -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
#        # 0.067133
#        make test CXX="clang++ -static -m64 -march=silvermont -mtune=silvermont" CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP" LDFLAGS=-pthread CXXFLAGS="-O3 --std=gnu++17 -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
        # 0.0339725
        make test CXX="~/intel/bin/icpc -static -m64 -march=silvermont -mtune=silvermont" CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP" LDFLAGS=-pthread CXXFLAGS="-O3 --std=c++17 -fopenmp -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
    fi
    cd -
    rsync -aPz --del $net.models nao@10.0.0.13:
#    # for debugging
#    ssh nao@10.0.0.13 gdb -x /home/nao/data/gdb_cmds.txt --return-child-result --quiet --args $net.models/test $net.models/ *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png
#    # for profiling
#    ssh nao@10.0.0.13 $net.models/test $net.models/ *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png
#    rsync -aPz nao@10.0.0.13:gmon.out $net.models/
#    gprof $net.models/test $net.models/gmon.out | tee $net.models/gprof.out
    # for timing
    ssh nao@10.0.0.13 $net.models/test $net.models/ *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png *.png

    rsync -aP nao@10.0.0.13:$net.models/time.txt $net.models/
done
