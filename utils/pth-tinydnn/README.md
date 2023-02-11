# pth to tiny-dnn converter

This folder is modeled after https://github.com/tiny-dnn/tiny-dnn/tree/master/examples/ssd_detection

The SSD detection example in tiny-dnn:
- converts weights & biases from pth to txt files (which tiny-dnn understands)
  - the specification for which layers go in which files is in convert_models.py
- includes a test cpp which:
  - sets up the network, layer by layer
  - loads the weights into the layers

Our version is more or less the same, but:
- out test.cpp uses the optimized versions of the layers
  - kernel size and dilation are template parameters
    - dilation defaults to 1

## our version:

```shell script
wget https://s3.amazonaws.com/amdegroot-models/ssd300_mAP_77.43_v2.pth
wget https://user-images.githubusercontent.com/1730504/47263759-67bc1900-d53a-11e8-91cd-4bb4648668b7.png https://user-images.githubusercontent.com/1730504/47264055-d9976100-d540-11e8-98a5-0af7871374fd.png
./convert_models.py ssd300_mAP_77.43_v2.pth models
source "$RUNSWIFT_CHECKOUT_DIR/softwares/ctc-linux64-atom-2.8.1.33/yocto-sdk/environment-setup-core2-32-sbr-linux"
make test CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP" LDFLAGS="-lpthread $LDFLAGS" CXXFLAGS="-O3 --std=gnu++14 -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
./test models/ 47263759-67bc1900-d53a-11e8-91cd-4bb4648668b7.png 47264055-d9976100-d540-11e8-98a5-0af7871374fd.png
```

### results

Benchmarking is done on kodos.  TODO: on a v6

```
real    0m4.90039s
user    0m4.852276s
sys     0m0.48009s
Bounding box coordinates:
x_min = -9.91948, x_max = 307.739, y_min = 57.651, y_max = 210.922, class = 1, score = 0.975054
real    0m4.87682s
user    0m4.872796s
sys     0m0.4014s
Bounding box coordinates:
x_min = 17.5945, x_max = 273.347, y_min = 68.3175, y_max = 280.413, class = 18, score = 0.977631
```

## tiny-dnn version:

```shell script
make test CPPFLAGS="-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP -DTINY_DNN_TEST" LDFLAGS="-lpthread $LDFLAGS" CXXFLAGS="-O3 --std=gnu++14 -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-dnn -I$RUNSWIFT_CHECKOUT_DIR/robot/tiny-jnn"
```

### results

Benchmarking is done on kodos.  TODO: on a v6

```
real    0m28.66s
user    0m28.623762s
sys     0m0.35970s
Bounding box coordinates:
x_min = -9.91944, x_max = 307.739, y_min = 57.651, y_max = 210.922, class = 1, score = 0.975054
real    0m28.3107s
user    0m28.305585s
sys     0m0.3986s
Bounding box coordinates:
x_min = 17.5944, x_max = 273.347, y_min = 68.3175, y_max = 280.413, class = 18, score = 0.977631
```

# Performance

I was able to look up how long each instruction takes (comments in the code) for most instructions, but not for MULPS
and ADDPS reading from memory (register *= memory).  I determined that MULPS reading from memory takes one more cycle.
Also, I saw that 2 MOVAPS instructions can run in the same cycle.  Unfortunately, I was not able to exploit this
information to get better performance.  I would assume that ADDPS reading from memory also takes one extra cycle.

```shell script
cd $RUNSWIFT_CHECKOUT_DIR/softwares
wget https://www.agner.org/optimize/testp.zip
mkdir testp
cd testp
unzip ../testp.zip
unzip DriverSrcLinux.zip
# assumes setup-gpu.sh has already been run
make KERNELDIR=$RUNSWIFT_CHECKOUT_DIR/softwares/linux-aldebaran/
sed -i 's/^make/#make/' install.sh
rsync -aP install.sh MSRdrv.ko nao@10.0.0.13:
ssh nao@10.0.0.13 sudo mount -o remount,rw /
ssh nao@10.0.0.13 sudo bash install.sh
unzip PMCTest.zip 
nano PMCTestB64.nasm
  # add your test
sudo apt install nasm
sed -i 's/-lpthread/-lpthread -static/' a64.sh
bash a64.sh
rsync -aP x nao@10.0.0.13:
ssh nao@10.0.0.13 ./x
# discovered that MULPS m128,x has latency 6
# so MOVAPS x,x + MULPS m128,x has latency 7
# and MOVUPS m128,x + MULPS x,x has latency 8
```