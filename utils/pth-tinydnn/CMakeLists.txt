project(pth-tinydnn)

include_directories($ENV{RUNSWIFT_CHECKOUT_DIR}/robot/tiny-jnn $ENV{RUNSWIFT_CHECKOUT_DIR}/robot/tiny-dnn)
add_definitions(-DDNN_USE_IMAGE_API -DCNN_USE_SSE -DCNN_USE_OMP)
link_libraries(pthread)
add_executable (test test.cpp)
