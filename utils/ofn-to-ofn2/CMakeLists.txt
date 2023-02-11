cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

project(OFN_TO_OFN2)

SET(CPP_FILES
  main.cpp
)

add_executable(ofn-to-ofn2.bin ${CPP_FILES})

TARGET_LINK_LIBRARIES(
  ofn-to-ofn2.bin
  soccer
)

set_target_properties(
  ofn-to-ofn2.bin
  PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH ""
  INSTALL_RPATH_USE_LINK_PATH FALSE
  SKIP_BUILD_RPATH FALSE
)
