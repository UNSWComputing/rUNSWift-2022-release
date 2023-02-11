cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

project(STATE_ESTIMATION_SIMULATOR)


INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})


SET(CPP_FILES
  main.cpp
)

add_executable(state-estimation-simulator.bin ${CPP_FILES})

TARGET_LINK_LIBRARIES(
  state-estimation-simulator.bin
  soccer
)

set_target_properties(
  state-estimation-simulator.bin
  PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH ""
  INSTALL_RPATH_USE_LINK_PATH FALSE
  SKIP_BUILD_RPATH FALSE
)
