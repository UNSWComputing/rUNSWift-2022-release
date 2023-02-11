cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

project(VATNAO-LEGACY)

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CTC_DIR}/libnaoqi/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/png/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/zlib/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/../sysroot_legacy/usr/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/png/include)
if (${CTC_DIR} MATCHES atom-2.1)
    # purposefully empty
    # when CTC_DIR is undefined, "${CTC_DIR} MATCHES atom-2.1" is false (expected)
    # when CTC_DIR is undefined, "NOT ${CTC_DIR} MATCHES atom-2.1" is also false (unexpected)
else(${CTC_DIR} MATCHES atom-2.1)
    # for toolchain 2.8 and no toolchain
    include_directories(SYSTEM "$ENV{RUNSWIFT_CHECKOUT_DIR}/robot/tiny-dnn")
endif (${CTC_DIR} MATCHES atom-2.1)


SET(CPP_FILES
  main.cpp
  options.cpp
  ui/ui.cpp
  ui/vatnaoManager.cpp
  ui/frameTools/readImage.cpp
  ui/frameTools/readBoundingBoxes.cpp
  ui/uiElements/imageView.cpp
  app/appAdaptor.cpp
  app/generateFrameInfo.cpp
  app/world/record2reader.cpp
  app/world/world.cpp
  ../offnao/frame.cpp
  ../offnao/naoData.cpp
)

SET(QT4_MOC
    ui/vatnaoManager.hpp
    ui/uiElements/imageView.hpp
)

QT4_WRAP_CPP(QT4_WRAPPED_MOC ${QT4_MOC})

add_executable(vatnao-legacy.bin ${CPP_FILES} ${QT4_WRAPPED_MOC})

set_source_files_properties(
    ${CPP_FILES}
    PROPERTIES COMPILE_FLAGS "-I${QWT_INCLUDE_DIR}"
)

find_library ( QGLVIEWER_LIBRARY NAMES QGLViewer qglviewer-qt4 QGLViewer-qt4 )
set(OpenGL_GL_PREFERENCE LEGACY)
find_package ( OpenGL  REQUIRED )

TARGET_LINK_LIBRARIES(
  vatnao-legacy.bin
  ${QT_LIBRARIES}
  ${QWT_LIBRARY}
  ${QGLVIEWER_LIBRARY}
  ${Boost_IOSTREAMS_LIBRARY}
  soccer
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  offproto
)

if (${CTC_DIR} MATCHES atom-2.1)
    TARGET_LINK_LIBRARIES(
            vatnao-legacy.bin
            ${RUNSWIFT_CHECKOUT_DIR}/softwares/libuuid.so.1.3.0
    )
endif (${CTC_DIR} MATCHES atom-2.1)

set_target_properties(
  vatnao-legacy.bin
  PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH ""
  INSTALL_RPATH_USE_LINK_PATH FALSE
  SKIP_BUILD_RPATH FALSE
)

configure_file(vatnao-wrapper.sh vatnao-legacy @ONLY)
configure_file(gdb-vatnao-wrapper.sh gdbvatnao-legacy @ONLY)
