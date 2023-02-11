cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

PROJECT(OFFNAO)

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CTC_DIR}/libnaoqi/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/png/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/zlib/include)
INCLUDE_DIRECTORIES(${CTC_DIR}/../sysroot_legacy/usr/include)

SET(OFFNAO_CXX_SRCS
   utils/OverlayPainter.cpp
   utils/FieldPainter.cpp
   utils/CPlaneColours.cpp
   frame.cpp
   main.cpp
   mediaPanel.cpp
   naoData.cpp
   visualiser.cpp
   readers/dumpReader.cpp
   readers/networkReader.cpp
   readers/reader.cpp
   readers/recordReader.cpp
   readers/record2Reader.cpp
   readers/bbdReader.cpp
   readers/bbd2Reader.cpp
   tabs/PointCloud.cpp
   tabs/classifier.cpp
   tabs/variableView.cpp
   tabs/plots.cpp
   tabs/yuvHistogram.cpp
   tabs/overviewTab.cpp
   tabs/LogsTab.cpp
   tabs/LogTab.cpp
   tabs/sensorTab.cpp
   tabs/graphTab.cpp
   tabs/jointsTab.cpp
   tabs/temperatureTab.cpp
   tabs/walkTab.cpp
   tabs/aroundFeetTab.cpp
   tabs/visionTab.cpp
   tabs/zmpTab.cpp
   tabs/fieldView.cpp
   tabs/cameraTab.cpp
   tabs/cameraPoseTab.cpp
   tabs/tab.cpp
)

SET(OFFNAO_MOC
   readers/reader.hpp
   readers/dumpReader.hpp
   readers/recordReader.hpp
   readers/record2Reader.hpp
   readers/bbdReader.hpp
   readers/bbd2Reader.hpp
   mediaPanel.hpp
   visualiser.hpp
   tabs/tab.hpp
   tabs/cameraTab.hpp
   tabs/cameraPoseTab.hpp
   tabs/graphTab.hpp
   tabs/jointsTab.hpp
   tabs/temperatureTab.hpp
   tabs/walkTab.hpp
   tabs/aroundFeetTab.hpp
   tabs/overviewTab.hpp
   tabs/LogsTab.hpp
   tabs/LogTab.hpp
   tabs/plots.hpp
   tabs/sensorTab.hpp
   tabs/visionTab.hpp
   tabs/zmpTab.hpp
)

## protobuf stuff

## improper fix
# this is definitely the wrong way to do it, but i don't know how much longer offnao will survive so i'm not spending time to fix it properly
# the proper fix is probably to move the block in runswift.cmake up a level so it can be used here
if(${CTC_DIR} MATCHES atom-2.8)
    set(Protobuf_INCLUDE_DIRS $ENV{RUNSWIFT_CHECKOUT_DIR}/softwares/ctc-linux64-atom-2.8.1.33/yocto-sdk/sysroots/core2-32-sbr-linux/usr/include/)
elseif(${CTC_DIR} MATCHES atom-2.1)
    set(Protobuf_INCLUDE_DIRS $ENV{RUNSWIFT_CHECKOUT_DIR}/softwares/protobuf-2.6.1/include/)
endif(${CTC_DIR} MATCHES atom-2.8)

IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.6)
    #3.10 on ubuntu 18.04
    MESSAGE ("cmake >= 3.6 detected")
    #set(Protobuf_DEBUG ON)
ELSE () #not ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.6
    #3.5 on ubuntu 16.04
    MESSAGE ("cmake < 3.6 detected")
    set(PROTOBUF_INCLUDE_DIRS ${Protobuf_INCLUDE_DIRS})
ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.6)
## end improper fix

# protobuf stuff as a separate library so it doesn't interfere with include dirs on other files
add_library(offproto serialise.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(offproto PRIVATE ${Protobuf_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})
target_include_directories(offproto PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
add_dependencies(offproto protoswift)

## end protobuf stuff

SET(OFFNAO_UI
   visualiser.ui
   ConnectionBar.ui
   tabs/LogTab.ui
   tabs/LogsTab.ui
)

SET(OFFNAO_RES
   resources/visualiser_resources.qrc
)

# build cxx files for resources
QT4_ADD_RESOURCES(OFFNAO_RES_SRCS ${OFFNAO_RES})

# build ui_XXX files from the XML-style .ui files
QT4_WRAP_UI(OFFNAO_UI_SRCS ${OFFNAO_UI})

# this moc's the above variable and appends to the cxx sources
QT4_WRAP_CPP(OFFNAO_MOC_SRCS ${OFFNAO_MOC})

ADD_EXECUTABLE(offnao.bin ${OFFNAO_CXX_SRCS} ${OFFNAO_RES_SRCS} ${OFFNAO_MOC_SRCS} ${OFFNAO_UI_SRCS})

set_source_files_properties(
   tabs/graphTab.cpp
   tabs/jointsTab.cpp
   tabs/temperatureTab.cpp
   tabs/plots.cpp
   tabs/walkTab.cpp
   tabs/aroundFeetTab.cpp
   tabs/zmpTab.cpp
   main.cpp
   visualiser.cpp
   ${OFFNAO_MOC_SRCS} #too lazy to split and list them
   PROPERTIES COMPILE_FLAGS "-I${QWT_INCLUDE_DIR}")

find_library ( QGLVIEWER_LIBRARY NAMES QGLViewer qglviewer-qt4 QGLViewer-qt4)
set(OpenGL_GL_PREFERENCE LEGACY)
find_package ( OpenGL  REQUIRED )

TARGET_LINK_LIBRARIES(
   offnao.bin
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
            offnao.bin
            ${RUNSWIFT_CHECKOUT_DIR}/softwares/libuuid.so.1.3.0
    )
endif (${CTC_DIR} MATCHES atom-2.1)

# unadulterated cmake rpath: $CTC_DIR/sysroot/usr/lib/qt4:$CTC_DIR/sysroot/usr/lib:build-relwithdebinfo/robot
set_target_properties(offnao.bin PROPERTIES
   BUILD_WITH_INSTALL_RPATH FALSE
   INSTALL_RPATH ""
   INSTALL_RPATH_USE_LINK_PATH FALSE
   SKIP_BUILD_RPATH FALSE
)

configure_file(offnao-wrapper.sh offnao @ONLY)
configure_file(goffnao-wrapper.sh goffnao @ONLY)
configure_file(voffnao-wrapper.sh voffnao @ONLY)
configure_file(cgoffnao-wrapper.sh cgoffnao @ONLY)
