if(CMAKE_TOOLCHAIN_FILE)
  # Override the fact that FindQt4 uses NO_CMAKE_FIND_ROOT_PATH
  set(QT_BINARY_DIR "${CTC_DIR}/../sysroot_legacy/usr/bin")
  if (QT_QMAKE_EXECUTABLE)
    # need this during link
    set(CMAKE_SYSROOT ${CTC_DIR}/cross/i686-aldebaran-linux-gnu/sysroot)
  else () # not QT_QMAKE_EXECUTABLE
    # need this during cmake
    set(CMAKE_SYSROOT ${CTC_DIR}/../sysroot_legacy)
  endif (QT_QMAKE_EXECUTABLE)
endif(CMAKE_TOOLCHAIN_FILE)

FIND_PACKAGE(Qt4 REQUIRED)


#SET(QT_USE_QTSVG TRUE)#svg support comes in via plugin when using QImage
SET(QT_USE_QTNETWORK TRUE)
SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QTXML TRUE)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Profile>:QT_NO_DEBUG>)
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Coverage>:QT_NO_DEBUG>)
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Valgrind>:QT_NO_DEBUG>)

find_path ( QWT_INCLUDE_DIR qwt.h /usr/include/qwt6 /usr/include/qwt5 /usr/include/qwt-qt4 /usr/include/qwt )
find_library ( QWT_LIBRARY NAMES qwt qwt6 qwt-qt4 )
