cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)
find_package(Boost REQUIRED)
message("BOOST_ROOT=${BOOST_ROOT}")
include_directories(${Boost_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
if(${CTC_DIR} MATCHES atom-2.8)
    include_directories(${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/usr/include)
endif(${CTC_DIR} MATCHES atom-2.8)
include_directories(${CTC_DIR}/libnaoqi/include)
if(${CTC_DIR} MATCHES atom-2.8)
    include_directories(${CTC_DIR}/qi-framework/include)
    # The lines below should already be included in the core2-32-sbr-linux/usr/include
    #include_directories(${CTC_DIR}/bzip2/include)

    set(
    ZLIB_INCLUDE_DIR
    ${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/usr/include
    )

    set(
    ZLIB_LIBRARIES
    ${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/lib/libz.so.1
    )

    set(
    BZIP2_LIBRARIES
    ${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib/libbz2.so.1
    )

    set(
    BZIP2_INCLUDE_DIR
    ${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/usr/include
    )

    set(
    PNG_LIBRARY
    ${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib/libpng16.so.16
    )

    set(
    PNG_PNG_INCLUDE_DIR
    ${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/usr/include
    )
elseif(${CTC_DIR} MATCHES atom-2.1)
    include_directories(${CTC_DIR}/bzip2/include)
    include_directories(${CTC_DIR}/zlib/include)
    include_directories(${CTC_DIR}/png/include)
    include_directories(${CTC_DIR}/jpeg/include)
endif(${CTC_DIR} MATCHES atom-2.8)
if(${CTC_DIR} MATCHES atom-2.1)
  include("${CMAKE_CURRENT_SOURCE_DIR}/libagent.cmake")
endif(${CTC_DIR} MATCHES atom-2.1)

include("${CMAKE_CURRENT_SOURCE_DIR}/runswift.cmake")

include("${CMAKE_CURRENT_SOURCE_DIR}/robot.cmake")

#include("${CMAKE_CURRENT_SOURCE_DIR}/testrunswift.cmake")
