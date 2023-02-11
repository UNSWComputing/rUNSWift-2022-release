if (${CTC_DIR} MATCHES atom-2.1)
    SET(ROBOT_SRCS
            # Motion
            motion/MotionAdapter.cpp
            motion/effector/AgentEffector.cpp
            motion/touch/AgentTouch.cpp
            )
else (${CTC_DIR} MATCHES atom-2.1)
    SET(ROBOT_SRCS
            # Motion
            motion/MotionAdapter.cpp
            motion/effector/LoLAEffector.cpp
            motion/touch/LoLATouch.cpp
            motion/LoLAData.cpp
            # need these for --simulation right now
            motion/effector/AgentEffector.cpp
            motion/touch/AgentTouch.cpp
            )
endif (${CTC_DIR} MATCHES atom-2.1)

set_source_files_properties(
   perception/PerceptionThread.cpp
   main.cpp
   PROPERTIES COMPILE_FLAGS "-I${PYTHON_INCLUDE_DIR}")

ADD_LIBRARY(robot-static STATIC ${ROBOT_SRCS} )
TARGET_LINK_LIBRARIES( robot-static soccer-static )
SET_TARGET_PROPERTIES(robot-static PROPERTIES OUTPUT_NAME "robot")
SET_TARGET_PROPERTIES(robot-static PROPERTIES PREFIX "lib")
SET_TARGET_PROPERTIES(robot-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

ADD_EXECUTABLE( runswift main.cpp )

message("CTC_DIR=${CTC_DIR}")

if (CMAKE_TOOLCHAIN_FILE)
    if (${CTC_DIR} MATCHES atom-2.8)
        TARGET_LINK_LIBRARIES(
                runswift
                robot-static
                ${PTHREAD_LIBRARIES}
                ${RUNSWIFT_BOOST}
                ${PYTHON_LIBRARY}
                ${PNG_LIBRARIES}
                pthread rt
                # TODO: Not quick hack to make it work.
                # TODO: ... somewhere upstream we are NOT finding BZip2 correctly,
                # TODO: ... i.e. we should be able to just
                # TODO: ... TARGET_LINK_LIBRARIES (runswift ${BZIP2_LIBRARIES})
                # TODO: ... http://www.cmake.org/Wiki/CMake:How_To_Find_Libraries#Using_external_libraries
                ${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib/libbz2.so.1
                # TODO: ... Ditto TARGET_LINK_LIBRARIES (runswift ${ZLIB_LIBRARIES}) should work but doesn't hence hack.
                #TARGET_LINK_LIBRARIES (runswift ${ZLIB_LIBRARIES})
                ${CTC_DIR}/yocto-sdk/sysroots/core2-32-sbr-linux/lib/libz.so.1
                ${JPEG_LIBRARY}
        )
    elseif (${CTC_DIR} MATCHES atom-2.1)
        TARGET_LINK_LIBRARIES(
                runswift
                robot-static
                ${PTHREAD_LIBRARIES}
        )
    else() # not ${CTC_DIR} MATCHES atom-2.[18]
        message(FATAL_ERROR "Unknown CTC_DIR")
    endif (${CTC_DIR} MATCHES atom-2.8)
else(CMAKE_TOOLCHAIN_FILE)
    find_library(RT_LIBRARY  rt)
    TARGET_LINK_LIBRARIES(
            runswift
            robot-static
            ${PTHREAD_LIBRARIES}
            ${RT_LIBRARY}
    )
endif (CMAKE_TOOLCHAIN_FILE)
ADD_CUSTOM_COMMAND ( TARGET runswift POST_BUILD
   COMMAND ${CMAKE_OBJCOPY} --only-keep-debug runswift runswift.debug
   COMMAND ${CMAKE_OBJCOPY} --strip-debug runswift
   COMMAND ${CMAKE_OBJCOPY} --add-gnu-debuglink=runswift.debug runswift
   COMMAND rm version.cpp
)
