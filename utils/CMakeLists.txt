cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)


include(bootstrap-robotdir.cmake)

if(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")
    message("Skipping offnao, vatnao and vatnao-legacy builds on macOS")
else() # ${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin"
    if(${CTC_DIR} MATCHES atom-2.8)
        # QT Error here
        message("Skipping offnao, vatnao and vatnao-legacy builds for v6")
    else(${CTC_DIR} MATCHES atom-2.8)
        include(bootstrap-qt.cmake)

        # offnao and vision tests are required due to use of absolute
        # paths in header files
        include_directories("offnao")
        include_directories("vatnao")
        include_directories("vatnao-legacy")

        add_subdirectory(offnao)
        add_subdirectory(vatnao)
        add_subdirectory(vatnao-legacy)
    endif(${CTC_DIR} MATCHES atom-2.8)
endif(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")

include_directories("state-estimation-simulator")
add_subdirectory(state-estimation-simulator)
add_subdirectory(ofn-to-ofn2)
