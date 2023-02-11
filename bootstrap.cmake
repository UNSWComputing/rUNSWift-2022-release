### Added by jayen for profiling
set ( CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_RELEASE} -pg" CACHE STRING
    "Flags used by the C++ compiler during profile builds."
    FORCE )

set ( CMAKE_C_FLAGS_PROFILE "${CMAKE_C_FLAGS_RELEASE} -pg" CACHE STRING
    "Flags used by the C compiler during profile builds."
    FORCE )

set ( CMAKE_EXE_LINKER_FLAGS_PROFILE
    "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -pg" CACHE STRING
    "Flags used for linking binaries during profile builds."
    FORCE )

set ( CMAKE_MODULE_LINKER_FLAGS_PROFILE
    "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} -pg" CACHE STRING
    "Flags used for linking binaries during profile builds."
    FORCE )

set ( CMAKE_SHARED_LINKER_FLAGS_PROFILE
    "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -pg" CACHE STRING
    "Flags used by the shared libraries linker during profile builds."
    FORCE )

MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_PROFILE
    CMAKE_C_FLAGS_PROFILE
    CMAKE_EXE_LINKER_FLAGS_PROFILE
    CMAKE_MODULE_LINKER_FLAGS_PROFILE
    CMAKE_SHARED_LINKER_FLAGS_PROFILE )

### Added by jayen for coverage
set ( CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_RELEASE} --coverage" CACHE STRING
    "Flags used by the C++ compiler during coverage builds."
    FORCE )

set ( CMAKE_C_FLAGS_COVERAGE "${CMAKE_C_FLAGS_RELEASE} --coverage" CACHE STRING
    "Flags used by the C compiler during coverage builds."
    FORCE )

set ( CMAKE_EXE_LINKER_FLAGS_COVERAGE
    "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --coverage" CACHE STRING
    "Flags used for linking binaries during coverage builds."
    FORCE )

set ( CMAKE_MODULE_LINKER_FLAGS_COVERAGE
    "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} --coverage" CACHE STRING
    "Flags used for linking binaries during coverage builds."
    FORCE )

set ( CMAKE_SHARED_LINKER_FLAGS_COVERAGE
    "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} --coverage" CACHE STRING
    "Flags used by the shared libraries linker during coverage builds."
    FORCE )

MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_COVERAGE
    CMAKE_C_FLAGS_COVERAGE
    CMAKE_EXE_LINKER_FLAGS_COVERAGE
    CMAKE_MODULE_LINKER_FLAGS_COVERAGE
    CMAKE_SHARED_LINKER_FLAGS_COVERAGE )

## Added by Sean for mudflap
set ( CMAKE_CXX_FLAGS_MUDFLAP "-fmudflapth -g" CACHE STRING
    "Flags used by the C++ compiler during mudflap builds."
    FORCE )

set ( CMAKE_C_FLAGS_MUDFLAP "-fmudflapth -g" CACHE STRING
    "Flags used by the C compiler during mudflap builds."
    FORCE )

set ( CMAKE_EXE_LINKER_FLAGS_MUDFLAP
    "-lmudflapth" CACHE STRING
    "Flags used for linking binaries during mudflap builds."
    FORCE )

set ( CMAKE_MODULE_LINKER_FLAGS_MUDFLAP
    "-lmudflapth" CACHE STRING
    "Flags used for linking binaries during mudflap builds."
    FORCE )

set ( CMAKE_SHARED_LINKER_FLAGS_MUDFLAP
    "-lmudflapth" CACHE STRING
    "Flags used by the shared libraries linker during mudflap builds."
    FORCE )

MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_MUDFLAP
    CMAKE_C_FLAGS_MUDFLAP
    CMAKE_EXE_LINKER_FLAGS_MUDFLAP
    CMAKE_MODULE_LINKER_FLAGS_MUDFLAP
    CMAKE_SHARED_LINKER_FLAGS_MUDFLAP )

### Added by jayen for valgrind
set ( CMAKE_CXX_FLAGS_VALGRIND "${CMAKE_CXX_FLAGS_RELEASE} -DVALGRIND -g" CACHE STRING
    "Flags used by the C++ compiler during valgrind builds."
    FORCE )

set ( CMAKE_C_FLAGS_VALGRIND "${CMAKE_C_FLAGS_RELEASE} -DVALGRIND -g" CACHE STRING
    "Flags used by the C compiler during valgrind builds."
    FORCE )

set ( CMAKE_EXE_LINKER_FLAGS_VALGRIND
    "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING
    "Flags used for linking binaries during valgrind builds."
    FORCE )

set ( CMAKE_MODULE_LINKER_FLAGS_VALGRIND
    "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING
    "Flags used for linking binaries during valgrind builds."
    FORCE )

set ( CMAKE_SHARED_LINKER_FLAGS_VALGRIND
    "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING
    "Flags used by the shared libraries linker during valgrind builds."
    FORCE )

MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_VALGRIND
    CMAKE_C_FLAGS_VALGRIND
    CMAKE_EXE_LINKER_FLAGS_VALGRIND
    CMAKE_MODULE_LINKER_FLAGS_VALGRIND
    CMAKE_SHARED_LINKER_FLAGS_VALGRIND )

# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
SET( CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Profile Coverage Mudflap Valgrind."
    FORCE )
