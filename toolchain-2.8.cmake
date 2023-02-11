cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

set(CTC_DIR $ENV{RUNSWIFT_CHECKOUT_DIR}/softwares/ctc-linux64-atom-2.8.1.33)
set(SYSROOTS ${CTC_DIR}/yocto-sdk/sysroots)
set(GNU_PREFIX ${SYSROOTS}/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 4)

set(CMAKE_SYSROOT ${SYSROOTS}/core2-32-sbr-linux)

set(INTEL_BIN /opt/intel/oneapi/compiler/2021.2.0/linux/bin/ia32)

if(EXISTS ${INTEL_BIN})
  # atom should be silvermont but icc will coerce it anyway
  add_definitions(-mtune=atom -gnu-prefix=${GNU_PREFIX})
else() # not EXISTS ${INTEL_BIN}
  add_definitions(-march=core2 -mtune=core2)
endif(EXISTS ${INTEL_BIN})
add_definitions(-m32 -msse3 -mfpmath=sse --sysroot=${SYSROOTS}/core2-32-sbr-linux -std=gnu++14)

set(CMAKE_C_COMPILER   ${GNU_PREFIX}gcc)
if(EXISTS ${INTEL_BIN})
  set(CMAKE_C_COMPILER ${INTEL_BIN}/icc)
  set(CMAKE_CXX_COMPILER ${INTEL_BIN}/icpc)
  set(CMAKE_AR ${INTEL_BIN}/xiar CACHE INTERNAL "")
  set(CMAKE_OBJCOPY ${GNU_PREFIX}objcopy)
  set(CMAKE_SHARED_LINKER_FLAGS "-gnu-prefix=${GNU_PREFIX} -m32 --sysroot=${SYSROOTS}/core2-32-sbr-linux" CACHE INTERNAL "")
  set(CMAKE_EXE_LINKER_FLAGS "-gnu-prefix=${GNU_PREFIX} -m32 --sysroot=${SYSROOTS}/core2-32-sbr-linux" CACHE INTERNAL "")
endif(EXISTS ${INTEL_BIN})

set(CMAKE_FIND_ROOT_PATH
    ${CTC_DIR}/yocto-sdk
    ${CTC_DIR}
)
# we may actually want native programs, but i prefer we override that on a FIND_XXX basis
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
