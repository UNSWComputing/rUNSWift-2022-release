PROJECT( RUNSWIFT )

############################ PROJECT SOURCES FILES
# Add source files needed to compile this project

SET(SOCCER_SRCS
   soccer.cpp

   # Python
   perception/behaviour/python/PythonSkill.cpp
   perception/behaviour/python/RobotModule.cpp
   perception/behaviour/python/RegisterConverters.cpp

   # Vision
   perception/vision/Vision.cpp
   perception/vision/Fovea.cpp
   perception/vision/Region/Region.cpp
   perception/vision/VisionAdapter.cpp
   perception/vision/camera/Camera.cpp
   perception/vision/camera/NaoCamera.cpp
   perception/vision/camera/NaoCameraV4.cpp
   perception/vision/camera/CameraToRR.cpp
   perception/vision/camera/CombinedCamera.cpp
   perception/vision/camera/NaoCameraDefinitions.cpp
   perception/vision/camera/terminalCalibration.cpp
   perception/vision/other/YUV.cpp
   perception/vision/other/Ransac.cpp
   perception/vision/other/GMM_classifier.cpp
   perception/vision/other/WriteImage.cpp
   perception/vision/regionfinder/ColourROI.cpp
   perception/vision/regionfinder/RobotColorROI.cpp
   perception/vision/detector/BallDetector.cpp
   perception/vision/detector/ClusterDetector.cpp
   perception/vision/detector/RegionFieldFeatureDetector.cpp
   perception/vision/detector/RobotDetector.cpp
   perception/vision/detector/SSRobotDetector.cpp
   perception/vision/detector/RandomForest.cpp
   perception/vision/detector/DNNHelper.cpp
   perception/vision/middleinfoprocessor/FieldBoundaryFinder.cpp
   perception/dumper/PerceptionDumper.cpp

   # State Estimation
   perception/stateestimation/StateEstimationAdapter.cpp
   perception/stateestimation/localiser/LocaliserTransitioner.cpp
   perception/stateestimation/localiser/Localiser.cpp
   perception/stateestimation/localiser/FieldFeatureLocations.cpp
   perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKF.cpp
   perception/stateestimation/localiser/multimodalcmkf/CMKF.cpp
   perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFParams.cpp
   perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFTransitioner.cpp
   perception/stateestimation/multiballtracker/MultiBallTracker.cpp
   perception/stateestimation/multiballtracker/multiballcmkf/MultiBallCMKF.cpp
   perception/stateestimation/multiballtracker/multiballcmkf/BCMKF.cpp
   perception/stateestimation/multiballtracker/multiballcmkf/MultiBallCMKFParams.cpp
   perception/stateestimation/egoballtracker/EgoBallTracker.cpp
   perception/stateestimation/egoballtracker/ballcmkf/BallCMKF.cpp
   perception/stateestimation/egoballtracker/ballcmkf/BallCMKFParams.cpp
   perception/stateestimation/teamballtracker/TeamBallTracker.cpp
   perception/stateestimation/teamballtracker/TeamBallTrackerTransitioner.cpp
   perception/stateestimation/teamballtracker/teamballkf/TeamBallKF.cpp
   perception/stateestimation/teamballtracker/teamballkf/TeamBallKFParams.cpp
   perception/stateestimation/teamballtracker/teamballkf/TeamBallKFTransitioner.cpp
   perception/stateestimation/robotfilter/RobotFilter.cpp
   perception/stateestimation/robotfilter/types/GroupedRobots.cpp
   perception/stateestimation/robotfilter/types/RobotObservation.cpp

   # Kinematics
   perception/kinematics/Kinematics.cpp
   perception/kinematics/Pose.cpp
   perception/kinematics/Parameters.cpp

   # Behaviour
   perception/behaviour/BehaviourAdapter.cpp
   perception/behaviour/BehaviourHelpers.cpp
   perception/behaviour/SafetySkill.cpp
   perception/behaviour/KickCalibrationSkill.cpp
   perception/behaviour/IMUCalibrationSkill.cpp

   # Types
   types/BallInfo.cpp
   types/BehaviourSharedData.cpp
   types/BroadcastData.cpp
   types/ButtonPresses.cpp
   types/CameraSettings.cpp
   types/FieldFeatureInfo.cpp
   types/RobotVisionInfo.cpp
   types/SharedStateEstimationBundle.cpp
   types/CleverNaoInfo.cpp

   # Misc
   blackboard/Blackboard.cpp
   gamecontroller/GameController.cpp
   gamecontroller/RoboCupGameControlData.cpp
   perception/PerceptionThread.cpp
   receiver/Nao.cpp
   receiver/RemoteControlReceiver.cpp
   receiver/Team.cpp
   thread/Thread.cpp
   thread/ThreadManager.cpp
   transmitter/Nao.cpp
   transmitter/OffNao.cpp
   transmitter/SimExTransmitter.cpp
   transmitter/Team.cpp
   utils/Cluster.cpp
   utils/Connection.cpp
   utils/LeastSquaresLine.cpp
   utils/Logger.cpp
   utils/Timer.cpp
   utils/body.cpp
   utils/home_nao.cpp
   utils/options.cpp
   utils/snappy/snappy-sinksource.cc
   utils/snappy/snappy-stubs-internal.cc
   utils/snappy/snappy.cc
   utils/speech.cpp
   utils/clevernao/CleverBehaviourGenerator.cpp
   utils/clevernao/CleverNao.cpp
   utils/ConfigUtils.cpp

    # Motion
   motion/generator/ActionGenerator.cpp
   motion/generator/GetupGenerator.cpp
   motion/generator/ClippedGenerator.cpp
   motion/generator/BodyModel.cpp
   motion/generator/DistributedGenerator.cpp
   motion/generator/HeadGenerator.cpp
   motion/generator/NullGenerator.cpp
   motion/generator/RefPickupGenerator.cpp
   motion/generator/DeadGenerator.cpp
   motion/generator/Walk2014Generator.cpp
   motion/generator/WalkEnginePreProcessor.cpp

   motion/touch/NullTouch.cpp
   motion/MotionOdometry.cpp

   # Simulator
   simulation/SimulationThread.cpp
   simulation/SimulationConnection.cpp
   simulation/Joints.cpp
   simulation/Sensors.cpp
   simulation/PerceptorInfo.cpp
   simulation/AngleSensor.cpp
   simulation/SimVisionAdapter.cpp
)


## protobuf stuff

if(${CTC_DIR} MATCHES atom-2.8)
  set(Protobuf_INCLUDE_DIR $ENV{RUNSWIFT_CHECKOUT_DIR}/softwares/ctc-linux64-atom-2.8.1.33/yocto-sdk/sysroots/core2-32-sbr-linux/usr/include/)
  set(Protobuf_LIBRARY $ENV{RUNSWIFT_CHECKOUT_DIR}/softwares/ctc-linux64-atom-2.8.1.33/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib/libprotobuf.so)
  set(Protobuf_PROTOC_EXECUTABLE $ENV{RUNSWIFT_CHECKOUT_DIR}/bin/protoc-2.8)
elseif (${CTC_DIR} MATCHES atom-2.1)
  set(Protobuf_INCLUDE_DIR $ENV{RUNSWIFT_CHECKOUT_DIR}/softwares/protobuf-2.6.1/include/)
  set(Protobuf_LIBRARY $ENV{RUNSWIFT_CHECKOUT_DIR}/softwares/protobuf-2.6.1/lib/libprotobuf.so)
  set(Protobuf_PROTOC_EXECUTABLE $ENV{RUNSWIFT_CHECKOUT_DIR}/bin/protoc-2.1)
endif(${CTC_DIR} MATCHES atom-2.8)

IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.6)
  #3.10 on ubuntu 18.04
  MESSAGE ("cmake >= 3.6 detected")
  #set(Protobuf_DEBUG ON)
ELSE () #not ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.6
  #3.5 on ubuntu 16.04
  MESSAGE ("cmake < 3.6 detected")
  set(PROTOBUF_INCLUDE_DIR ${Protobuf_INCLUDE_DIR})
  set(PROTOBUF_LIBRARY ${Protobuf_LIBRARY})
  set(PROTOBUF_PROTOC_EXECUTABLE ${Protobuf_PROTOC_EXECUTABLE})
ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.6)

find_package(Protobuf REQUIRED)
protobuf_generate_cpp(
    PROTO_SRCS
    PROTO_HDRS
    blackboard/Blackboard.proto
    blackboard/naoData.proto
    transmitter/Commands.proto
)

# protobuf stuff as a separate library so it doesn't interfere with include dirs on other files
add_library(protoswift blackboard/serialise.cpp transmitter/Commands.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(protoswift PRIVATE ${Protobuf_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})
target_include_directories(protoswift PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(protoswift ${Protobuf_LIBRARY} ${PROTOBUF_LIBRARY})

## end protobuf stuff

############################ CHECK LIBRARY / EXECUTABLE OPTION

# version.cpp dependency needs to be in this file only, otherwise robot.cmake compiles version.cpp.o before version.cpp is fully written
# surprisingly it seems ok for version.cpp to be in both libsoccer.a and libsoccer.so, and for libsoccer.so to link to libsoccer.a below
ADD_LIBRARY(soccer-static STATIC ${SOCCER_SRCS} version.cpp)
ADD_LIBRARY(soccer SHARED version.cpp)

find_package(PythonLibs 2 REQUIRED)

############################ INCLUDE DIRECTORY
include_directories("$ENV{RUNSWIFT_CHECKOUT_DIR}/utils/librcsscontroller/include")
include_directories("$ENV{RUNSWIFT_CHECKOUT_DIR}/utils/findballexp/include")
if (${CTC_DIR} MATCHES atom-2.1)
    # purposefully empty
    # when CTC_DIR is undefined, "${CTC_DIR} MATCHES atom-2.1" is false (expected)
    # when CTC_DIR is undefined, "NOT ${CTC_DIR} MATCHES atom-2.1" is also false (unexpected)
else(${CTC_DIR} MATCHES atom-2.1)
    # for toolchain 2.8 and no toolchain
    include_directories(SYSTEM "$ENV{RUNSWIFT_CHECKOUT_DIR}/robot/tiny-dnn")
    include_directories(SYSTEM "$ENV{RUNSWIFT_CHECKOUT_DIR}/robot/tiny-jnn")
endif (${CTC_DIR} MATCHES atom-2.1)

# Define include directories here
set_source_files_properties(
   perception/behaviour/python/PythonSkill.cpp
   perception/behaviour/BehaviourAdapter.cpp
   perception/behaviour/python/RobotModule.cpp
   perception/behaviour/python/RegisterConverters.cpp
   PROPERTIES COMPILE_FLAGS "-I${PYTHON_INCLUDE_DIR}")

SET_TARGET_PROPERTIES(soccer-static PROPERTIES OUTPUT_NAME "soccer")
SET_TARGET_PROPERTIES(soccer-static PROPERTIES PREFIX "lib")
SET_TARGET_PROPERTIES(soccer PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(soccer-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

ADD_CUSTOM_COMMAND ( OUTPUT version.cpp
   COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../bin/genversion.sh > version.cpp
)

SET ( SWIG_DEPENDENCIES
   perception/behaviour/python/robot.i
   perception/behaviour/python/eigen.i
   utils/body.hpp
   utils/boostSerializationVariablesMap.hpp
   utils/SPLDefs.hpp
   utils/speech.hpp
   perception/kinematics/Parameters.hpp
   perception/vision/camera/CameraDefinitions.hpp
   perception/kinematics/Pose.hpp
   gamecontroller/RoboCupGameControlData.hpp
   types/BehaviourRequest.hpp
   types/Point.hpp
   types/PPoint.hpp
   types/BBox.hpp
   types/ActionCommand.hpp
   types/ButtonPresses.hpp
   types/Odometry.hpp
   types/JointValues.hpp
   types/SensorValues.hpp
   types/RRCoord.hpp
   types/AbsCoord.hpp
   types/BroadcastData.hpp
   types/BallInfo.hpp
   types/RobotVisionInfo.hpp
   types/FieldBoundaryInfo.hpp
   types/FieldFeatureInfo.hpp
   blackboard/Blackboard.hpp
)

ADD_DEFINITIONS( -DEIGEN_DONT_ALIGN )

#ADD_CUSTOM_COMMAND ( OUTPUT RobotModule.cpp
#   COMMAND swig2.0 -Wextra -w509 -python -c++ -I${CMAKE_CURRENT_SOURCE_DIR} -o RobotModule.cpp ${CMAKE_CURRENT_SOURCE_DIR}/perception/behaviour/python/robot.i
#        && patch robot.py ${CMAKE_CURRENT_SOURCE_DIR}/perception/behaviour/python/robot.py.patch
#        && patch RobotModule.cpp ${CMAKE_CURRENT_SOURCE_DIR}/perception/behaviour/python/RobotModule.cpp.patch
#        && mv robot.py ${CMAKE_CURRENT_SOURCE_DIR}/../image/home/nao/data/behaviours/robot.py
#   DEPENDS ${SWIG_DEPENDENCIES}
#)

#SET_SOURCE_FILES_PROPERTIES( RobotModule.cpp PROPERTIES GENERATED TRUE )
SET_SOURCE_FILES_PROPERTIES( version.cpp PROPERTIES GENERATED TRUE )
SET_SOURCE_FILES_PROPERTIES( log.cpp PROPERTIES GENERATED TRUE )

############################ SET LIBRARIES TO LINK WITH
# Add any 3rd party libraries to link each target with here
find_package(Boost COMPONENTS system python program_options thread serialization regex unit_test_framework REQUIRED)
find_package(ZLIB    REQUIRED)
find_package(BZip2   REQUIRED)
find_package(PNG     REQUIRED)
find_library(JPEG_LIBRARY NAMES jpeg HINTS ${CTC_DIR}/jpeg/lib)

# TODO: What is Threads and why is it required? It's not part of Boost...
#find_package(Threads REQUIRED)

SET ( RUNSWIFT_BOOST  ${Boost_SYSTEM_LIBRARY}
                      ${Boost_REGEX_LIBRARY}
                      ${Boost_THREAD_LIBRARY}
                      ${Boost_PROGRAM_OPTIONS_LIBRARY}
                      ${Boost_SERIALIZATION_LIBRARY}
                      ${Boost_PYTHON_LIBRARY} )

TARGET_LINK_LIBRARIES(
   soccer-static

   protoswift

   pthread

   ${BZIP2_LIBRARIES}
   ${JPEG_LIBRARY}
   ${PNG_LIBRARIES}
   ${PYTHON_LIBRARY}
   ${RUNSWIFT_BOOST}
   ${ZLIB_LIBRARIES}
)

TARGET_LINK_LIBRARIES(
   soccer
   soccer-static
)
