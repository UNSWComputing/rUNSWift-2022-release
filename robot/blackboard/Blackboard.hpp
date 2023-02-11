#pragma once

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/function.hpp>
#endif
#include <signal.h>
#include <string>
#include <map>
#include <vector>
#include <deque>

#include "utils/body.hpp"
#include "utils/boostSerializationVariablesMap.hpp"
#include "perception/kinematics/Parameters.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "types/SharedStateEstimationBundle.hpp"
#include "perception/kinematics/Pose.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "utils/Logger.hpp"
#include "transmitter/TransmitterDefs.hpp"
#include "types/BehaviourRequest.hpp"

#include "types/ActionCommand.hpp"
#include "types/ButtonPresses.hpp"
#include "types/Odometry.hpp"
#include "types/SensorValues.hpp"
#include "types/RRCoord.hpp"
#include "types/AbsCoord.hpp"
#include "types/XYZ_Coord.hpp"
#include "types/SPLStandardMessage.hpp"
#include "types/BroadcastData.hpp"
#include "types/BehaviourSharedData.hpp"
#include "types/MotionDebugInfo.hpp"

#include "types/CleverNaoInfo.hpp"

#include "types/BallInfo.hpp"
#include "types/RobotVisionInfo.hpp"
#include "types/RobotObstacle.hpp"
#include "types/FieldBoundaryInfo.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/Odometry.hpp"
#include "types/CameraSettings.hpp"

#include "perception/vision/Region/Region.hpp"
#include "soccer.hpp"

#include "utils/ProtobufSerialisable.hpp"


namespace offnao
{
   class Blackboard;
}

/**
 * Macro to wrap reads to module's blackboard.
 * @param module which module's blackboard to read
 * @param component the component to read
 */
#define readFrom(module, component) \
    blackboard->read(&(blackboard->module.component))

/**
 * Macro to wrap array reads from module's blackboard.
 * Performs a memcpy on the provided arguments.
 * @param module which module's blackboard to read from
 * @param component while component to be written
 * @param dest where to write to
 */
#define readArray(module, component, dest) \
    memcpy(dest, blackboard->module.component, \
           sizeof(blackboard->module.component));

/**
 * Macro to wrap writes to module's blackboard.
 * @param module which module's blackboard to write
 * @param component while component to write
 * @param value the value to be written
 */
#define writeTo(module, component, value) \
    blackboard->write(&(blackboard->module.component), value);

/**
 * Macro to wrap array writes to module's blackboard.
 * Performs a memcpy on the provided arguments.
 * @param module which module's blackboard to write
 * @param component while component to write
 * @param value the value to be written
 */
#define writeArray(module, component, value) \
    memcpy(blackboard->module.component, value, \
           sizeof(blackboard->module.component));

/**
 * Macro to wrap acquiring a mutex on a the blackboard.
 * @param name which module's lock to acquire
 */
#define acquireLock(name) \
    (blackboard->locks.name)->lock();

/**
 * Macro to wrap releasing a mutex on the blackboard.
 * @param name which module's lock to release
 */
#define releaseLock(name) \
    (blackboard->locks.name)->unlock();

/**
 * Blackboard shared memory class, used for inter-module communication.
 * The Blackboard is friends with each of the module adapters
 * so that they can access Blackboard privates (and nobody else can)
 * The safeRun templated function has access to the blackboard
 * to look up thread timings and things like that
 */

class NetworkReader;
class OverviewTab;

struct KinematicsBlackboard {
    explicit KinematicsBlackboard();
    void readOptions(const boost::program_options::variables_map& config);
    Parameters parameters;
    std::string bodyName;
};

/* Data Behaviour module will be sharing with others */
struct BehaviourBlackboard {
    explicit BehaviourBlackboard();
    void readOptions(const boost::program_options::variables_map& config);
    BehaviourRequest request[2]; // double buffer to avoid concurrent access with motion
    int readBuf;
    std::string skill;
    std::string headskill;
    std::string positioning;
    BehaviourSharedData behaviourSharedData;
    bool remoteStiffen;
    bool useGetups;
    CleverNaoInfoBehaviour cleverNaoInfo;
};

/* Data State Estimation module will be sharing */
struct StateEstimationBlackboard {
    explicit StateEstimationBlackboard();
    void readOptions(const boost::program_options::variables_map& config);

    // Global robot position and uncertainty
    AbsCoord robotPos;
    float robotPosUncertainty;
    float robotHeadingUncertainty;
    std::vector<AbsCoord> allRobotPos;

    // Robot relative coords.
    RRCoord ballPosRR;

    // Cartesian robot relative ball coords.
    AbsCoord ballPosRRC;

    // Robot relative ball velocity.
    // TODO: make this in global coords and change behaviours correspondingly.
    // Ball Detection currently assumes this is in Robot Relative coordinates
    AbsCoord ballVelRRC;

    // Global ball velocity
    AbsCoord ballVel;

    // Global ball position
    AbsCoord ballPos;

    // Team ball position
    AbsCoord teamBallPos;

    // Team ball velocity
    AbsCoord teamBallVel;

    // Team ball position uncertainty
    float teamBallPosUncertainty;

    SharedStateEstimationBundle sharedStateEstimationBundle;

    bool havePendingOutgoingSharedBundle;
    std::vector<bool> havePendingIncomingSharedBundle;

    /** filtered positions of visual robots */
    std::vector<RobotObstacle> robotObstacles;

    // Whether we have had a recent team ball update
    bool hadTeamBallUpdate;
};

/* Data Vision module will be sharing with others */
struct VisionBlackboard {

    explicit VisionBlackboard();
    void readOptions(const boost::program_options::variables_map& config);
    /* Time the frame was captured */
    int64_t timestamp;

    /* Detected features */
    std::vector<BallInfo> balls;
    std::vector<RobotVisionInfo> robots;
    std::vector<FieldBoundaryInfo> fieldBoundaries;
    std::vector<FieldFeatureInfo> fieldFeatures;
    std::vector<RegionI> regions;

    /** Saliency scan */
    Colour *topSaliency;
    Colour *botSaliency;

    /** Pointer to the current frame being processed by Vision */
    uint8_t const* topFrame;
    uint8_t const* botFrame;

    /** JPEG compression for serialization */
    int8_t topFrameJPEGQuality;
    int8_t botFrameJPEGQuality;

    /** Current camera settings on the robot */
    CameraSettings topCameraSettings;
    CameraSettings botCameraSettings;

    // since offnao is compiled for v5, we need to send these over to offnao for the manual camera pose
   float horizontalFieldOfView;
   float verticalFieldOfView;
};

struct PerceptionBlackboard {
    explicit PerceptionBlackboard();
    uint32_t kinematics;
    uint32_t stateEstimation;
    uint32_t vision;
    uint32_t behaviour;
    uint32_t total;
};

struct GameControllerBlackboard {
    explicit GameControllerBlackboard();
    void readOptions(const boost::program_options::variables_map& config);
    bool connect;
    bool connected;
    RoboCupGameControlData data;
    TeamInfo our_team;
    int player_number;
    uint8_t gameState;
    char* lastGameControllerIPAddress;
    bool whistleDetected;
    uint8_t challengePhase;
};


struct MotionBlackboard {
    explicit MotionBlackboard();
    SensorValues sensors;
    float uptime;
    ActionCommand::All active;
    Odometry odometry;
    ButtonPresses buttons;
    Pose pose;
    XYZ_Coord com;
    JointValues jointRequest;
    MotionDebugInfo motionDebugInfo;
    CleverNaoInfoMotion cleverNaoInfo;
    bool isStiff;
};

struct RemoteControlBlackboard {
    explicit RemoteControlBlackboard();
    BehaviourRequest request;
    time_t time_received;
};

struct ReceiverBlackboard {
    explicit ReceiverBlackboard();
    // one for each robot on the team
    SPLStandardMessage message[ROBOTS_PER_TEAM];
    BroadcastData data[ROBOTS_PER_TEAM];
    int team;
    void readOptions(const boost::program_options::variables_map& config);
    time_t lastReceived[ROBOTS_PER_TEAM];
    std::vector<bool> incapacitated;
};


struct SynchronisationBlackboard {
    explicit SynchronisationBlackboard();
    boost::mutex *buttons;
    boost::mutex *serialization;
};

struct ThreadBlackboard {
    explicit ThreadBlackboard();
    std::map<std::string, boost::function<void(const boost::program_options::variables_map &)> > configCallbacks;
};

class Blackboard : ProtobufSerialisable {
    friend class boost::serialization::access;
    template<class Archive> friend void
    boost::serialization::save_construct_data(Archive &ar, const Blackboard *t, const unsigned int) {
        // TODO(jayen): move code out of header file - http://stackoverflow.com/questions/4112075/trouble-overriding-save-construct-data-when-serializing-a-pointer-to-a-class-with
        // save data required to construct instance
        ar << t->config;
    }

    // Functions
    template <class T> friend void *safelyRun(void * foo);

    public:
        explicit Blackboard();
        explicit Blackboard(const boost::program_options::variables_map &vm);
        virtual ~Blackboard();

        /* Function to read a component from the Blackboard */
        template<class T> const T& read(const T *component);

        /* Write a component to the Blackboard */
        template<class T> void write(T *component, const T& value);

        /**
         * helper for serialization
         */
        template<class Archive>
        void shallowSerialize(Archive &ar, const unsigned int version);
        /**
         * serialises the blackboard for storing to a file or network
         */
        template<class Archive>
        void save(Archive &ar, const unsigned int version) const;
        /**
         * serialises the blackboard for loading from a file or network
         */
        template<class Archive>
        void load(Archive &ar, const unsigned int version);

        /**
         * serialises the blackboard with protobuf for storing to a file
         * or network
         */
        void serialise(std::ostream&) const;
        static void serialise(const Blackboard &cpp, offnao::Blackboard &pb);

        /**
         * deserialises the blackboard with protobuf for loading from a file
         * or network
         */
        void deserialise(std::istream&);
        static void deserialise(Blackboard &cpp, const offnao::Blackboard &pb);
    #ifndef SWIG
            BOOST_SERIALIZATION_SPLIT_MEMBER();
    #endif

        /* We now have a private inner-class blackboard for each of the
         * modules. Appropriate constructors should be placed in Blackboard.cpp
         * since there's no guarantee as to which order threads start in. */

        // TODO(find a better solution) private:

        /* Stores command-line/config-file options
         * should be read by modules on start
         * functionality may be added later to allow change at runtime */
        boost::program_options::variables_map config;

        /**
         * the mask of what is stored/loaded from a file or network
         */
        OffNaoMask_t mask;

        /* Options callback for changes at runtime */
        void readOptions(const boost::program_options::variables_map& config);

        /* Data Kinematics module will be sharing with others */
        KinematicsBlackboard kinematics;

        /* Data Behaviour module will be sharing with others */
        BehaviourBlackboard behaviour;

        /* Data Localisation module will be sharing */
        StateEstimationBlackboard stateEstimation;

        /* Data Vision module will be sharing with others */
        VisionBlackboard vision;

        PerceptionBlackboard perception;

        /* Data GameController will be sharing */
        GameControllerBlackboard gameController;

        /* Data Motion module will be sharing with others */
        MotionBlackboard motion;

        /* Data received from friendly robots */
        ReceiverBlackboard receiver;

        /* Data received from remote-control piece in Off-Nao */
        RemoteControlBlackboard remoteControl;
        /* Data ThreadWatcher will be sharing with others */
        ThreadBlackboard thread;

        /* Locks used for inter-thread synchronisation */
        SynchronisationBlackboard locks;
};

#include "Blackboard.tcc"
