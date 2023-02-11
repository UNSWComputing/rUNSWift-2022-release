#ifndef Q_MOC_RUN
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#endif
#include "types/LastSecondInfo.hpp"
#include "types/Ipoint.hpp"
#include "types/PostInfo.hpp"
#include "types/FootInfo.hpp"

#include "soccer.hpp"

template<class T>
const T& Blackboard::read(const T *component) {
    return *component;
}

template<class T>
void Blackboard::write(T *component, const T& value) {
    *component = value;
}

/* ============================================================================
 *                     BACKWARDS-COMPATIBLE SERIALISATION
 * ============================================================================
 *             IF YOU ARE MODIFYING BLACKBOARD, PLEASE READ THIS!
 *
 * When changing the Blackboard (i.e. adding, removing, or changing the type of
 * a variable), there a few simple steps that must be taken to ensure that the
 * new Blackboard instance is backwards compatible. Otherwise, offnao (or other
 * dump readers) will not be able to interpret dumps prior to the change.
 *
 * FOR ANY CHANGES
 * 1. Increment the Boost class version (defined below):
 *     e.g.
 *             BOOST_CLASS_VERSION(Blackboard, n);
 *     Becomes:
 *             BOOST_CLASS_VERSION(Blackboard, n+1);
 *
 * ADDING A VARIABLE
 * 1. Increment the Boost class version (see above)
 * 2. Add the variable to the 'shallowSerialize' function with a version
 * conditional, e.g.
 *
 *     if (version > n)   // where n is the pre-increment version
 *     {
 *         ar & x;        // where x is the new variable
 *     }
 *
 * REMOVING A VARIABLE
 * 1. Increment the Boost class version (see above)
 * 2. Add a version conditional to the variable in the 'shallowSerialize'
 * function, e.g.
 *
 *     if (version <= n)  // where n is the pre-increment version
 *     {
 *         T tmp;
 *         ar & tmp;      // this is essentially a default/junk value
 *     }
 *
 * CHANGING THE TYPE OF A VARIABLE
 * 1. Increment the Boost class version (see above)
 * 2. Add a version conditional to the variable in the 'shallowSerialize'
 * function, e.g.
 *
 *     if (version <= n)  // where n is the pre-increment version
 *     {
 *          T1 tmp;
 *          ar & tmp;
 *          x = (T2)tmp;  // assuming T1 can be type-cast converted
 *                        // to T2 (conversion may be more complicated)
 *     }
 *     else
 *     {
 *          ar & x;
 *     }
 *
 * GENERAL COMMENT
 * The above guidelines must be followed when modifying the Blackboard.
 *
 * To reviewers: if you are reviewing code that has not followed the above
 * guidelines, please ask the author to make the appropriate changes before
 * approving the pull request.
 *
 * For more info, see the Wiki page titled 'Serialization'.
 * @see https://github.com/UNSWComputing/rUNSWift/wiki/Blackboard-Serialization
 */
BOOST_CLASS_VERSION(Blackboard, 28);

template<class Archive>
void Blackboard::shallowSerialize(Archive & ar, const unsigned int version) {

    if (version < 15) {
        throw std::runtime_error("Depricated 2011 dump file detected");
    }

    if (version < 26) {
       bool team_red = false;
       ar & team_red;
    }
    ar & gameController.player_number;

    ar & motion.sensors;
    ar & motion.pose;
    ar & motion.com;
    ar & motion.odometry;
    ar & motion.active;

    ar & perception.behaviour;
    ar & perception.kinematics;
    ar & perception.stateEstimation;
    ar & perception.total;
    ar & perception.vision;

    // This request was updated for version 19 but not sure if more is required
    ar & behaviour.request;

    // ar & kinematics.sonarFiltered;
    ar & kinematics.parameters;

    if (version < 28)
    {
        SensorValues tmpSensorValues;
        ar & tmpSensorValues;
    }

    if (1) {//(this->mask & ROBOT_FILTER_MASK) {
        ar & stateEstimation.robotObstacles;
    } else {
        std::vector<RobotVisionInfo> empty;
        ar & empty;
    }

    if (version < 27)
    {
        std::vector<Ipoint> tmpLandmarks;
        ar & tmpLandmarks;
    }

    /* Only serialise the things below if WHITEBOARD_MASK is not set.
     * We also ONLY want this to happen when we are serialising in Offnao,
     * which occurs when we save the dump. WHITEBOARD_MASK can only be set
     * in the save function in offnao.
     */
    if (!(this->mask & WHITEBOARD_MASK)){
        ar & vision.timestamp;
        if (version < 27)
        {
            PostInfo::Type tmpPostInfoType(PostInfo::pNone);
            ar & tmpPostInfoType;
            float tmpFloat;
            ar & tmpFloat;
            int tmpInt;
            ar & tmpInt;
            ar & tmpInt;
            std::vector<FootInfo> tmpFeetBoxes;
            ar & tmpFeetBoxes;
        }
        ar & vision.balls;
        if (version < 27)
        {
            BallHint tmpBallHint;
            ar & tmpBallHint;
            std::vector<PostInfo> tmpPosts;
            ar & tmpPosts;
        }
        ar & vision.robots;
        ar & vision.fieldBoundaries;
        ar & vision.fieldFeatures;
        if (version < 27)
        {
            unsigned int tmpMissedFrames;
            ar & tmpMissedFrames;
            std::pair<int, int> tmpdxdy;
            ar & tmpdxdy;
        }

        if(version >= 17) {
            ar & vision.regions;
        }
    }

    ar & vision.topCameraSettings;
    ar & vision.botCameraSettings;

    if (version >= 17 && version < 27)
    {
        LastSecondInfo tmpLastSecond;
        ar & tmpLastSecond;
    }

    ar & receiver.message;
    ar & receiver.data;
    if (version >= 25)
        ar & receiver.lastReceived;
    ar & receiver.incapacitated;


    ar & stateEstimation.robotPos;
    ar & stateEstimation.allRobotPos;
    if (version < 24) {
        uint32_t tmpBallLostCount;
        ar & tmpBallLostCount;
        if(version >= 18) {
            uint32_t tmpBallSeenCount;
            ar & tmpBallSeenCount;
        }
    }

    ar & stateEstimation.ballPosRR;
    ar & stateEstimation.ballPosRRC;
    ar & stateEstimation.ballVelRRC;
    ar & stateEstimation.ballVel;
    ar & stateEstimation.ballPos;
    ar & stateEstimation.teamBallPos;
    ar & stateEstimation.teamBallVel;
    ar & stateEstimation.sharedStateEstimationBundle;

    ar & stateEstimation.havePendingOutgoingSharedBundle;
    ar & stateEstimation.havePendingIncomingSharedBundle;
}

// not the same as load, so we can lock when we serialise the blackboard and saliency images
template<class Archive>
void Blackboard::save(Archive & ar, const unsigned int version) const {
    // note, version is always the latest when saving
    OffNaoMask_t mask = this->mask;
    if ((mask & SALIENCY_MASK) && (!vision.topSaliency || !vision.botSaliency))
        mask &= (~SALIENCY_MASK);
    if ((mask & RAW_IMAGE_MASK) && (!vision.topFrame || !vision.botFrame))
        mask &= (~RAW_IMAGE_MASK);
    ar & boost::serialization::make_nvp("Mask", mask);

    if ((mask & BLACKBOARD_MASK) && (mask & SALIENCY_MASK)) {
        locks.serialization->lock();
        ((Blackboard*)this)->shallowSerialize(ar, version);
        // TODO(jayen): RLE
        ar & boost::serialization::
        make_nvp(
            "TopSaliency",
            boost::serialization::
            make_binary_object(vision.topSaliency, sizeof(TopSaliency))
        );
        ar & boost::serialization::
        make_nvp(
            "BotSaliency",
            boost::serialization::
            make_binary_object(vision.botSaliency, sizeof(BotSaliency))
        );
        locks.serialization->unlock();
    } else if (mask & BLACKBOARD_MASK) {
        ((Blackboard*)this)->shallowSerialize(ar, version);
    } else if (mask & SALIENCY_MASK) {
        // TODO(jayen): RLE
        ar & boost::serialization::
        make_nvp(
            "TopSaliency",
            boost::serialization::
            make_binary_object(vision.topSaliency, sizeof(TopSaliency))
        );
        ar & boost::serialization::
        make_nvp(
            "BotSaliency",
            boost::serialization::
            make_binary_object(vision.botSaliency, sizeof(BotSaliency))
        );
    }
    if (mask & RAW_IMAGE_MASK) {
        // TODO(jayen): zlib
        ar & boost::serialization::make_nvp(
            "Top Raw Image",
            boost::serialization:: make_binary_object((void *)vision.topFrame,
            sizeof(uint8_t[TOP_IMAGE_ROWS * TOP_IMAGE_COLS * 2]))
        );
        ar & boost::serialization::make_nvp(
            "Bot Raw Image",
            boost::serialization:: make_binary_object((void *)vision.botFrame,
            sizeof(uint8_t[BOT_IMAGE_ROWS * BOT_IMAGE_COLS * 2]))
        );
    }

    ar & stateEstimation.robotPos;
}

template<class Archive>
void Blackboard::load(Archive & ar, const unsigned int version) {
    ar & mask;
    if (mask & BLACKBOARD_MASK)
        shallowSerialize(ar, version);
    if (mask & SALIENCY_MASK) {
       vision.topSaliency = (Colour *) new TopSaliency;
        // TODO(jayen): RLE
       ar & boost::serialization::make_binary_object(vision.topSaliency, sizeof(TopSaliency));

       vision.botSaliency = (Colour *) new BotSaliency;
        // TODO(jayen): RLE
       ar & boost::serialization::make_binary_object(vision.botSaliency, sizeof(BotSaliency));
    }
    if (mask & RAW_IMAGE_MASK) {
        vision.topFrame = new uint8_t[TOP_IMAGE_ROWS * TOP_IMAGE_COLS * 2];
        ar & boost::serialization::
        make_binary_object((void *)vision.topFrame, sizeof(uint8_t[TOP_IMAGE_ROWS * TOP_IMAGE_COLS * 2]));
        vision.botFrame = new uint8_t[BOT_IMAGE_ROWS * BOT_IMAGE_COLS * 2];
        ar & boost::serialization::
        make_binary_object((void *)vision.botFrame, sizeof(uint8_t[BOT_IMAGE_ROWS * BOT_IMAGE_COLS * 2]));
    }

    ar & stateEstimation.robotPos;
}

namespace boost {
    namespace serialization {
        /* // boost docs are broken
        template<class Archive>
        inline void save_construct_data(Archive &ar, const Blackboard *t, const unsigned int file_version) {
            // save data required to construct instance
            ar << t->config;
        }
        */
        template<class Archive>
        inline void load_construct_data(Archive &ar, Blackboard *t, const unsigned int file_version) {
            // retrieve data from archive required to construct new instance
            boost::program_options::variables_map config;
            ar >> config;
            // invoke inplace constructor to initialize instance of Blackboard
            ::new(t) Blackboard(config);
        }
    }
}
