#include <fstream>
#include <limits>
#include <cctype>
#include <unistd.h>
#include <limits.h>
#include <sstream>
#include "motion/generator/GetupGenerator.hpp"
#include "utils/Logger.hpp"
#include "utils/angles.hpp"
#include "utils/speech.hpp"
#include <boost/algorithm/string.hpp>
#include "../MotionDefs.hpp"

using namespace std;
using boost::program_options::variables_map;

#define MIN_BATTERY_CHARGE_TO_DO_FAST_GETUP_FRONT 0.5
#define MIN_BATTERY_CHARGE_TO_DO_FAST_GETUP_BACK 0.4

// An estimate of how many frames it takes a joint to achieve the position
// after requested (on V5). Found by comparing the current sensor value of a joint,
// and the previous joint requests it has sent to the robot.
#define HYP_CYCLIC_BUFFER_LENGTH 4

// A factor to multiply the difference in hyp before adding to the hip pitch.
// Found empirically
#define HYP_TO_HP_MULTIPLY_FACTOR 0.7

GetupGenerator::GetupGenerator(std::string falldirection)
    : isFrontGetup(falldirection == "FRONT")
{
    max_iter = 0;
    current_time = NOT_RUNNING;

    // Initialise cyclic buffer variables
    for (unsigned i = 0; i < HYP_CYCLIC_BUFFER_LENGTH; ++i)
        hypCyclicBuffer.push_back(0);
    hypCyclicBufferIndex = 0;
};

GetupGenerator::~GetupGenerator()
{
    llog(INFO) << "GetupGenerator destroyed" << std::endl;
};

bool GetupGenerator::isActive()
{
    return current_time != NOT_RUNNING;
};

void GetupGenerator::reset()
{
    current_time = 0;
    motionOdometry.reset();
}

void GetupGenerator::stop()
{
}

JointValues GetupGenerator::makeJoints(ActionCommand::All *request,
                                       Odometry *odometry,
                                       const SensorValues &sensors,
                                       BodyModel &bodyModel,
                                       float ballX,
                                       float ballY,
                                       MotionDebugInfo &motionDebugInfo)
{
    JointValues j;
    if (current_time == NOT_RUNNING)
    {
        active = request->body;
        j = joints[joints.size() - 1];
    }
    else
    {
        JointValues newJoints = sensors.joints;
        for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++)
        {
            newJoints.stiffnesses[i] = 1.0f;
        }
        if (current_time == 0)
        {
            // Get latest batterycharge, and choose getup
           chooseGetup(sensors.sensors[Sensors::Battery_Charge], sensors.joints.angles);
            joints.clear();
            constructPose(path, individualPath, bodyName);

            // Interpolate joints
            interpolate(newJoints);
        }
        j = joints[current_time++];
        if (current_time == (signed int)joints.size()) // if we just did last action
        {
            current_time = NOT_RUNNING;
        }
    }

    // Update odometry, (important for gyroscopez)
    *odometry = *odometry + motionOdometry.updateOdometry(sensors, Odometry());

    // The HipYawPitch of the robot isn't a super powerful joint, and can sometimes
    // not achieve the requested position. This can cause a huge offset of >20-30 degrees
    // between the requested and actual joint position.
    // An example case where this happens is in a fast front getup, where the robot, is trying to bring its
    // body upright, and trying to close its legs together at the same time. If both feet are on
    // the ground, closing the hipyawpitch doesn't work. This results in the robot torso not
    // achieving an upright position. To keep the torso in the expected position, we add an
    // offset to the hippitch joints, depending on the difference between the requested and joint
    // value of the hipyawpitch.
    // The cyclic buffer accounts for the delay between requesting a joint value and the joint actually
    // achieving the position. This allows time for the joints to move after being requested, without
    // being falsely recognised as a "failure to move hipyawpitch".

    // Find error between expected and read lhipyawpitch, and add to hip pitch
    float difference = hypCyclicBuffer[hypCyclicBufferIndex] - sensors.joints.angles[Joints::LHipYawPitch];

    // Add offsets to the LHipPitch and RHipPitch
    j.angles[Joints::LHipPitch] += HYP_TO_HP_MULTIPLY_FACTOR * difference;
    j.angles[Joints::RHipPitch] += HYP_TO_HP_MULTIPLY_FACTOR * difference;

    // Add new request value to cyclic buffer
    hypCyclicBuffer[hypCyclicBufferIndex] = j.angles[Joints::LHipYawPitch];
    hypCyclicBufferIndex = (hypCyclicBufferIndex + 1) % HYP_CYCLIC_BUFFER_LENGTH;

    return j;
};

void GetupGenerator::interpolate(JointValues newJoint, int duration)
{
    if (joints.empty())
    {
        max_iter = duration / (1000.0 * MOTION_DT);
        // Reserve space for the interpolation when the generator
        // first called
        for (int i = 0; i < max_iter; i++)
        {
            joints.push_back(newJoint);
        }
        joints.push_back(newJoint);
    }
    else
    {
        int inTime = 0;
        float offset[Joints::NUMBER_OF_JOINTS];

        if (duration != 0)
        {
            inTime = duration / (1000.0 * MOTION_DT);
            JointValues currentJoint = joints.back();

            // Calculate the difference between new joint and the previous joint
            for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++)
            {
                offset[i] = (newJoint.angles[i] - currentJoint.angles[i]) / inTime;
            }

            for (int i = 0; i < inTime; i++)
            {
                JointValues inJoint;
                for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++)
                {
                    inJoint.angles[j] = joints.back().angles[j] + offset[j];
                    inJoint.stiffnesses[j] = newJoint.stiffnesses[j];
                }
                joints.push_back(inJoint);
            }
        }
        else
        {

            JointValues firstJoint = joints.at(max_iter);
            // Calculate the difference between the joint at MAX_ITER position
            // with the new joint
            for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++)
            {
                offset[i] = (firstJoint.angles[i] - newJoint.angles[i]) / max_iter;
            }

            joints[0] = newJoint;
            for (int i = 1; i < max_iter; i++)
            {
                for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++)
                {
                    joints[i].angles[j] = joints[i - 1].angles[j] + offset[j];
                    joints[i].stiffnesses[j] = firstJoint.stiffnesses[j];
                }
            }
        }
    }
}

bool GetupGenerator::individualPathExists(std::string individualPath, std::string bodyName)
{
    ifstream ifs;
    ifs.open(string(individualPath + "/" + bodyName + "_" + file_name + ".pos").c_str());

    if (!ifs.is_open())
    {
        return false;
    }
    else
    {
        llog(INFO) << "GetupGenerator: Individual file path found." << endl;
        ifs.close();
        return true;
    }
}

void GetupGenerator::constructPose(std::string path, std::string individualPath, std::string bodyName)
{

    /*
    * Check for individual pos files, if they exist.
    * Getups require the entire body of the nao, so individual pos fles exist
    * to account for multiple joint offsets. If no file exists for them,
    * then simply use the default pos file.
    */

    bool useIndividalPath = individualPathExists(individualPath, bodyName);

    ifstream in;
    if (useIndividalPath)
    {
        in.open(string(individualPath + "/" + bodyName + "_" + file_name + ".pos").c_str());
    }
    else
    {
        in.open(string(path + "/" + file_name + ".pos").c_str());
    }

    llog(INFO) << "GetupGenerator(" << file_name << ") creating" << endl;

    if (!in.is_open())
    {
        llog(FATAL) << "GetupGenerator cannot open " << file_name << endl;
    }
    else
    {
        JointValues newJoint(0);

        // Set to default 1 stiffness
        for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
            newJoint.stiffnesses[i] = 1.0f;

        while (!in.eof())
        {

            std::string line;
            std::getline(in, line);

            if (boost::starts_with(line, "!"))
            {
                boost::erase_all(line, "!");

                std::istringstream ss(line);

                float angle;
                for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
                {
                    ss >> angle;
                    newJoint.angles[i] = DEG2RAD(angle);
                }

#ifdef CTC_2_1
                // If V5, we have to move the LHand
                float angleLHand = newJoint.angles[Joints::LHand];
                for (unsigned i = V6Joints::LHand; i > Joints::LHand; --i)
                {
                    newJoint.angles[i] = newJoint.angles[i - 1];
                }
                newJoint.angles[Joints::LHand] = angleLHand;
#endif

                int duration;
                ss >> duration;

                interpolate(newJoint, duration);

                // Set to default 1 stiffness
                for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
                    newJoint.stiffnesses[i] = 1.0f;
            }

            if (boost::starts_with(line, "$"))
            {
                boost::erase_all(line, "$");

                std::istringstream ss(line);

                float stiffness;
                for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
                {
                    ss >> stiffness;
                    newJoint.stiffnesses[i] = stiffness;
                }

#ifdef CTC_2_1
                // If V5, we have to move the LHand
                float stiffnessLHand = newJoint.stiffnesses[Joints::LHand];
                for (unsigned i = V6Joints::LHand; i > Joints::LHand; --i)
                {
                    newJoint.stiffnesses[i] = newJoint.stiffnesses[i - 1];
                }
                newJoint.stiffnesses[Joints::LHand] = stiffnessLHand;
#endif // CTC_2_1
            }
        }
        in.close();
    }
    llog(INFO) << "GetupGenerator(" << file_name << ") created" << endl;
}

void GetupGenerator::chooseGetup(float batteryCharge, const float angles[Joints::NUMBER_OF_JOINTS]) {
//   std::cerr << __FILE__ << "\t" << __LINE__ << "\tisFrontGetup:\t" << isFrontGetup << "\n";
    if (isFrontGetup)
    {
       // if shoulder pitches are forwards, and elbow rolls are inwards seem like they are stuck on the ground
//       Joints::outputValues(std::cerr, angles);
//        if ((angles[Joints::LShoulderPitch] < DEG2RAD(86) && (angles[Joints::LElbowRoll] < DEG2RAD(-10) || angles[Joints::LShoulderRoll] < DEG2RAD(-10))) ||
//            (angles[Joints::RShoulderPitch] < DEG2RAD(86) && (angles[Joints::RElbowRoll] > DEG2RAD(10) || angles[Joints::RShoulderRoll] > DEG2RAD(10)))) {
// //          std::cerr << __FILE__ << "\t" << __LINE__ << "\tarms seem stuck!!!  trying to unstick"  << "\n";
//           file_name = "unstickArms";
//        } else {
      if (getupSpeed == "FAST") {
          if (batteryCharge > MIN_BATTERY_CHARGE_TO_DO_FAST_GETUP_FRONT)
            file_name = "getupFrontFast";
          else
            file_name = "getupFront";
      } else if (getupSpeed == "MODERATE") {
          file_name = "getupFront";
      } else if (getupSpeed == "SLOW") {
          file_name = "getupFrontSlow";
      }
      //  }
    }
    else
    {
        if (getupSpeed == "FAST")
        {
            if (batteryCharge > MIN_BATTERY_CHARGE_TO_DO_FAST_GETUP_BACK)
                #ifdef CTC_2_1
                    file_name = "getupBackFast";
                #else
                    file_name = "getupBackFastV6";
                #endif // CTC_2_1
            else
                file_name = "getupBack";
        }
        else if (getupSpeed == "MODERATE")
        {
            file_name = "getupBack";
        }
        else if (getupSpeed == "SLOW")
        {
            file_name = "getupBackSlow";
        }
    }
}

void GetupGenerator::readOptions(const boost::program_options::variables_map &config)
{
    path = config["motion.path"].as<std::string>();
    individualPath = config["motion.individualConfigPath"].as<std::string>();
    bodyName = config["player.bodyName"].as<std::string>();
    getupSpeed = config["motion.getup_speed"].as<std::string>();
}
