#include "perception/vision/VisionAdapter.hpp"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <sys/time.h>        /* For gettimeofday */
#include <pthread.h>
#include <vector>

#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "utils/Timer.hpp"
#include "types/CombinedCameraSettings.hpp"
#include "types/CombinedFrame.hpp"

using namespace std;
using namespace boost::algorithm;

extern int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP;
extern int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT;
extern int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP;
extern int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT;
int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP;
int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT;
int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP;
int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT;

VisionAdapter::VisionAdapter(Blackboard *bb)
   : Adapter(bb),
     vision_()
{
    combined_camera_ = new CombinedCamera(
        (blackboard->config)["vision.dumpframes"].as<bool>(),
        (blackboard->config)["vision.dumprate"].as<int>(),
        (blackboard->config)["vision.dumpfile"].as<string>()
    );

    combined_frame_ = boost::shared_ptr<CombinedFrame>();

    RegionI topRegion = vision_.getFullRegionTop();
    RegionI botRegion = vision_.getFullRegionBot();
    const Fovea *foveaTop = topRegion.getInternalFovea();
    const Fovea *foveaBot = botRegion.getInternalFovea();

    ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP = (blackboard->config)["vision.top.adaptivethresholdingwindow"].as<int>();
    ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT = (blackboard->config)["vision.bot.adaptivethresholdingwindow"].as<int>();
    ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP = (blackboard->config)["vision.top.adaptivethresholdingpercent"].as<int>();
    ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT = (blackboard->config)["vision.top.adaptivethresholdingpercent"].as<int>();

    writeTo(vision, topSaliency, (Colour*)foveaTop->getInternalColour());
    writeTo(vision, botSaliency, (Colour*)foveaBot->getInternalColour());
}

void VisionAdapter::tick() {
    Timer t;
    uint32_t time;

    /*
     * Camera Tick
     */
    llog_open(VERBOSE) << "Vision Camera Tick" << endl;
    t.restart();
    tickCamera();
    time = t.elapsed_us();
    if (time < 30000) {
        llog_close(VERBOSE) << "Vision Camera Tick: OK " << time << " us" << endl;
    } else {
        llog_close(ERROR) << "Vision Camera Tick: TOO LONG " << time << " us" << endl;
    }

    /*
     * Process Tick
     */
    llog_open(VERBOSE) << "Vision Process Tick" << endl;
    t.restart();
    tickProcess();
    time = t.elapsed_us();
    if (time < 30000) {
        llog_close(VERBOSE) << "Vision Process Tick: OK " << time << " us" << endl;
    } else {
        llog_close(ERROR) << "Vision Process Tick: TOO LONG " << time << " us" << endl;
    }

}

void VisionAdapter::tickCamera() {
    if (combined_camera_ == NULL) {
        llog_middle(WARNING) << "No Camera provided to the VisionAdapter" << endl;
    } else {
         writeTo(vision, topFrame, combined_camera_->getFrameTop());
         writeTo(vision, botFrame, combined_camera_->getFrameBottom());

         // Write the camera settings to the Blackboard
         // for syncing with OffNao's camera tab
         CombinedCameraSettings settings = combined_camera_->getCameraSettings();
         writeTo(vision, topCameraSettings, settings.top_camera_settings);
         writeTo(vision, botCameraSettings, settings.bot_camera_settings);
    }
}

void VisionAdapter::tickProcess() {
    Timer t;
    VisionInfoIn info_in;

    // Used by the Python WallTimer.py
    // Might not belong here, just quick fixing Ready skill for now.
    struct timeval tv;
    gettimeofday(&tv, 0);

    int64_t vision_timestamp = tv.tv_sec * 1e6 + tv.tv_usec;

    // TODO Read current Pose from blakboard
    conv_rr_.pose = readFrom(motion, pose);
    conv_rr_.updateAngles(readFrom(motion, sensors));

    info_in.cameraToRR = conv_rr_;
    info_in.pose = conv_rr_.pose;
    info_in.robotPose = readFrom(stateEstimation, robotPos);

    // Set latestAngleX.
    info_in.latestAngleX =
              readFrom(motion, sensors).sensors[Sensors::InertialSensor_AngleX];

    // TODO ADD THIS IN
    conv_rr_.findEndScanValues();

    /*
     * Acquire blackboard lock
     */
    acquireLock(serialization);
    llog_middle(VERBOSE) << "Vision tickProcess acquireLock(serialization) took " << t.elapsed_us()
      << " us" << endl;
    t.restart();

    /*
     * Reading from Blackboard into VisionBlackboardInfo
     */
    // NOTE: You can add things to info_in below by going
    // NOTE: info_in.robotDetection.sonar = readFrom(kinematics, sonarFiltered)

    info_in.top_camera_settings = readFrom(vision, topCameraSettings);
    info_in.bot_camera_settings = readFrom(vision, botCameraSettings);

    boost::shared_ptr<CombinedFrame> combined_frame_;
    combined_frame_ = boost::shared_ptr<CombinedFrame>(new CombinedFrame(
        readFrom(vision, topFrame),
        readFrom(vision, botFrame),
        conv_rr_,
        combined_frame_
    ));

    llog_middle(VERBOSE) << "Vision reading images from blackboard took " << t.elapsed_us()
      << " us" << endl;

    t.restart();

    /*
     * Running Process Frame
     */
    llog(VERBOSE) << "Vision reading remaining data from blackboard took " << t.elapsed_us() << " us" << endl;
    t.restart();

    pthread_yield();
    usleep(1); // force sleep incase yield sucks

    VisionInfoOut info_out = vision_.processFrame(*(combined_frame_.get()), info_in);

    llog(VERBOSE) << "Vision processFrame() took " << t.elapsed_us() << " us" << endl;
    t.restart();

    /*
     * Writing Results back to blackboard
     */

    // NOTE: You can add things back to the blackboard by going
    // NOTE: writeTo(vision, [blackboard var name], info_out.[info_in var name])

    writeTo (vision, timestamp,       vision_timestamp        );
    // Note that these regions will not be able to access their underlying pixel
    // data.
    writeTo (vision, regions,         info_out.regions        );
    writeTo (vision, balls,           info_out.balls          );
    writeTo (vision, robots,          info_out.robots         );
    writeTo (vision, fieldFeatures,   info_out.features       );
    writeTo (vision, fieldBoundaries, info_out.boundaries     );

    releaseLock(serialization);
    llog(VERBOSE) << "Vision writing back to blackboard took " << t.elapsed_us() << " us" << endl;
}
