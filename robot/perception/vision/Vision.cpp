#include <list>

#include "perception/vision/Vision.hpp"
#include "perception/vision/detector/RegionFieldFeatureDetector.hpp"
#include "perception/vision/middleinfoprocessor/FieldBoundaryFinder.hpp"
#include "types/CombinedFovea.hpp"
#include "types/CombinedFrame.hpp"
#include "utils/Logger.hpp"
#include "perception/vision/Fovea.hpp"
#include "perception/vision/regionfinder/ColourROI.hpp"
#include "perception/vision/detector/BallDetector.hpp"
#include "perception/vision/detector/RobotDetector_fwd_decl.hpp"
#include "soccer.hpp"

extern int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP;
extern int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT;
extern int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP;
extern int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT;

/////////////// PART TO EDIT
enum MidProcessor {
    MID_PROCESSOR_FIELD_BOUNDARY = 0,
    MID_PROCESSOR_COLOUR_ROI = 1,
    MID_PROCESSOR_TOTAL = 2
};

enum Detect {
    DETECTOR_FIELD_LINE = 0,
    DETECTOR_BALL,
    DETECTOR_ROBOT,
    DETECTOR_TOTAL
};

void Vision::setupAlgorithms_() {
    addMiddleInfoProcessor_(MID_PROCESSOR_FIELD_BOUNDARY, new FieldBoundaryFinder());
    addMiddleInfoProcessor_(MID_PROCESSOR_COLOUR_ROI, new ColourROI());
#ifndef CTC_2_1
    addDetector_(DETECTOR_ROBOT, newSSRobotDetector());
#else
    addDetector_(DETECTOR_ROBOT, newRobotDetector());
#endif
    addDetector_(DETECTOR_FIELD_LINE, new RegionFieldFeatureDetector());
    addDetector_(DETECTOR_BALL, new BallDetector());
}

void Vision::runAlgorithms_() {
    Timer t;
    t.restart();
    runMiddleInfoProcessor_(MID_PROCESSOR_FIELD_BOUNDARY);
    runMiddleInfoProcessor_(MID_PROCESSOR_COLOUR_ROI);
    regionFinderTime += t.elapsed_us();
    t.restart();
    runDetector_(DETECTOR_ROBOT);
    robotDetectorTime += t.elapsed_us();
    t.restart();
    runDetector_(DETECTOR_FIELD_LINE);
    fieldFeaturesTime += t.elapsed_us();
    t.restart();
    runDetector_(DETECTOR_BALL);
    ballDetectorTime += t.elapsed_us();
}
//////////////////////////////////////////

void Vision::runMiddleInfoProcessor_(uint32_t index) {
    getMiddleInfoProcessor_(index)->find(info_in_, info_middle_, info_out_);
}

void Vision::runDetector_(uint32_t index) {
    getDetector_(index)->detect(info_in_, info_middle_, info_out_);
}

Vision::Vision()
  : bbox_top_(BBox(Point(0,0), Point(TOP_SALIENCY_COLS, TOP_SALIENCY_ROWS))),
    bbox_bot_(BBox(Point(0,0), Point(BOT_SALIENCY_COLS, BOT_SALIENCY_ROWS))),
    combined_fovea_(CombinedFovea(
        new Fovea(bbox_top_, TOP_SALIENCY_DENSITY, true, true),
        new Fovea(bbox_bot_, BOT_SALIENCY_DENSITY, false, true)
    )),
    full_region_top_(RegionI(bbox_top_, true, *combined_fovea_.top_, TOP_SALIENCY_DENSITY)),
    full_region_bot_(RegionI(bbox_bot_, false, *combined_fovea_.bot_, BOT_SALIENCY_DENSITY)),
    frameCount(0), foveaTime(0), fieldFeaturesTime(0),
    regionFinderTime(0), ballDetectorTime(0)
{
    llog(INFO) << "Vision Created" << std::endl;

    detectors_ = new Detector*[DETECTOR_TOTAL];
    middle_info_processors_ = new MiddleInfoProcessor*[MID_PROCESSOR_TOTAL];

    for (size_t i = 0; i < DETECTOR_TOTAL; ++i) {
        detectors_[i] = NULL;
    }
    for (size_t i = 0; i < MID_PROCESSOR_TOTAL; ++i) {
        middle_info_processors_[i] = NULL;
    }

    setupAlgorithms_();

}

Vision::~Vision() {
    for (size_t i = 0; i < MID_PROCESSOR_TOTAL; ++i) {
        delete middle_info_processors_[i];
    }
    for (size_t i = 0; i < DETECTOR_TOTAL; ++i) {
        delete detectors_[i];
    }
    delete detectors_;
    delete middle_info_processors_;

    llog(INFO) << "Vision Destroyed" << std::endl;
}

void Vision::addMiddleInfoProcessor_(uint32_t index, MiddleInfoProcessor* processor) {
    middle_info_processors_[index] = processor;
}

MiddleInfoProcessor* Vision::getMiddleInfoProcessor_(uint32_t index) {
    return middle_info_processors_[index];
}

void Vision::addDetector_(uint32_t index, Detector* detector) {
    detectors_[index] = detector;
}

Detector* Vision::getDetector_(uint32_t index) {
    return detectors_[index];
}


VisionInfoOut Vision::processFrame(const CombinedFrame& this_frame, const VisionInfoIn& info_in) {


    Timer t;
    uint32_t time;

    VisionInfoMiddle info_middle;
    VisionInfoOut info_out;

    info_in_ = info_in;
    info_middle_ = info_middle;
    info_out_ = info_out;

    info_out_.cameraToRR = &(this_frame.camera_to_rr_);

    // Track the number of frames we've seen.
    ++frameCount;

    /*
     * Primary Region Creation
     */
    t.restart();
    combined_fovea_.generate(this_frame,
                             ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP,
                             ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP,
                             ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT,
                             ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT);

    time = t.elapsed_us();
    llog(VERBOSE) << "Fovea generation took " << time << " us" << std::endl;
    foveaTime += time;
    t.restart();

    // TODO: Do not copy these, re-generate these? Probably trivial improvement - for later
    full_region_top_ = RegionI(bbox_top_, true, *combined_fovea_.top_, TOP_SALIENCY_DENSITY);
    full_region_bot_ = RegionI(bbox_bot_, false, *combined_fovea_.bot_, BOT_SALIENCY_DENSITY);

    /*
     * Full Finders, Region Finders, and Detectors
     */
    info_middle_.full_regions.push_back(full_region_top_);
    info_middle_.full_regions.push_back(full_region_bot_);
    info_middle_.this_frame = &this_frame;
    if (offNao) {

    }
    else {
        runAlgorithms_();
    }

    // Log the 1000 frame average vision timings.
    if(frameCount == 1000)
    {
        // Reset the frame count.
        frameCount = 0;

        // Print the timings.
        llog(INFO) << std::endl << "VISION TIMINGS (us)" << std::endl;
        llog(INFO) << "Average fovea generation time: " <<
                                        ((float)foveaTime)/1000.0f << std::endl;
        llog(INFO) << "Average robot detector time: " <<
                                ((float)robotDetectorTime)/1000.0f << std::endl;
        llog(INFO) << "Average field features time: " <<
                                ((float)fieldFeaturesTime)/1000.0f << std::endl;
        llog(INFO) << "Average Region finder time: " <<
                                 ((float)regionFinderTime)/1000.0f << std::endl;
        llog(INFO) << "Average ball detector time: " <<
                               ((float)ballDetectorTime) / 1000.0f << std::endl;
        llog(INFO) << "Average total processFrame time: " << ((float)(DCCTime+
            foveaTime+fieldFeaturesTime+regionFinderTime+ballDetectorTime + robotDetectorTime)) /
                                                           1000.0f << std::endl;

        // Reset timers.
        DCCTime = 0;
        foveaTime = 0;
        robotDetectorTime = 0;
        fieldFeaturesTime = 0;
        regionFinderTime = 0;
        ballDetectorTime = 0;
    }

    return info_out_;
}
