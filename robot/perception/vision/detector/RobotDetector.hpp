#ifndef PERCEPTION_VISION_DETECTOR_ROBOTDETECTOR
#define PERCEPTION_VISION_DETECTOR_ROBOTDETECTOR

#include "perception/vision/detector/DetectorInterface.hpp"


#include "../regionfinder/RobotColorROI.hpp"
#include "RandomForest.hpp"
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#ifndef CTC_2_1
#include "tiny_dnn/network.h"
#include "DNNHelper.hpp"
#endif

#define DNN_WEIGHTS_DIR "data/dnn_model/"

//vatnao debugging
//#define ROBOT_DETECTOR_USES_VATNAO
// #ifdef ROBOT_DETECTOR_USES_VATNAO
// 	#include "soccer.hpp"
// 	#include "../VisionDebuggerInterface.hpp"
// //#define OUTPUT_DATA
// #endif

//#define ROBOT_DETECTOR_TIMER
#ifdef ROBOT_DETECTOR_TIMER
#include "utils/Timer.hpp"
#endif

// Class for creating candidate robots
class Cluster {
public:
    std::vector<RegionI> regions_;
    // all the values are raw
    BBox box_;

    BBox fbox_;

    //features
    int whitePixels_;

    float solidity_;

    float ratio_;

    bool isRobot_;

    float confidence_;

    Cluster();
    void addRegionToCluster(const RegionI& region, int whiteCount);

	bool overlapsRegion(RegionI& region);

    Point getBaseCenter();

	inline bool isTopCamera() {
        	return regions_.front().isTopCamera();
    }
};

// #ifdef ROBOT_DETECTOR_USES_VATNAO
// // Essentially just a class to wrap up my vatnao calls
// class RobotDetectorDebugger {
// public:
// 	//defines the neccesary vatnao variables
// 	RobotDetectorDebugger();

// 	void highlightRegions(std::vector<RegionI>& regions);
// 	void highlightCandidates(std::vector<Cluster>& candidates);

// 	void showSaliencyImage(const RegionI& region);

//     void outputToFile(std::vector<Cluster>& candidates);

//     void outputTimings(std::vector<float>& timings, int numRegions, int numCandidates);

// private:

//     RegionI* newTop;
//     RegionI* newBot;

//     VisionDebugModule* vdm_;

//     std::map<int, bool> framesWritten_;
//     bool dataHeaderWritten_;
//     bool dataHeaderWrittenTimings_;
// };
// #endif

class RobotDetector : public Detector {
public:
	RobotDetector();
	void detect(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

private:

    RegionI* newTop_;
    //RegionI* newBot_;

    RandomForest classifier_;

    RobotColorROI* regionFinder;

    std::vector<Cluster> clusters;

    std::vector<int> activatedCounts_;

    #ifndef CTC_2_1
    tiny_dnn::network<sequential> nn;
    #endif

	void runThresholding(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

	void runColorRoi(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

  void runBasepointScanner(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

	void findRobots(VisionInfoMiddle& info_mid, VisionInfoOut& info_out, std::vector<Cluster>& candidates);

	std::vector<Cluster> createCandidateRobots(std::vector<RegionI>& regions);

    std::vector<Cluster> DBSCAN(std::vector<RegionI>& regions, int epsilon, unsigned int minpoints);

    std::vector<Cluster> createClustersFromRegions(std::vector<RegionI>& regions);

    std::vector<Cluster> createClustersFromBasepoints(std::vector<Point>& basePoints, std::vector<Point>& basePointImageCoords);

    std::vector<int> rangeQuery(std::vector<RegionI>& regions, int currRegionId, int epsilon);

    int distance(const RegionI& regionA, const RegionI& regionB);

    bool inEllipse(const RegionI& regionA, const RegionI& regionB);

    bool overlaps(RegionI& regionA, RegionI& regionB);

    void featureExtraction(VisionInfoMiddle& info_mid, BBox bound, std::vector<float>& features);

    Eigen::MatrixXf imageExtraction(VisionInfoMiddle& info_mid, BBox bound);

    Eigen::MatrixXf convertMat(RegionI& region);

    #ifndef CTC_2_1
    tiny_dnn::network<sequential> load_3_layer_cnn();

    Eigen::MatrixXf dnn_resize(Eigen::MatrixXf &, int, int);

    int dnn_predict(Eigen::MatrixXf &, network<sequential>);
    #endif

// #ifdef ROBOT_DETECTOR_USES_VATNAO
// 	RobotDetectorDebugger* debugger_;
// #endif
};





#endif
