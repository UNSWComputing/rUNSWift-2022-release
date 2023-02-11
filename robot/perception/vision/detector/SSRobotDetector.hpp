#ifndef CTC_2_1
#ifndef PERCEPTION_VISION_DETECTOR_ROBOTDETECTOR
#define PERCEPTION_VISION_DETECTOR_ROBOTDETECTOR

#include "perception/vision/detector/DetectorInterface.hpp"


#include "../regionfinder/RobotColorROI.hpp"
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "tiny_dnn/network.h"
#include "tiny_dnn/util/nms.h"
#include "DNNHelper.hpp"
#include "utils/home_nao.hpp"

//#define RD_USE_PIXEL_CLASSIFIER // otherwise use YOLO robot detector

class SSRobotDetector : public Detector {
public:
    SSRobotDetector();
    void detect(VisionInfoIn const& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out); // called in Vision.cpp
private:
    RegionI* newTop_;
    tiny_dnn::network<sequential> nn;
    tiny_dnn::vec_t convertVecT(RegionI const& region);
    void constructNet(tiny_dnn::network<tiny_dnn::sequential> &nn);
    void loadWeights(std::string const& path, tiny_dnn::network<tiny_dnn::sequential> &nn);
    void drawBBoxes(std::vector<RobotVisionInfo> const& robots);
    tiny_dnn::vec_t resizeImage(tiny_dnn::vec_t const& src, int w1, int h1, int w2, int h2);
    
    #ifdef RD_USE_PIXEL_CLASSIFIER
    Eigen::MatrixXf convertMat(tiny_dnn::vec_t const& vec, int height, int width);
    std::vector<RobotVisionInfo> getRobotsInfo(Eigen::MatrixXf& mat, VisionInfoOut& info_out);
    Eigen::MatrixXf dnnPredict(vec_t const& img, network<sequential>& nn);
    const std::string weight_path = getHomeNao("data/vision/robotdetection/JNN7.weights");
    #else 
    void getCandidates(tiny_dnn::vec_t &output, std::vector<tiny_dnn::bounding_box> &bboxes, std::vector<std::vector<float>> const& anchors);
    std::vector<RobotVisionInfo> getRobotsInfo(std::vector<tiny_dnn::bounding_box> &bboxes, std::vector<int> &keep_indices, VisionInfoOut& info_out);

    /*
     * Model constants and hyperparameters are specific to the YOLO neural network
     * defined in SSRobotDetector.cpp.
     */
    const std::string weight_path = getHomeNao("data/vision/robotdetection/rolov5xxs3_256.weights");
    const size_t in_height = 192;
    const size_t in_width = 256;
    const size_t height_scale_factor = 960 / in_height; // to scale bbox height up to match a 1280x960 image
    const size_t width_scale_factor = 1280 / in_width; // to scale bbox width up to match a 1280x960 image

    const size_t out_height = 12; 
    const size_t out_width = 16; 
    const size_t out_channels_per_anchor = 6; // x,y,w,h,p,c
    const size_t cell_height = in_height / out_height;
    const size_t cell_width =  in_width / out_width;
    
    const std::vector<std::vector<float>> anchors = {{18.498, 29.69}, {39.13,69.62}, {79.15,138.91}};
    const size_t nb_anchors = anchors.size();
    const float conf_thres = 0.2f; // hyperparameter for obtaining candidate bboxes
    const float iou_thres = 0.4f;  // hyperparameter for NMS

    #endif // RD_USE_PIXEL_CLASSIFIER
};

#endif
#endif
