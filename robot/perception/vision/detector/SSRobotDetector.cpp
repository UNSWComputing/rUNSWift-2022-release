#ifndef CTC_2_1
#include "SSRobotDetector.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <ctime> // test time to detect

#include <tiny_jnn/layers/convolutional_layer.h>
#include <tiny_dnn/layers/convolutional_layer.h>
#include <tiny_jnn/layers/transpose_layer.h>
#include <tiny_dnn/layers/batch_normalization_layer.h>
#include <tiny_dnn/layers/max_pooling_layer.h>
#include <tiny_dnn/layers/fully_connected_layer.h>
#include <tiny_dnn/activations/relu_layer.h>
#include <tiny_dnn/activations/silu_layer.h>
#include <tiny_dnn/activations/sigmoid_layer.h>
#include "utils/eigen_helpers.hpp"
#include "utils/home_nao.hpp"

SSRobotDetector::SSRobotDetector(){
    constructNet(nn);
    loadWeights(SSRobotDetector::weight_path, nn);
}

Detector *newSSRobotDetector() {
    return new SSRobotDetector();
}

void SSRobotDetector::detect(VisionInfoIn const& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {
    #ifdef RD_USE_PIXEL_CLASSIFIER
    std::cout << "Pixel classifier detect\n";
    auto time_interval = std::clock();
    auto start_time = std::clock();
    
    //STEP 1 - Get top camera image
    BBox full_bound = BBox(Point(0,0), Point(TOP_IMAGE_ROWS-1, TOP_IMAGE_COLS-1)); 
    RegionI *top_region = new RegionI(info_middle.full_regions[0].subRegion(full_bound));
    std::cout << "Time to extract full region: " << ((std::clock() - time_interval) / (double) CLOCKS_PER_SEC) <<"s\n"; // 0s
    time_interval = std::clock();

    //STEP 2 - Minmax normalise grayscale image and convert to vec_t 
    tiny_dnn::vec_t dnn_image = convertVecT(*top_region); 
    std::cout << "Time to convert to vec_t: " << ((std::clock() - time_interval) / (double) CLOCKS_PER_SEC) <<"s\n";
    time_interval = std::clock();

    //STEP 3 - Pass image through net and threshold output
    Eigen::MatrixXf result = dnnPredict(dnn_image, nn);
    std::cout << "Time to predict: " << ((std::clock() - time_interval) / (double) CLOCKS_PER_SEC) <<"s\n";
    time_interval = std::clock();

    //STEP 4 - Apply connected-component labeling and extract bboxes and store them
    std::vector <RobotVisionInfo> robots = getRobotsInfo(result, info_out);
    info_out.robots = robots;
    std::cout << "Time to get Robot info: " << ((std::clock() - time_interval) / (double) CLOCKS_PER_SEC) <<"s\n";
    std::cout << "Total elapsed time: " << ((std::clock() - start_time) / (double) CLOCKS_PER_SEC) <<"s\n";
    
    
    #else
    // Step 1 - Get top camera image (default wxh: 320x240)
    BBox full_bound = BBox(Point(0,0), Point(TOP_IMAGE_ROWS-1, TOP_IMAGE_COLS-1)); 
    RegionI *top_region = new RegionI(info_middle.full_regions[0].subRegion(full_bound));
    
    // STEP 2 - Normalise grayscale image and convert to vec_t 
    tiny_dnn::vec_t dnn_image = convertVecT(*top_region); 
    tiny_dnn::vec_t dnn_image_resized = resizeImage(dnn_image, 320, 240, SSRobotDetector::in_width, SSRobotDetector::in_height);

    // Step 3 - Pass image through net
    tiny_dnn::vec_t output = nn.predict(dnn_image_resized);
    
    // Step 4 - Extract candidate bboxes from output
    std::vector<tiny_dnn::bounding_box> bboxes;
    SSRobotDetector::getCandidates(output, bboxes, anchors);
    
    // Step 5 - Apply NMS to get indices of final bboxes
    std::vector<int> keep_indices = tiny_dnn::nms(bboxes, SSRobotDetector::iou_thres);
    
    // Step 6 - Get final bboxes and store them
    std::vector <RobotVisionInfo> robots = getRobotsInfo(bboxes, keep_indices, info_out);
    info_out.robots = robots;
    // for (auto i = keep_indices.begin(); i!=keep_indices.end(); ++i) {
    //     std::cout << bboxes[*i].x_min << ' ' 
    //               << bboxes[*i].y_min << ' ' 
    //               << bboxes[*i].x_max << ' '
    //               << bboxes[*i].y_max << ' '
    //               << bboxes[*i].score << '\n'; 
    // }
    #endif // RD_USE_PIXEL_CLASSIFIER
    
}

tiny_dnn::vec_t SSRobotDetector::convertVecT(RegionI const& region) {
    const int rows = region.getRows();
    const int cols = region.getCols();
    tiny_dnn::vec_t dnn_image(rows * cols);
    // Normalize and convert to type vec_t
    RegionI::iterator_raw curr_point = region.begin_raw();
    for(int pixel = 0; pixel < rows * cols; ++pixel) {
        dnn_image[pixel] = float(curr_point.getY()/255.0f);
        ++curr_point;
    }
    return dnn_image;
}

tiny_dnn::vec_t SSRobotDetector::resizeImage(tiny_dnn::vec_t const& src, int w1, int h1, int w2, int h2) {
    const float x_ratio = w1 / float(w2);
    const float y_ratio = h1 / float(h2);

    if (x_ratio == 1 && y_ratio == 1)
        return src;

    tiny_dnn::vec_t dnn_image(w2 * h2);
    float px, py;
    for (int i=0; i<h2; i++) {
        for (int j=0; j<w2;j++) {
            px = floor(j*x_ratio);
            py = floor(i*y_ratio);
            dnn_image[(i*w2)+j] = src[int(py*w1)+px];
        }
    }
    return dnn_image;
}

#ifdef RD_USE_PIXEL_CLASSIFIER
Eigen::MatrixXf SSRobotDetector::convertMat(tiny_dnn::vec_t const& vec, int height, int width) {
    Eigen::MatrixXf mat(height, width); 
    int x = 0;
    int y = 0;
    float thresh = -2; // sigmoid(thresh) = predicted prob(robot). e.g. sigmoid(-1)=27%
    for (auto i = vec.begin(); i != vec.end(); ++i) {
        mat(x,y) = *i > thresh ? 1 : 0; 
        ++y;
        if (y == width) {  
            y = 0;
            ++x;
        }
    }
    return mat;
}

Eigen::MatrixXf SSRobotDetector::dnnPredict(vec_t const& dnn_image, network<sequential>& nn) {
    // Forward pass
    auto result = nn.predict(dnn_image);
    
    // Convert output to matrix
    const int width = sqrt(4 * result.size() / 3);
    const int height = result.size() / width;
    Eigen::MatrixXf mat = convertMat(result, height, width);
    
    return mat;
}

std::vector<RobotVisionInfo> SSRobotDetector::getRobotsInfo(Eigen::MatrixXf& mat, VisionInfoOut& info_out) {
    // Connected component labeling
    std::vector <RobotVisionInfo> robots;
    std::unordered_map<int, int> label_mappings;
    int label = 2;

    // Label classified pixels
    for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
            if (mat(i,j) == 1) {
                // Above pixel labeled
                if (i > 0 && mat(i-1, j) > 1) {
                    mat(i,j) = mat(i-1, j);
                    // Left pixel is part of region, map left pixel value to current pixel value
                    if (j > 0 && mat(i, j-1) > 1 && mat(i-1,j) != mat(i,j-1)) {
                        label_mappings[mat(i,j-1)] = mat(i,j);
                    } 
                } 
                // Left pixel labeled
                else if (j > 0 && mat(i, j-1) > 1) {
                    mat(i,j) = mat(i,j-1);
                } 
                // New label
                else {
                    mat(i,j) = label;
                    ++label;
                }
            }
        }
    }

    // Replace equivalent labels (second run)
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            label = mat(i,j);
            while (label_mappings.find(label) != label_mappings.end()) {
                label = label_mappings[label];
            } 
            mat(i,j) = label;
        }
    }
    // std::cout << '\n' << mat << '\n';

    // Get bounding boxes
    std::unordered_map<int, std::vector<int>> bbox_mappings;

    for (int i = 0; i < mat.rows(); ++i){
        for (int j = 0; j < mat.cols(); ++j){
            int label = mat(i,j);
            if (label > 1) {
                // New robot region found
                if (bbox_mappings.find(label) == bbox_mappings.end()) {
                    bbox_mappings[label] = std::vector<int>{j, i, -1, -1};
                } 
                // Update robot region
                else {
                    bbox_mappings[label][0] = std::min(bbox_mappings[label][0], j);
                    bbox_mappings[label][1] = std::min(bbox_mappings[label][1], i);
                    bbox_mappings[label][2] = std::max(bbox_mappings[label][2], j);
                    bbox_mappings[label][3] = std::max(bbox_mappings[label][3], i);
                }
            }
        }
    }

    // Get robot info from bboxes
    for (auto const& bbox: bbox_mappings) { 
        // Only consider bboxes wider and taller than 1 pixel in the 15x20 output
        if (bbox.second[2] >= 0 && bbox.second[3] >= 0 && (bbox.second[2]-bbox.second[0]) > 1 && (bbox.second[3]-bbox.second[1]) > 1) {
            BBox detected_bound = BBox(Point(64 * bbox.second[0], 64 * bbox.second[1]), Point(64 * bbox.second[2], 64 * bbox.second[3])); // multiply b/c its relative to original size
            Point feet(detected_bound.width()/2 + detected_bound.a.x(), detected_bound.b.y());  //no top camera, therefore top is zero
            RRCoord rr_feet = info_out.cameraToRR->convertToRR(feet, false);
            robots.push_back(RobotVisionInfo(rr_feet, RobotVisionInfo::rUnknown, detected_bound, RobotVisionInfo::TOP_CAMERA));
        }
    }
    return robots;
}
#endif

void SSRobotDetector::constructNet(tiny_dnn::network<tiny_dnn::sequential> &nn) {
    #ifdef RD_USE_PIXEL_CLASSIFIER
    using tiny_jnn::conv;
    using transpose_layer = tiny_jnn::transpose_layer;
    using batch_norm = tiny_dnn::batch_normalization_layer;
    using relu = tiny_dnn::relu_layer;

    nn 
    << conv<8,3>(320, 240, 8, 3, 1, 8, padding::same, true, 4, 2) // 320x240x1 -> 80x120x8
    << batch_norm(80 * 120, 8, 1e-5, 0.1, net_phase::test)
    << relu(80, 120, 8)
    << conv<3,3>(80, 120, 3, 3, 8, 8, padding::same, true, 1, 1) // 80x120x8 -> 80x120x8 
    << batch_norm(80 * 120, 8, 1e-5, 0.1, net_phase::test)
    << relu(80, 120, 8)
    << transpose_layer(80, 120, 8) // 80x120x8 -> 120x80x8
    << conv<8,3>(120, 80, 8, 3, 8, 16, padding::same, true, 4, 2) // 120x80x8 -> 30x40x16 
    << batch_norm(30 * 40, 16, 1e-5, 0.1, net_phase::test)
    << relu(30, 40, 16)
    << transpose_layer(30, 40, 16) // 30x40x16 -> 40x30x16
    << conv<3,3>(40, 30, 3, 3, 16, 16, padding::same, true, 1, 1) // 40x30x16 -> 40x30x16 
    << batch_norm(40 * 30, 16, 1e-5, 0.1, net_phase::test)
    << relu(40, 30, 16)
    << conv<3,3>(40, 30, 3, 3, 16, 32, padding::same, true, 2, 2) // 40x30x16 -> 20x15x32 
    << batch_norm(20 * 15, 32, 1e-5, 0.1, net_phase::test)
    << relu(20, 15, 32)
    << conv<3,3>(20, 15, 3, 3, 32, 32, padding::same, true, 1, 1) // 20x15x32 -> 20x15x32 
    << batch_norm(20 * 15, 32, 1e-5, 0.1, net_phase::test)
    << relu(20, 15, 32)
    << conv<3,3>(20, 15, 3, 3, 32, 32, padding::same, true, 1, 1) // 20x15x32 -> 20x15x32 
    << batch_norm(20 * 15, 32, 1e-5, 0.1, net_phase::test)
    << relu(20, 15, 32)
    << conv<3,3>(20, 15, 3, 3, 32, 1, padding::same, true, 1, 1); // 20x15x32 -> 20x15x1 
    #else
    using tiny_jnn::conv;
    using sigmoid = tiny_dnn::sigmoid_layer;
    using silu = tiny_dnn::silu_layer;
    
    nn // ~8ms on robot 0.711mAP@0.5
    << conv<3,3>(256, 192, 3, 3, 1, 4, padding::same, true, 2, 2)
    << silu(128, 96, 4)
    << conv<8,3>(128, 96, 8, 3, 4, 4, padding::same, true, 4, 2)
    << silu(32, 48, 4)
    << conv<3,3>(32, 48, 3, 3, 4, 4, padding::same, true, 1, 1)
    << silu(32, 48, 4)
    << conv<3,3>(32, 48, 3, 3, 4, 8, padding::same, true, 1, 2)
    << silu(32, 24, 8)
    << conv<3,3>(32, 24, 3, 3, 8, 8, padding::same, true, 1, 1)
    << silu(32, 24, 8)
    << conv<3,3>(32, 24, 3, 3, 8, 12, padding::same, true, 2, 2)
    << silu(16, 12, 12)
    << conv<3,3>(16, 12, 3, 3, 12, 12, padding::same, true, 1, 1)
    << silu(16, 12, 12)
    << conv<3,3>(16, 12, 3, 3, 12, 16, padding::same, true, 1, 1)
    << silu(16, 12, 16)
    << conv<1,1>(16, 12, 1, 1, 16, 18, padding::valid, true, 1, 1)
    << sigmoid(16, 12, 18);
    #endif // RD_USE_PIXEL_CLASSIFIER
}

void SSRobotDetector::loadWeights(std::string const& path, tiny_dnn::network<tiny_dnn::sequential> &nn) {
    std::ifstream fin(path);
  
    if (fin.fail()) {
        std::cout << "Could not open file " << path << "\n";
    } else {
        fin >> nn;
        std::cout << path << " loaded successfully!.\n";
    }
}



#ifndef RD_USE_PIXEL_CLASSIFIER
void SSRobotDetector::getCandidates(tiny_dnn::vec_t &output, std::vector<tiny_dnn::bounding_box> &bboxes, std::vector<std::vector<float>> const& anchors) {
    
    // Iterate through each cell
    for (size_t a = 0; a < SSRobotDetector::nb_anchors; ++a) { // iterate over anchors
        size_t anchor_offset = a * SSRobotDetector::out_channels_per_anchor * SSRobotDetector::out_height * SSRobotDetector::out_width;
        for (size_t h = 0; h < SSRobotDetector::out_height; ++h) { // iterate over height
            size_t height_offset = h * SSRobotDetector::out_width;
            for (size_t w = 0; w < SSRobotDetector::out_width; ++w) { // iterate over width
                // Confidence
                float conf = output[anchor_offset + 4 * SSRobotDetector::out_height * SSRobotDetector::out_width + height_offset + w] *
                               output[anchor_offset + 5 * SSRobotDetector::out_height * SSRobotDetector::out_width + height_offset + w];
                // Obtain bbox candidates
                if (conf > SSRobotDetector::conf_thres) {
                    float xx = SSRobotDetector::cell_width * (output[anchor_offset + height_offset + w] * 2.0f - 0.5f + w);
                    float yy = SSRobotDetector::cell_height * (output[anchor_offset + height_offset + w + SSRobotDetector::out_width * SSRobotDetector::out_height * 1] * 2.0f - 0.5f + h);
                    float ww = std::pow(output[anchor_offset + height_offset + w + SSRobotDetector::out_width * SSRobotDetector::out_height * 2] * 2.0f, 2.0f) * anchors[a][0];
                    float hh = std::pow(output[anchor_offset + height_offset + w + SSRobotDetector::out_width * SSRobotDetector::out_height * 3] * 2.0f, 2.0f) * anchors[a][1];
                
                    float x1 = xx - ww / 2.0f;
                    float y1 = yy - hh / 2.0f;
                    float x2 = xx + ww / 2.0f;
                    float y2 = yy + hh / 2.0f;
                    // std::cout << x1 << ' ' << y1 << ' ' << x2 << ' ' << y2 << ' ' << conf << '\n';
                    bboxes.push_back(tiny_dnn::bounding_box{x1, y1, x2, y2, conf});
                }
            }
        }
    }
}

std::vector<RobotVisionInfo> SSRobotDetector::getRobotsInfo(std::vector<tiny_dnn::bounding_box> &bboxes, std::vector<int> &keep_indices, VisionInfoOut& info_out) {
    // Get robot info from bboxes
    std::vector <RobotVisionInfo> robots;
    // Get robot info from bboxes
    for (auto i = keep_indices.begin(); i!=keep_indices.end(); ++i) {
        BBox detected_bound = BBox(
            Point(SSRobotDetector::width_scale_factor*bboxes[*i].x_min, SSRobotDetector::height_scale_factor*bboxes[*i].y_min), 
            Point(SSRobotDetector::width_scale_factor*bboxes[*i].x_max, SSRobotDetector::height_scale_factor*bboxes[*i].y_max)
        ); 
        Point feet(detected_bound.width()/2 + detected_bound.a.x(), detected_bound.b.y());  //no top camera, therefore top is zero
        RRCoord rr_feet = info_out.cameraToRR->convertToRR(feet, false);
        robots.push_back(RobotVisionInfo(rr_feet, RobotVisionInfo::rUnknown, detected_bound, RobotVisionInfo::TOP_CAMERA));
    }
    return robots;
}
#endif
#endif
