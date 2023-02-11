/*
    Copyright (c) 2013, Taiga Nomi and the respective contributors
    All rights reserved.

    Use of this source code is governed by a BSD-style license that can be found
    in the LICENSE file.
*/
#include <sys/time.h>
#include <sys/resource.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

#include "tiny_dnn/tiny_dnn.h"
#include "tiny_jnn/layers/convolutional_layer.h"
#include "tiny_jnn/layers/transpose_layer.h"
#define INPUT_WIDTH 320
#define INPUT_HEIGHT 240
#define N_ANCHORS 8732
#define N_CLASSES 21
#define BG_CLASS_ID 0
#define NMS_THRESHOLD 0.5
#define MEAN_B 123
#define MEAN_G 117
#define MEAN_R 104

void convert_image(const std::string& imagefilename,
                   int w,
                   int h,
                   tiny_dnn::vec_t& data) {
  const int MEAN_BGR[] = {MEAN_B, MEAN_G, MEAN_R};

  tiny_dnn::image<> img(imagefilename, tiny_dnn::image_type::grayscale);
  tiny_dnn::image<> resized = resize_image(img, w, h);
  data                      = resized.to_vec();

  size_t spatial_size = resized.height() * resized.width();
  for (size_t c = 0; c < resized.depth(); ++c) {
    for (size_t i = 0; i < spatial_size; ++i) {
      data[c * spatial_size + i] -= MEAN_BGR[c];
    }
  }
}

void concat_hwc_features(tiny_dnn::vec_t& collections,
                         size_t n_collection_items,
                         tiny_dnn::vec_t& feature,
                         size_t in_spatial_size,
                         size_t in_channels) {
  tiny_dnn::vec_t t_feature;
  t_feature.resize(feature.size());

  // transpose features to HxWxC
  for (size_t i = 0; i < in_spatial_size; ++i) {
    for (size_t j = 0; j < in_channels; ++j) {
      t_feature[i * in_channels + j] = feature[j * in_spatial_size + i];
    }
  }

  // Append features to vectors
  for (size_t i = 0; i < t_feature.size(); ++i) {
    collections[n_collection_items + i] = t_feature[i];
  }
}

void inline_softmax(tiny_dnn::vec_t& confidences) {
  for (size_t i = 0; i < N_ANCHORS; ++i) {
    float sum = 0;
    for (size_t j = 0; j < N_CLASSES; ++j) {
      sum += exp(confidences[i * N_CLASSES + j]);
    }

    for (size_t j = 0; j < N_CLASSES; ++j) {
      confidences[i * N_CLASSES + j] =
        exp(confidences[i * N_CLASSES + j]) / sum;
    }
  }
}

#ifdef TINY_DNN_TEST
template<int window_width_tiny_jnn,
         int window_height_tiny_jnn,
         int w_dilation_tiny_jnn = 1,
         int h_dilation_tiny_jnn = 1>
using conv = tiny_dnn::convolutional_layer;
#endif

void construct_nets(std::vector<tiny_dnn::network<tiny_dnn::sequential>>& nets,
                    const std::string& modelFolder) {
#ifndef TINY_DNN_TEST
  using tiny_jnn::conv;
#endif
  using pool       = tiny_dnn::max_pooling_layer;
  using relu       = tiny_dnn::relu_layer;
  using batch_norm = tiny_dnn::batch_normalization_layer;
  using net_phase  = tiny_dnn::net_phase;
  using l2norm     = tiny_dnn::l2_normalization_layer;
  using pad        = tiny_dnn::zero_pad_layer;
  using transpose  = tiny_jnn::transpose_layer;

  %(nets)s

  for (size_t i = 0; i < nets.size(); ++i) {
    std::ostringstream modelPath;
    modelPath << modelFolder << std::setfill('0') << std::setw(2) << i + 1
              << ".weights";
    std::ifstream ifs(modelPath.str());
    if (ifs.fail()) {
      std::cout << "Failed to load weights from " << modelPath.str()
                << std::endl;
    } else {
      std::cout << "Loading weights from " << modelPath.str() << std::endl;
    }
    ifs >> nets[i];
  }
}

// returns wall-clock detection time in seconds
double detect(std::vector<tiny_dnn::network<tiny_dnn::sequential>>& nets,
              const std::string& src_filename) {
  // convert imagefile to vec_t
  tiny_dnn::vec_t img;
  convert_image(src_filename, INPUT_WIDTH, INPUT_HEIGHT, img);
  struct rusage start_rusage, end_rusage;
  getrusage(RUSAGE_SELF, &start_rusage);
  auto start = std::chrono::steady_clock::now();

  auto feature = img;
  for (int n = 0; n < nets.size(); ++n) {
     feature = nets[n].predict(feature);
  }

  auto end = std::chrono::steady_clock::now();
  getrusage(RUSAGE_SELF, &end_rusage);
  double elapsed_seconds = std::chrono::duration_cast<
                           std::chrono::duration<double> >(end - start).count();
  std::cout << "real    0m" << elapsed_seconds << "s" << std::endl;
  struct timeval diff;
  timersub(&(end_rusage.ru_utime), &(start_rusage.ru_utime), &diff);
  std::cout << "user    0m" << diff.tv_sec << "." << diff.tv_usec << "s" << std::endl;
  timersub(&(end_rusage.ru_stime), &(start_rusage.ru_stime), &diff);
  std::cout << "sys     0m" << diff.tv_sec << "." << diff.tv_usec << "s" << std::endl;

  auto checksum = std::accumulate(feature.begin(), feature.end(), 0.);
  std::cout << "checksum:\t" << checksum << std::endl;

  return elapsed_seconds;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: example_ssd_test model_folder_path img_file_path [img_file_path ...]";
    return -1;
  }
  std::vector<tiny_dnn::network<tiny_dnn::sequential>> nets;

  auto modelFolder = argv[1];
  construct_nets(nets, modelFolder);
  double real = 0;
  for (int i = 2; i < argc; ++i) {
     real += detect(nets, argv[i]);
  }
   std::ostringstream modelPath;
   modelPath << modelFolder << "/time.txt";
   std::ofstream ofs(modelPath.str());
   ofs << real / (argc - 2) << std::endl;
  std::cout << "Average inference time written to " << modelPath.str() << ": " << real / (argc - 2) << std::endl;
}
