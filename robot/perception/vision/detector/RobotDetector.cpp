
#include "RobotDetector.hpp"
#include <iostream>
#include <fstream>
#ifndef CTC_2_1
   #include <tiny_dnn/layers/convolutional_layer.h>
   #include <tiny_dnn/layers/max_pooling_layer.h>
   #include <tiny_dnn/layers/fully_connected_layer.h>
   #include <tiny_dnn/activations/relu_layer.h>
#endif
#include "perception/vision/other/WriteImage.hpp"
#include "utils/eigen_helpers.hpp"
#include "utils/home_nao.hpp"

//Parameters for adaptive thresholding
#ifdef CTC_2_1
  #define ATR_MEAN_PERCENT -20
#else
  #define ATR_MEAN_PERCENT -50
#endif

#define ATR_WINDOW 90

// Used for vatnao debugging
// #define RD_USING_VATNAO
#ifdef RD_USING_VATNAO
#include "soccer.hpp"
#include "../VisionDebuggerInterface.hpp"
VisionPainter *p = NULL;
VisionDebugQuery q;
VisionPainter *topPainter = NULL;
VisionPainter *botPainter = NULL;
#endif // RD_USING_VATNAO

// IF_RD_USING_VATNAO allows vatnao stuff to be nicely written without having #ifdefs
// everywhere! (https://stackoverflow.com/questions/7246512/ifdef-inside-a-macro)
#ifdef RD_USING_VATNAO
#define IF_RD_USING_VATNAO(WHAT) if (vdm != NULL) {WHAT}
#else
#define IF_RD_USING_VATNAO(WHAT) /* do nothing */
#endif


#define WRITE_THRESHOLDED_REGION_DIR "thresholdedRegions/"
// #define WRITE_THRESHOLDED_REGION true
#ifdef WRITE_THRESHOLDED_REGION
    static int num_images = 100000;
#endif // WRITE_THRESHOLDED_REGION

RobotDetector::RobotDetector() {
//#ifdef ROBOT_DETECTOR_USES_VATNAO
//    debugger_ = new RobotDetectorDebugger();
//#endif
	regionFinder = new RobotColorROI();

  IF_RD_USING_VATNAO(
    // Add options in vatnao
    vdm->addOption("Show Candidate base point");
    vdm->addOption("Show whole saliency image");
    topPainter = vdm->getFrameOverlayPainter(4, true);
    botPainter = vdm->getFrameOverlayPainter(4, false);

  )

    #ifndef CTC_2_1

    nn = load_3_layer_cnn();
    #endif
}

Detector *newRobotDetector() {
   return new RobotDetector();
}

void RobotDetector::detect(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {

    IF_RD_USING_VATNAO(
        q = vdm->getQuery();
    )

#ifdef ROBOT_DETECTOR_TIMER
    std::vector<float> timings;
    Timer timer;
    timer.restart();
#endif

    // Declare a new info_middle so we don't edit the actual one
    VisionInfoMiddle editable_middle = info_middle;

    // ColorRoi modifies info_out. We don't want to add my regions to the base regions so give it a dummy.
    VisionInfoOut newOut;

    // Step 1) - Rethreshold downsampled full regions
	runThresholding(info_in, editable_middle, newOut);

  #ifdef WRITE_THRESHOLDED_REGION
    char name[] = "a";
    char location[] = WRITE_THRESHOLDED_REGION_DIR;
    std::stringstream dir;
    dir << location << num_images << ".png";
    num_images++;
    std::cout << "Writing binary sldkfj spot image to: " << dir.str().c_str() << "\n";
    WriteImage w;
    if(w.writeImage(editable_middle.full_regions[0], COLOUR_FORMAT, dir.str().c_str(), name)) {
        std::cout << "Success\n";
    }
    else {
        std::cout << "Failed\n";

    }
    num_images++;
  #endif  // WRITE_THRESHOLDED_REGION


#ifdef ROBOT_DETECTOR_TIMER
    timings.push_back(timer.elapsed_us());
    timer.restart();
#endif

    // Step 2) - Rerun a modified colorROI to generate regions
#ifdef CTC_2_1
    runColorRoi(info_in, editable_middle, info_out);
#else
    runBasepointScanner(info_in, editable_middle, info_out);
#endif



#ifdef ROBOT_DETECTOR_TIMER
    timings.push_back(timer.elapsed_us());
    timer.restart();
#endif

    // Step 3) - Use density based clustering to generate candidates
#ifdef CTC_2_1
    clusters = DBSCAN(editable_middle.roi, 70, 1);
#else
    clusters = createClustersFromBasepoints(editable_middle.basePoints, editable_middle.basePointImageCoords);
#endif
    // clusters = createClustersFromRegions(editable_middle.roi);
    //clusters = createCandidateRobots(editable_middle.roi);
#ifdef ROBOT_DETECTOR_TIMER
    timings.push_back(timer.elapsed_us());
    timer.restart();
#endif
    // Step 40 Run the classifier
    findRobots(editable_middle, info_out, clusters);
#ifdef ROBOT_DETECTOR_TIMER
    timings.push_back(timer.elapsed_us());
    debugger_->outputTimings(timings, editable_middle.roi.size(), clusters.size());
#endif
//#ifdef ROBOT_DETECTOR_USES_VATNAO
//    debugger_->highlightCandidates(clusters);
//#endif
#ifdef OUTPUT_DATA
    debugger_->outputToFile(clusters);
#endif

    delete(newTop_);
    //delete(newBot_);

    return;
}

void RobotDetector::runThresholding(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {

    newTop_ = new RegionI(info_middle.full_regions[0].new_region(Point(0,0),
        Point(info_middle.full_regions[0].getCols(), info_middle.full_regions[0].getRows()), false, 4, ATR_WINDOW,
                                                                                               ATR_MEAN_PERCENT, true));
	//newBot_ = new RegionI(info_middle.full_regions[1].new_region(Point(0,0),
    //    Point(info_middle.full_regions[1].getCols(), info_middle.full_regions[1].getRows()), false, 4, ATR_WINDOW,
    //                                                                                           ATR_MEAN_PERCENT, true));

    info_middle.full_regions[0] = *newTop_;
    //info_middle.full_regions[1] = *newBot_;

    return;
}

void RobotDetector::runColorRoi(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {

	//ensure to clear out existing rois
	info_middle.roi.clear();

	// Run the region finder. This will update info_middle
	regionFinder->find(info_in, info_middle, info_out);

    activatedCounts_ = regionFinder->activatedCounts();

//#ifdef ROBOT_DETECTOR_USES_VATNAO
//	debugger_->highlightRegions(info_middle.roi);
//    debugger_->showSaliencyImage(*newTop_);
//#endif
	return;
}

void RobotDetector::runBasepointScanner(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {

	//ensure to clear out existing rois
	info_middle.roi.clear();

  float windowSize = 10; // pixels
  float RATIO_TO_CONSIDER_ROBOT = 0.8;
  int rows = newTop_->getRows();
  int cols = newTop_->getCols();


  IF_RD_USING_VATNAO(
      if (q.options["Show whole saliency image"] == "true") {
          for (int i = 0; i < cols; ++i)
          {
              for (int j = 0; j < rows; ++j)
              {
                  topPainter->draw(i*4, j*4, newTop_->getPixelColour(i, j) == cWHITE ? VisionPainter::WHITE : VisionPainter::BLACK);
              }
          }
      }
  )

  for (int i = 0; i < cols; ++i)
  {
      float windowValue = 0;
      for (int j = rows - 1; j > rows - windowSize - 1; --j)
      {
          windowValue += newTop_->getPixelColour(i, j) == cWHITE ? 1 : 0;
      }

      for (int j = rows - windowSize -1; j > 0; --j)
      {
          windowValue += newTop_->getPixelColour(i, j) == cWHITE ? 1 : 0;
          windowValue -= newTop_->getPixelColour(i, j + windowSize) == cWHITE ? 1 : 0;

          if (windowValue > windowSize * RATIO_TO_CONSIDER_ROBOT)
          {
              for (int k = j + windowSize; k > j; --k)
              {
                  if (newTop_->getPixelColour(i, k) == cWHITE)
                  {

                      IF_RD_USING_VATNAO(
                          topPainter->draw(i*4, k*4, VisionPainter::GREEN);
                      )

                      Point rawPoint = newTop_->getInternalFovea()->mapFoveaToImage(Point(i, k));
                      RRCoord rr_base = info_out.cameraToRR->convertToRR(rawPoint, true);
                      if (rr_base.distance() < 3000)
                      {
                          info_middle.basePoints.push_back(rr_base.toCartesian());
                          info_middle.basePointImageCoords.push_back(Point(i, k));
                      }
                      //     int robot_width_one_side_in_pixels = 10 / (rr_base.distance() / 1000.f);
                      //     int robot_height_in_pixels = 40 / (rr_base.distance() / 1000.f);
                      //     Point upper_left(i - robot_width_one_side_in_pixels, k - robot_height_in_pixels);
                      //     Point lower_right(i + robot_width_one_side_in_pixels, k + 1);
                      //     RegionI newRegion = RegionI(newTop_->subRegion(upper_left, lower_right));
                      //     info_middle.roi.push_back(newRegion);

                      //     IF_RD_USING_VATNAO(
                      //         topPainter->drawRect(
                      //             (i - robot_width_one_side_in_pixels)*4,
                      //             (k - robot_height_in_pixels)*4,
                      //             (robot_width_one_side_in_pixels * 2)*4,
                      //             (robot_height_in_pixels + 1)*4,
                      //             VisionPainter::CYAN);
                      //     )
                      // }
                      break;
                  }
              }
              break;
          }
      }
  }
}


Eigen::MatrixXf RobotDetector::convertMat(RegionI& region){

    RegionI::iterator_fovea cur_point = region.begin_fovea();
    // Track the literal location of the iterators.
    int x = 0;
    int y = 0;
    // The number of rows and columns in the region.
    int rows = region.getRows();
    int cols = region.getCols();
    Eigen::MatrixXf dst(rows, cols);
    // Loop
    for(int pixel = 0; pixel < cols * rows; ++pixel)
    {
        // std::cout<<cur_point.colour() << " ";
        if (cur_point.colour() == cWHITE){
            dst(y, x) = 1;
        }
        else{
            dst(y, x) = 0;
        }
        cur_point++;
        x++;
        if (x == cols){
            x = 0;
            ++y;
        }
    }
    return dst;
}


void RobotDetector::featureExtraction(VisionInfoMiddle& info_mid, BBox bound, std::vector<float>& features){
    RegionI *newBoundRegion = new RegionI(info_mid.full_regions[0].subRegion(bound));
    features.push_back((float)bound.height()/bound.width());
    features.push_back(bound.height()*bound.width());

    Eigen::MatrixXf imageMat = convertMat(*newBoundRegion);
    features.push_back(imageMat.mean());

    float m00 = imageMat.sum();
    float m10 = 0;
    float m01 = 0;
    for (int i = 0; i < imageMat.rows(); i++){
        for (int j = 0; j < imageMat.cols(); j++){
            m10 += imageMat(i, j) * i;
            m01 += imageMat(i, j) * j;
        }
    }
    features.push_back((m10/m00)/bound.width());
    features.push_back((m01/m00)/bound.height());
    delete newBoundRegion;
}

Eigen::MatrixXf RobotDetector::imageExtraction(VisionInfoMiddle& info_mid, BBox bound){
    RegionI *newBoundRegion = new RegionI(info_mid.full_regions[0].subRegion(bound));

    #ifdef WRITE_THRESHOLDED_REGION
        // RegionI *newBoundRegion = new RegionI(info_mid.full_regions[0].subRegion(candidates[i].fbox_));


        char name[] = "a";
        char location[] = WRITE_THRESHOLDED_REGION_DIR;
        std::stringstream dir;
        dir << location << num_images << ".png";
        num_images++;
        std::cout << "Writing binary sldkfj spot image to: " << dir.str().c_str() << "\n";
        WriteImage w;
        if(w.writeImage(*newBoundRegion, COLOUR_FORMAT, dir.str().c_str(), name)) {
            std::cout << "Success\n";
        }
        else {
            std::cout << "Failed\n";

    }
    num_images++;
#endif  // WRITE_THRESHOLDED_REGION

    Eigen::MatrixXf imageMat = convertMat(*newBoundRegion);
    delete newBoundRegion;
    return imageMat;
}





// Runs the random forest classification and stores the candidates in info_out
void RobotDetector::findRobots(VisionInfoMiddle& info_mid, VisionInfoOut& info_out, std::vector<Cluster>& candidates) {

    std::vector<RobotVisionInfo> robots;

    for(size_t i = 0; i < candidates.size(); ++i) {
        BBox bound = candidates[i].box_;
        if (bound.width() > bound.height()) continue;

        #ifdef CTC_2_1

        std::vector<float> features;
        featureExtraction(info_mid, candidates[i].fbox_, features);
        float confidence;
        int classification = classifier_.classify(features, confidence);
        candidates[i].isRobot_ = classification;
        candidates[i].confidence_ = confidence;
        if(classification == 0||confidence <= 0.7)
            continue;

        #else

        Eigen::MatrixXf image = imageExtraction(info_mid, candidates[i].fbox_);
        Eigen::MatrixXf resized = dnn_resize(image, 32, 32);
        int classification = dnn_predict(resized, nn);
        if (classification == 0)
            continue;
        #endif





        Point(bound.b.x()-bound.a.x(), bound.b.y() + (!candidates[i].isTopCamera()*TOP_IMAGE_ROWS));
        Point feet(bound.width()/2 + bound.a.x(), bound.b.y() + (!candidates[i].isTopCamera()*TOP_IMAGE_ROWS));
        RRCoord rr_feet = info_out.cameraToRR->convertToRR(feet, false);

        // Sometimes we get nan in the observations, ignore these
        if (!check_finite(rr_feet.vec, __FILE__ "\tobservation.rr.vec"))
        {
            std::cerr << "rr_feet:\t" << rr_feet << "\n";
            continue;
        }


        if (candidates[i].isTopCamera())
            robots.push_back(RobotVisionInfo(rr_feet, RobotVisionInfo::rUnknown, bound, RobotVisionInfo::TOP_CAMERA));
        else
            robots.push_back(RobotVisionInfo(rr_feet, RobotVisionInfo::rUnknown, bound, RobotVisionInfo::BOT_CAMERA));

    }

    info_out.robots = robots;

    return;
}

// Naively creates candidate clusters by merging any regions that overlap each other
std::vector<Cluster> RobotDetector::createCandidateRobots(std::vector<RegionI>& regions) {
	std::vector<Cluster> clusters;

    size_t processed = 0;
    std::vector<bool> isProcessed(regions.size(), false);

    //all regions will eventually be added to a cluster (even if its just the one region)
    while(processed < regions.size()) {
        Cluster newCluster = Cluster();
        int merges = -1;
        while(merges != 0) {
            merges = 0;
            for(size_t i = 0; i < regions.size(); ++i) {
                if(isProcessed[i])
                    continue;

                if(regions[i].getCols()/regions[i].getRows() > 8) {
                    processed++;
                    isProcessed[i] = true;
                    continue;
                }
                //if its the first region in the cluster, add it
                else if(newCluster.regions_.empty()) {
                    newCluster.addRegionToCluster(regions[i], activatedCounts_[i]);
                    isProcessed[i] = true;
                    processed++;
                }
                //if it overlaps the cluster add it
                else if(newCluster.overlapsRegion(regions[i])) {
                    newCluster.addRegionToCluster(regions[i], activatedCounts_[i]);
                    isProcessed[i] = true;
                    processed++;
                    merges+=1;
                }
            }
        }
        clusters.push_back(newCluster);
    }
	return clusters;
}

// Creates candidate clusters by merging using a slightly modified Density based clustering algorithm
std::vector<Cluster> RobotDetector::DBSCAN(std::vector<RegionI>& regions, int epsilon, unsigned int minPoints) {
    //-2 = unprocessed
    //-1 = noise
    std::vector<int> labels(regions.size(), -2);

    //cluster label
    int c = 0;

    for(unsigned int regionID = 0; regionID < regions.size(); regionID++) {
        BBox reg = regions[regionID].getBoundingBoxRaw();
        //region already processed so keep going
        if(regions[regionID].getCols()/regions[regionID].getRows() > 8) {
            labels[regionID] = -1;
        }


        if(labels[regionID] != -2)
            continue;


        std::vector<int> neighbours = rangeQuery(regions, regionID, epsilon);
        if(neighbours.size() < minPoints) {
            labels[regionID] = -1; //noise since less than required points
            continue;
        }



        c++;
        labels[regionID] = c;

        for(unsigned int connectedRegion = 0; connectedRegion<neighbours.size(); connectedRegion++) {
            if(labels[neighbours[connectedRegion]] == -1)
                labels[neighbours[connectedRegion]] = c;
            if(labels[neighbours[connectedRegion]] != -2)
                continue;
            labels[neighbours[connectedRegion]] = c;

            std::vector<int> connectedNeighbours = rangeQuery(regions, neighbours[connectedRegion], epsilon);

            if(connectedNeighbours.size() >= minPoints) {
                neighbours.insert(neighbours.end(), connectedNeighbours.begin(), connectedNeighbours.end());
            }
        }
    }


    std::vector<Cluster> clusters;
    //now need to put appropriate regions in their clusters
    for(int i = 0; i < c; ++i) {
        clusters.push_back(Cluster());
    }


    for(unsigned int regionID = 0; regionID != labels.size(); regionID++) {
        if(labels[regionID] >=0) {
            clusters[labels[regionID]-1].addRegionToCluster(regions[regionID], activatedCounts_[regionID]);
        }
    }
    return clusters;
}

std::vector<Cluster> RobotDetector::createClustersFromRegions(std::vector<RegionI>& regions)
{
    std::vector<Cluster> clusters;
    for (unsigned i = 0; i < regions.size(); ++i)
    {
        Cluster cluster;
        cluster.addRegionToCluster(regions[i], 0);
        clusters.push_back(cluster);
    }
    return clusters;
}

std::vector<Cluster> RobotDetector::createClustersFromBasepoints(std::vector<Point> &basePoints, std::vector<Point> &basePointImageCoords)
{
//   std::cout << "basePoints.size(): "<< basePoints.size() << std::endl;

    std::vector<Cluster> clusters;
    // std::vector<std::pair<int, int> > pairs;
    std::vector<std::vector<int> > groups;
    for (unsigned i = 0; i < basePoints.size(); ++i)
    {
        bool addedToGroup = false;
        for (unsigned j = 0; j < basePoints.size(); ++j)
        {
            // don't do anything if its the same point
            if (i == j)
                continue;

            // if points are close, group them
            if ((basePoints[i]-basePoints[j]).norm() < 100)
            {
                // Look through the groups if we already have either of the points
                for (unsigned k = 0; k < groups.size(); ++k)
                {
                    // If first is in group, and second isn't, add it to the group
                    if (std::find(groups[k].begin(), groups[k].end(), i) != groups[k].end() &&
                        std::find(groups[k].begin(), groups[k].end(), j) == groups[k].end())
                    {
                        groups[k].push_back(j);
                        addedToGroup = true;
                        break;
                    }

                    // If second is in group, and first isn't, add it to the group
                    if (std::find(groups[k].begin(), groups[k].end(), j) != groups[k].end() &&
                        std::find(groups[k].begin(), groups[k].end(), i) == groups[k].end())
                    {
                        groups[k].push_back(i);
                        addedToGroup = true;
                        break;
                    }
                }
                // std::cout << i << ", " << j << std::endl;
            }
        }

        if (!addedToGroup)
        {
            std::vector<int> newGroup;
            newGroup.push_back(i);
            groups.push_back(newGroup);
        }
    }

    // std::vector<std::vector<int> > groups;
    // for (unsigned i = 0; i < pairs.size(); ++i)
    // {
    //     bool addedToGroup = false;
    //     for (unsigned j = 0; j < groups.size(); ++j)
    //     {
    //         // If first is in group, and second isn't, add it to the group
    //         if (std::find(groups[j].begin(), groups[j].end(), pairs[i].first) != groups[j].end() &&
    //             std::find(groups[j].begin(), groups[j].end(), pairs[i].second) == groups[j].end())
    //         {
    //             groups[j].push_back(pairs[i].second);
    //             addedToGroup = true;
    //             break;
    //         }

    //         // If second is in group, and first isn't, add it to the group
    //         if (std::find(groups[j].begin(), groups[j].end(), pairs[i].second) != groups[j].end() &&
    //             std::find(groups[j].begin(), groups[j].end(), pairs[i].first) == groups[j].end())
    //         {
    //             groups[j].push_back(pairs[i].first);
    //             addedToGroup = true;
    //             break;
    //         }
    //     }
    //     if (!addedToGroup)
    //     {
    //         std::vector<int> newGroup;
    //         newGroup.push_back(pairs[i].first);
    //         newGroup.push_back(pairs[i].second);
    //         groups.push_back(newGroup);
    //     }
    // }

    // for (unsigned i = 0; i < groups.size(); ++i)
    // {
    //     for (unsigned j = 0; j < groups[i].size(); ++j)
    //     {
    //         std::cout << groups[i][j];
    //     }
    //     std::cout << std::endl;
    // }

    std::vector<RegionI> regions;
    for (unsigned i = 0; i < groups.size(); ++i)
    {
        Point averageImageCoord(0, 0);
        Point averageRRCoord(0, 0);
        for (unsigned j = 0; j < groups[i].size(); ++j)
        {
            averageImageCoord += basePointImageCoords[groups[i][j]];
            averageRRCoord += basePoints[groups[i][j]];
        }
        averageImageCoord /= groups[i].size();
        averageRRCoord /= groups[i].size();

        float distance = averageRRCoord.norm();

        int robot_width_one_side_in_pixels = 10 / (distance / 1000.f);
        int robot_height_in_pixels = 40 / (distance / 1000.f);
        Point upper_left(averageImageCoord[0] - robot_width_one_side_in_pixels, averageImageCoord[1] - robot_height_in_pixels);
        Point lower_right(averageImageCoord[0] + robot_width_one_side_in_pixels, averageImageCoord[1] + 1);
        RegionI newRegion = RegionI(newTop_->subRegion(upper_left, lower_right));
        // info_middle.roi.push_back(newRegion);

        regions.push_back(newRegion);
    }

    std::vector<int> merged(regions.size(), 1);
    for (unsigned int regionID = 0; regionID < regions.size(); regionID ++){
        for (unsigned int next = regionID + 1; next < regions.size(); next ++){
            if (overlaps(regions[regionID], regions[next])){
                merged[regionID] = 0;
                // TODO EXPAND REGIONS AND PUT IT TO regions[next]

                BBox current_box = regions[regionID].getBoundingBoxFovea();
                BBox next_box = regions[next].getBoundingBoxFovea();

                int upper_left_x = std::min(current_box.a.x(), next_box.a.x());
                int upper_left_y = std::min(current_box.a.y(), next_box.a.y());
                Point upper_left(upper_left_x, upper_left_y);

                int lower_right_x = std::max(current_box.b.x(), next_box.b.x());
                int lower_right_y = std::max(current_box.b.y(), next_box.b.y());
                Point lower_right(lower_right_x, lower_right_y);

                RegionI newRegion = RegionI(newTop_->subRegion(upper_left, lower_right));
                regions[next] = newRegion;
                continue;
            }


        }
    }

    for (unsigned int regionID = 0; regionID < regions.size(); regionID ++){

        if (merged[regionID] == 0) continue;

        Cluster cluster;
        cluster.addRegionToCluster(regions[regionID], 0);
        clusters.push_back(cluster);

        BBox show_box = regions[regionID].getBoundingBoxFovea();
        IF_RD_USING_VATNAO(
            topPainter->drawRect(
                show_box.a.x()*4,
                show_box.a.y()*4,
                ( show_box.b.x() -  show_box.a.x())*4,
                ( show_box.b.y() -  show_box.a.y())*4,
                VisionPainter::CYAN);
        )
    }

    // std::cout << std::endl;

    return clusters;
}

// Finds the neighbours for a particular region (governed by currRegionID) and epsilon
std::vector<int> RobotDetector::rangeQuery(std::vector<RegionI>& regions, int currRegionID, int epsilon) {
    std::vector<int> neighbours;
    for(unsigned int regionID = 0; regionID < regions.size(); regionID++) {
        if(regions[regionID].getCols()/regions[regionID].getRows() > 8) {
            continue;
        }
        if(regions[regionID].isTopCamera() != regions[currRegionID].isTopCamera()) {
            continue;
        }
        BBox currRegBox = regions[currRegionID].getBoundingBoxRaw();

    // Classify the region as a neighbour if it either overlaps the region or is within epsilon of it
    if(distance(regions[currRegionID], regions[regionID]) <= epsilon || overlaps(regions[currRegionID], regions[regionID])) {
            neighbours.push_back(regionID);
        }
    }
    return neighbours;
}

//note could have a variable epsilon based on size of region.

//distance between the centrepoints of the square.
int RobotDetector::distance(const RegionI& regionA, const RegionI& regionB) {

    double aX = (regionA.getBoundingBoxRaw().b.x() + regionA.getBoundingBoxRaw().a.x())/2;
    double aY = (regionA.getBoundingBoxRaw().b.y() + regionA.getBoundingBoxRaw().a.y())/2;
    double bX = (regionB.getBoundingBoxRaw().b.x() + regionB.getBoundingBoxRaw().a.x())/2;
    double bY = (regionB.getBoundingBoxRaw().b.y() + regionB.getBoundingBoxRaw().a.y())/2;


    double distance = std::sqrt((aX-bX)*(aX-bX) + (aY-bY)*(aY-bY));

    return distance;
}

// Determines whether regionB is in the elipse of region A
bool RobotDetector::inEllipse(const RegionI& regionA, const RegionI& regionB) {
    double aX = (regionA.getBoundingBoxRaw().b.x() + regionA.getBoundingBoxRaw().a.x())/2;
    double aY = (regionA.getBoundingBoxRaw().b.y() + regionA.getBoundingBoxRaw().a.y())/2;
    double bX = (regionB.getBoundingBoxRaw().b.x() + regionB.getBoundingBoxRaw().a.x())/2;
    double bY = (regionB.getBoundingBoxRaw().b.y() + regionB.getBoundingBoxRaw().a.y())/2;

    // Modfy the size of the ellipse by setting these parameters
    float rx = (regionA.getBoundingBoxRaw().width())/1;
    float ry = (regionA.getBoundingBoxRaw().height())/1;

    if(((bX-aX)*(bX-aX)/(rx*rx))+((bY-aY)*(bY-aY))/(ry*ry) <= 1) {
        return true;
    }
    return false;

    //equation of ellipse

}

// Determines whether 2 regions overlap each other
bool RobotDetector::overlaps(RegionI& regionA, RegionI& regionB) {
    Cluster newCluster;
    newCluster.addRegionToCluster(regionA, 0);
    return newCluster.overlapsRegion(regionB);
}




/******************************* DEBUGGER *************************/
// #ifdef ROBOT_DETECTOR_USES_VATNAO
// RobotDetectorDebugger::RobotDetectorDebugger() {
// 	if (vdm != NULL) {
//         	vdm->addOption("Show RobotDetector Regions");
//         	vdm->addOption("Show whole saliency image");
// 		vdm->addOption("Show candidate robots");
//     }
//     dataHeaderWritten_ = false;
//     dataHeaderWrittenTimings_ = false;
// }

// void RobotDetectorDebugger::highlightRegions(std::vector<RegionI>& regions) {
//     //first check that vision debugger module exists (this is defined elsewhere so you shouldn't need to worry about it)
//     if(vdm != NULL) {

//         // create a query object to access vatnao UI information
//         VisionDebugQuery myQ = vdm->getQuery();

//         // check whether the 'tick box' is selected for showing curved regions
//         if (myQ.options["Show RobotDetector Regions"] == "true") {
//             for(size_t i = 0; i < regions.size(); ++i) {
//                 RegionI currRegion = regions[i];
//                 // get the bounding box of the region
//                 BBox boundingBox = currRegion.getBoundingBoxRaw();

//                 // Prety sure the painter function starts from the top left of the region
//                 int topLeftX = boundingBox.a.x();
//                 int topLeftY = boundingBox.a.y();

//                 // need to create a vatnao painter object which is our interface into the vatnao display frame
//                 // 5 defines the factor of downsampling for the painter. true = top camera, false = bottom
//                 // We want to downsample the image because that means we will get thicker lines drawn by vatnao
//                 VisionPainter *myPainterTop = vdm->getFrameOverlayPainter(5, true);
//                 VisionPainter *myPainterBot = vdm->getFrameOverlayPainter(5, false);

//                 // this function is defined in VisionDebuggerInterface.hpp
//                 // We again scale the coordinate points so that they match up with the coordinate system used by the painter
//                 if(currRegion.isTopCamera())
//                     myPainterTop->drawRect(topLeftX/5, topLeftY/5, boundingBox.width()/5, boundingBox.height()/5,  VisionPainter::RED);
//                 else
//                     myPainterBot->drawRect(topLeftX/5, topLeftY/5, boundingBox.width()/5, boundingBox.height()/5,  VisionPainter::RED);
//             }
//         }
//     }
// 	return;
// }


// void RobotDetectorDebugger::highlightCandidates(std::vector<Cluster>& candidates) {
//     //first check that vision debugger module exists (this is defined elsewhere so you shouldn't need to worry about it)
//     if(vdm != NULL) {

//         // create a query object to access vatnao UI information
//         VisionDebugQuery myQ = vdm->getQuery();

//         // check whether the 'tick box' is selected for showing curved regions
//         if (myQ.options["Show candidate robots"] == "true") {
//             for(size_t i = 0; i < candidates.size(); ++i) {
//                 Cluster currCandidate = candidates[i];
//                 // get the bounding box of the region
//                 BBox boundingBox = currCandidate.box_;

//                 // Prety sure the painter function starts from the top left of the region
//                 int topLeftX = boundingBox.a.x();
//                 int topLeftY = boundingBox.a.y();

//                 // need to create a vatnao painter object which is our interface into the vatnao display frame
//                 // 5 defines the factor of downsampling for the painter. true = top camera, false = bottom
//                 // We want to downsample the image because that means we will get thicker lines drawn by vatnao
//                 VisionPainter *myPainterTop = vdm->getFrameOverlayPainter(5, true);
//                 VisionPainter *myPainterBot = vdm->getFrameOverlayPainter(5, false);

//                 // this function is defined in VisionDebuggerInterface.hpp
//                 // We again scale the coordinate points so that they match up with the coordinate system used by the painter
// 			if(currCandidate.isTopCamera())
//                     myPainterTop->drawRect(topLeftX/5, topLeftY/5, boundingBox.width()/5, boundingBox.height()/5,  VisionPainter::PINK);
//                 else
//                     myPainterBot->drawRect(topLeftX/5, topLeftY/5, boundingBox.width()/5, boundingBox.height()/5,  VisionPainter::PINK);
//             }
//         }
//     }
// 	return;
// }


// void RobotDetectorDebugger::showSaliencyImage(const RegionI& region) {
//     if (vdm != NULL) {
//         VisionDebugQuery myQ = vdm->getQuery();
//         VisionPainter *p = vdm->getGivenRegionOverlayPainter(region);
//         if (myQ.options["Show whole saliency image"] == "true") {
//             p->drawColourSaliency(region);
//         }
//     }
// }

// void RobotDetectorDebugger::outputToFile(std::vector<Cluster>& candidates) {
//     if(vdm != NULL) {
//         std::string dataFileName_ = vdm->getFilename();
//         // If we haven't written the header row then we need to do that first
//         // This will also clear the existing file
//         if(!dataHeaderWritten_) {
//             std::ofstream dataFile;
//             dataFile.open(dataFileName_.c_str());
//             dataFile << "FrameNum, x, y, isTopCamera, height, width, whites, numRegions, classification, confidence, realRobot\n";
//             dataFile.close();
//             dataHeaderWritten_ = true;
//         }

//         unsigned int currFrame = vdm->getCurrentFrame();
//         if(framesWritten_.count(currFrame) != 0)
//             return;
//         std::ofstream dataFile;
//         dataFile.open(dataFileName_.c_str(), std::ios::app);
//         for(size_t i = 0; i < candidates.size(); ++i) {
//             Point foot = candidates[i].getBaseCenter();
//             std::stringstream myLine;
//             myLine << currFrame;
//             myLine << "," << foot[0];
//             myLine << "," << foot[1];
//             myLine << "," << candidates[i].isTopCamera();
//             myLine << "," << candidates[i].box_.height();
//             myLine << "," << candidates[i].box_.width();
//             myLine << "," << candidates[i].whitePixels_;
//             myLine << "," << candidates[i].regions_.size();
//             myLine << "," << candidates[i].isRobot_;
//             myLine << "," << candidates[i].confidence_;
//             myLine << "," << 0;

//             dataFile << myLine.str() <<"\n";
//         }
//         dataFile.close();
//         framesWritten_[currFrame] = true;
//     }
//     return;
// }

// void RobotDetectorDebugger::outputTimings(std::vector<float>& timings, int numRegions, int numCandidates) {
//     if(vdm != NULL) {
//         std::string dataFileName_ = vdm->getFilename();
//         dataFileName_.resize(dataFileName_.length()-8);
//         dataFileName_ += "timings.csv";
//         // If we haven't written the header row then we need to do that first
//         // This will also clear the existing file
//         if(!dataHeaderWrittenTimings_) {
//             std::ofstream dataFile;
//             dataFile.open(dataFileName_.c_str());
//             dataFile << "FrameNum, numRegions, numCandidates, thresholding, colorRoi, clustering, classification, total\n";
//             dataFile.close();
//             dataHeaderWrittenTimings_ = true;
//             // HACK first classification value is fucked. so hard code it
//             // so that the results aren't effed
//             timings[3] = 15;
//         }

//         unsigned int currFrame = vdm->getCurrentFrame();
//         if(framesWritten_.count(currFrame) != 0)
//             return;
//         std::ofstream dataFile;
//         dataFile.open(dataFileName_.c_str(), std::ios::app);
//         std::stringstream myLine;
//         myLine << currFrame;
//         myLine << "," << numRegions;
//         myLine << "," << numCandidates;
//         myLine << "," << timings[0];
//         myLine << "," << timings[1];
//         myLine << "," << timings[2];
//         myLine << "," << timings[3];
//         myLine << "," << (timings[0]+timings[1]+timings[2]+timings[3]);

//         dataFile << myLine.str() <<"\n";
//         dataFile.close();

//     }
//     return;

// }
// #endif

//-------------------------------- Cluster ----------------------
Cluster::Cluster() :
    box_(BBox(Point(0,0), Point(0,0))), fbox_(BBox(Point(0,0), Point(0,0))),  whitePixels_(0), isRobot_(false) {};

void Cluster::addRegionToCluster(const RegionI& region, int whiteCount) {
    bool firstRegion = regions_.size() == 0 ? true : false;
    regions_.push_back(region);
    BBox bb = region.getBoundingBoxRaw();
    BBox fbb = region.getBoundingBoxFovea();
    if (box_.a.y() > bb.a.y() || firstRegion) {
        box_.a.y() = bb.a.y();
        fbox_.a.y() = fbb.a.y();
    }
    if (box_.b.y() < bb.b.y() || firstRegion) {
        box_.b.y() = bb.b.y();
        fbox_.b.y() = fbb.b.y();
    }
    if (box_.a.x() > bb.a.x() || firstRegion){
        box_.a.x() = bb.a.x();
        fbox_.a.x() = fbb.a.x();
    }
    if (box_.b.x() < bb.b.x() || firstRegion) {
        box_.b.x() = bb.b.x();
        fbox_.b.x() = fbb.b.x();
    }
    //whitePixels_ += whiteCount;
};

bool Cluster::overlapsRegion(RegionI& region) {
    BBox rBox = region.getBoundingBoxRaw();
    if((region.isTopCamera() && isTopCamera()) ||
        (!region.isTopCamera() && !isTopCamera()))    {
        if(std::max(box_.a.x(), rBox.a.x()) <= std::min(box_.b.x(), rBox.b.x())
            && std::max(box_.a.y(), rBox.a.y()) <= std::min(box_.b.y(), rBox.b.y())) {
                return true;
        }
    }
    return false;
}

Point Cluster::getBaseCenter() {
    return Point(box_.a.x() + box_.width()/2, box_.b.y());
}

#ifndef CTC_2_1
tiny_dnn::network<sequential> RobotDetector::load_3_layer_cnn(){

    using conv     = tiny_dnn::convolutional_layer;
    using max_pool = tiny_dnn::max_pooling_layer;
    using fc       = tiny_dnn::fully_connected_layer;
    using relu     = tiny_dnn::relu_layer;
    using tiny_dnn::core::connection_table;


    tiny_dnn::network<sequential> nn;
    nn << conv(32,32,3,1,4) /* 32x32 in, 5x5 kernel, 1-6 fmaps conv */
        << relu(30,30,4)
        << max_pool(30, 30, 4, 2) /* 28x28 in, 6 fmaps, 2x2 subsampling */
        << conv(15, 15, 3, 4, 8) // layer 3
        << relu(13, 13, 8)
        << max_pool(13, 13, 8, 2)
        << conv(6, 6, 3, 8, 8, padding::valid, true, 2, 2) // layer 6
        << relu(2, 2, 8)
        << max_pool(2, 2, 8, 2)
        << fc(1 * 1 * 8, 8) // layer 9
        << relu(1, 1, 8)
        << fc(8, 2);


  //   for (int i = 0; i < nn.depth(); i++)
	// {
	// 	std::cout << "#layer:" << i << "\n";
	// 	std::cout << "layer type:" << nn[i]->layer_type() << "\n";
	// 	std::cout << "input:" << nn[i]->in_data_size() << "(" << nn[i]->in_data_shape() << ")\n";
	// 	std::cout << "output:" << nn[i]->out_data_size() << "(" << nn[i]->out_data_shape() << ")\n";

	// }

    std::string weight_path;
    std::string bias_path;
    std::string root_path = getHomeNao(DNN_WEIGHTS_DIR);

    std::vector<vec_t*> weights_input_layer0 = nn[0]->weights();
    weight_path = root_path + "conv_0_w.txt";
    bias_path = root_path + "conv_0_b.txt";
    Read_File(*weights_input_layer0[0], weight_path);
    Read_File(*weights_input_layer0[1], bias_path);

    weight_path = root_path + "conv_1_w.txt";
    bias_path = root_path + "conv_1_b.txt";
    std::vector<vec_t*> weights_layer0_layer1 = nn[3]->weights();
    Read_File(*weights_layer0_layer1[0], weight_path);
    Read_File(*weights_layer0_layer1[1], bias_path);

    weight_path = root_path + "conv_2_w.txt";
    bias_path = root_path + "conv_2_b.txt";
    std::vector<vec_t*> weights_layer1_layer2 = nn[6]->weights();
    Read_File(*weights_layer1_layer2[0], weight_path);
    Read_File(*weights_layer1_layer2[1], bias_path);

    weight_path = root_path + "fc_1_w.txt";
    bias_path = root_path + "fc_1_b.txt";
    std::vector<vec_t*> weights_layer2_layer3 = nn[9]->weights();
    Read_File(*weights_layer2_layer3[0], weight_path);
    Read_File(*weights_layer2_layer3[1], bias_path);

    weight_path = root_path + "out_1_w.txt";
    bias_path = root_path + "out_1_b.txt";
    std::vector<vec_t*> weights_layer3_output = nn[11]->weights();
    Read_File(*weights_layer3_output[0], weight_path);
    Read_File(*weights_layer3_output[1], bias_path);

    return nn;
}

Eigen::MatrixXf RobotDetector::dnn_resize(Eigen::MatrixXf &img, int resize_height, int resize_width)
{
    /*
	Resize the input image with Bilinear Interpolation method.

	*/
	int rows = img.rows();
    int cols = img.cols();
    float deltaX = (float)(rows) / resize_width;
    float lastX = (0.5 * rows) / resize_width - 0.5;
    // float x = lastX;
    float deltaY = (float)(cols) / resize_height;
    float lastY = (0.5 * cols) / resize_height - 0.5;
    float y = lastY;
    Eigen::MatrixXf dst(resize_width, resize_height);
    for(int i = 0; i < resize_width; ++i) {
        //float x = (i+0.5)*rows/NEW_SIZE-0.5;
        float x = lastX;
        lastX += deltaX;
        int fx = (int)x;
        x -= fx;
        short x1 = (1.f - x) * 2048;
        short x2 = 2048 - x1;
        if (fx >= rows - 1){
            fx = rows - 2;
        }
        lastY = (0.5 * cols) / resize_height -0.5;
        for(int j = 0; j < resize_height; ++j) {
            //float y = (j+0.5)*cols/NEW_SIZE-0.5;
            y = lastY;
            lastY += deltaY;

            int fy = (int)y;
            y -= fy;
            if (fy >= cols - 1){
                fy = cols - 2;
            }
            short y1 = (1.f - y) * 2048;
            short y2 = 2048 - y1;
            dst(i, j) = ((int)img(fx, fy) * x1 * y1 + (int)img(fx + 1, fy) * x2 * y1
                        + (int)img(fx, fy + 1) * x1 * y2 + (int)img(fx + 1, fy + 1) * x2 * y2) >> 22;

        }
    }
    return dst;
}


int RobotDetector::dnn_predict(Eigen::MatrixXf &img, network<sequential> nn){

    vec_t dnn_image;
    for (int i = 0; i < img.rows(); i++){
        for (int j = 0; j < img.cols(); j++){
            dnn_image.push_back(img(i, j));
        }
    }
    vec_t result = nn.predict(dnn_image);
    if (result[0] > result[1]){
        return 0;
    }
    return 1;
}
#endif
