#include "tab.hpp"

#include "utils/OverlayPainter.hpp"
#include "perception/vision/Vision.hpp"
#include "classifier.hpp"
#include "perception/vision/other/YUV.hpp"

using namespace std;

QRgb Tab::getRGB(unsigned int col,
                 unsigned int row,
                 const uint8_t *yuv,
                 int num_cols) {

   int y = gety(yuv, row, col, num_cols);
   int u = getu(yuv, row, col, num_cols);
   int v = getv(yuv, row, col, num_cols);
// 80
   if (y < 130 && u < 145 && v < 238) { // 130, 137, 238
      //return Classifier::yuv2rgb(0,0,0);//Classifier::yuv2rgb(255,255,0);
   }
   // 80,80,80 in RGB = 85,128,140
   // 50,70,80 in RGB = 72,135,120
   if (y < 85 && u < 128 && v < 140) {// || (/*u < 85 ||*/ u > 170) || (/*v < 140 ||*/ v > 170)) {
      //return Classifier::yuv2rgb(0,0,0);
   }


   return Classifier::yuv2rgb(gety(yuv, row, col, num_cols),
                              getu(yuv, row, col, num_cols),
                              getv(yuv, row, col, num_cols));
}

// clamps an integer to [0...255]
// OpenCV has a saturate_cast, but byte me
uint8_t byteMe(int value) {
   // if value <  0, mask = 0
   // if value >= 0, mask = 0xffffffff
   int mask = -(value >= 0);

   // if value <  0, value = 0
   // if value >= 0, value = value
   value = value & mask;

   // if value <  255, mask = 0
   // if value >= 255, mask = 0xffffffff
   mask = -(value >= 255);

   // if value <  255, return value
   // if value >= 255, return 255
   return value | mask;
}

// inline version of Classifier::yuv2rgb with some tweaks to use only integer arithmetic,
// as suggested by https://en.wikipedia.org/wiki/YUV#Y%E2%80%B2UV444_to_RGB888_conversion
// actual formula from https://en.wikipedia.org/wiki/YCbCr#ITU-R_BT.601_conversion
// overall about 4x faster
// could be faster by having a variable for 298 * C, 409 * E, etc., but meh
void Tab::drawImage(QImage *image, bool top) {
   int R               = (top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
   int C               = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
   int BYTES_PER_PIXEL = 2;

   const uint8_t *const frame    = top ? topFrame : botFrame;
   const uint8_t *const frameEnd = frame + R * C * BYTES_PER_PIXEL;

   uint64_t            *data      = reinterpret_cast<uint64_t *>(image->bits());
   for (const uint32_t *pixelPair = reinterpret_cast<const uint32_t *>(frame);
        pixelPair < reinterpret_cast<const uint32_t *>(frameEnd);
        ++pixelPair, ++data) {
      // vyuy in little-endian
      uint32_t  yuyv = *pixelPair;
      //             Y'
      const int C1   = (yuyv & 0xffu) - 16;
      yuyv >>= 8u;
      //            C_B
      const int D = (yuyv & 0xffu) - 128;
      yuyv >>= 8u;
      //             Y'
      const int C2 = (yuyv & 0xffu) - 16;
      yuyv >>= 8u;
      //            C_R
      const int E = (yuyv & 0xffu) - 128;
      yuyv >>= 8u;

      uint64_t qrgb2 = 0xff;
      // R_D
      qrgb2 <<= 8u;
      qrgb2 |= byteMe((298 * C2 + 000 * D + 409 * E) >> 8u);
      // G_D
      qrgb2 <<= 8u;
      qrgb2 |= byteMe((298 * C2 - 100 * D - 208 * E) >> 8u);
      // B_D
      qrgb2 <<= 8u;
      qrgb2 |= byteMe((298 * C2 + 516 * D + 000 * E) >> 8u);

      qrgb2 <<= 8u;
      qrgb2 |= 0xff;
      // R_D
      qrgb2 <<= 8u;
      qrgb2 |= byteMe((298 * C1 + 000 * D + 409 * E) >> 8u);
      // G_D
      qrgb2 <<= 8u;
      qrgb2 |= byteMe((298 * C1 - 100 * D - 208 * E) >> 8u);
      // B_D
      qrgb2 <<= 8u;
      qrgb2 |= byteMe((298 * C1 + 516 * D + 000 * E) >> 8u);

      *data = qrgb2;
   }
}

void Tab::drawOverlaysGeneric (QPaintDevice *topImage,
      QPaintDevice                          *botImage,
      const std::pair<int, int>             *horizon,
      const std::vector<BallInfo>           *balls,
      const std::vector<RobotVisionInfo>    *robots,
      const std::vector<FieldBoundaryInfo>  *fieldBoundaries,
      const std::vector<FieldFeatureInfo>   *fieldFeatures,
      const std::vector<RegionI>            *regions,
      float scale)
{
   OverlayPainter topPainter;
   OverlayPainter botPainter;

   if (topImage) {
      topPainter.begin(topImage);
      topPainter.scale (scale, scale);
   }

   if (botImage){
      botPainter.begin(botImage);
      botPainter.scale (scale, scale);
      botPainter.translate(0, -TOP_IMAGE_ROWS);
   }

   if (topImage && horizon) {
      topPainter.drawHorizon(*horizon);
   }

   if (balls) {
      std::vector<BallInfo>::const_iterator i;
      for (i = balls->begin (); i != balls->end (); ++ i) {
         if (topImage) topPainter.drawBallOverlay (*i);
         if (botImage) botPainter.drawBallOverlay (*i);
      }
   }

   if (robots) {
      std::vector<RobotVisionInfo>::const_iterator i;
      for (i = robots->begin (); i != robots->end (); ++ i) {
         if (topImage) topPainter.drawRobotOverlay (*i);
         if (botImage) botPainter.drawRobotOverlay (*i);
      }
   }

   if (fieldBoundaries) {
      std::vector<FieldBoundaryInfo>::const_iterator i;
      for (i = fieldBoundaries->begin (); i != fieldBoundaries->end (); ++ i) {
         if (topImage) topPainter.drawFieldBoundaryOverlay (*i);
         if (botImage) botPainter.drawFieldBoundaryOverlay (*i);
      }
   }

   if (fieldFeatures) {
      std::vector<FieldFeatureInfo>::const_iterator i;
      for (i = fieldFeatures->begin (); i != fieldFeatures->end (); ++ i) {
         if (topImage) topPainter.drawFieldFeatureOverlay (*i);
         if (botImage) botPainter.drawFieldFeatureOverlay (*i);
      }
   }

   if (regions) {
      std::vector<RegionI>::const_iterator i;
      for (i = regions->begin (); i != regions->end (); ++ i) {
          if(i->isTopCamera() && topImage)
            topPainter.drawRegionOverlay(*i);
          if(!i->isTopCamera() && botImage)
            botPainter.drawRegionOverlay(*i);
      }
   }

   if (topImage) topPainter.end();
   if (botImage) botPainter.end();
}

void Tab::tabSelected()
{
}

void Tab::tabDeselected()
{
}

#if 0
      vector<std::pair<uint16_t, uint16_t> > *edgePoints,
      FieldEdge edgeLines[2],
      uint8_t numEdgeLines,
      uint16_t *goals,
      uint16_t numPosts,
      WhichPosts posts,
      ImageRegion *regions,
      uint16_t numRegions,
      uint16_t radius,
      std::pair<uint16_t, uint16_t> ballCentre,
      std::pair<uint16_t, uint16_t> *ballEdgePoints,
      uint16_t numBallEdgePoints,
      RobotRegion **robotRegions,
      uint16_t numRobotRegions,
      ImageRegion **lineRegions,
      uint16_t numLineRegions,
      uint16_t numFeet,
      uint16_t *footImageCoords,
      float scale) {
   QPainter painter(image);
   if (edgePoints != 0) {
      painter.setPen(QColor(1, 0, 0));
      painter.setBrush(QBrush(QColor(255, 255, 0)));
      vector<pair<uint16_t, uint16_t> >::const_iterator p;
      for (p = edgePoints->begin(); p != edgePoints->end(); ++p) {
         painter.drawEllipse(QPoint((*p).first*scale, (*p).second)*scale, 2, 2);
      }
   }

   if (edgeLines != 0) {
      painter.setPen(QColor(255, 255, 0));
      for (int i = 0; i < numEdgeLines; ++i) {
         painter.drawLine(
               0, -1*(float)edgeLines[i].t3/edgeLines[i].t2 * scale,
               (IMAGE_COLS-1)*scale,
               scale*((-1*(float)edgeLines[i].t3
                - ((float)edgeLines[i].t1)*(IMAGE_COLS-1))
               /edgeLines[i].t2));
               painter.setPen(QColor(255, 0, 0));
      }
   }

   if (numPosts != 0 && goals != 0) {
      if (posts >= p_legacy_YELLOW_LEFT) {
         painter.setPen(QColor(255, 255, 0));
      } else {
         painter.setPen(QColor(0, 255, 255));
      }
      for (uint16_t p = 0; p < numPosts*4; p += 4) {
         QLineF lines[4] = {
            QLineF(scale*goals[p], scale*goals[p+1], scale*goals[p+2],
                   scale*goals[p+1]),
            QLineF(scale*goals[p+2], scale*goals[p+1], scale*goals[p+2],
                   scale*goals[p+3]),
            QLineF(scale*goals[p+2], scale*goals[p+3], scale*goals[p],
                   scale*goals[p+3]),
            QLineF(scale*goals[p], scale*goals[p+3], scale*goals[p],
                   scale*goals[p+1])
         };
         painter.drawLines(lines, 4);
      }
   }

   if (radius != 0) {
      painter.setPen(QColor(0, 0, 0));
      painter.setBrush(QBrush(Qt::NoBrush));
      painter.drawEllipse(QPoint(scale * ballCentre.first,
              scale * ballCentre.second),
              (int)(scale * radius), (int)(scale * radius));
      painter.setPen(QColor(255, 255, 255));
      int i;
      for (i = 0; i < numBallEdgePoints; i++) {
         painter.drawEllipse(QPoint(ballEdgePoints[i].first,
                  ballEdgePoints[i].second), 1, 1);
      }
      painter.setPen(QColor(0,255,0));
      /*if (numBallEdgePoints != 0) {
         for (; i < numBallEdgePoints + NUM_CENTRE_REPEATS; i++) {
            painter.drawEllipse(QPoint(ballEdgePoints[i].first,
                     ballEdgePoints[i].second), 1, 1);
         }
      }*/
      /*painter.drawEllipse(QPoint(scale*ballCentre.first,
                          scale*ballCentre.second),
                          (int)(scale*radius), (int)(scale*radius));*/
   }

   if (numFeet > 0){
      painter.setPen(QColor(0, 0, 255));
      painter.setBrush(QBrush(Qt::NoBrush));
      painter.drawEllipse(QPoint(scale * footImageCoords[0],
              scale * footImageCoords[1]),
              (int)(scale * FOOT_RADIUS), (int)(scale * FOOT_RADIUS));
      if (numFeet > 1){
         painter.drawEllipse(QPoint(scale * footImageCoords[2],
              scale * footImageCoords[3]),
              (int)(scale * FOOT_RADIUS), (int)(scale * FOOT_RADIUS));
      }
   }

   if (numRobotRegions != 0) {
      painter.setPen(QColor(255, 255, 255));
      for (uint16_t p = 0; p < numRobotRegions; p++) {

         if (robotRegions[p]->type == RED_ROBOT) {
            painter.setPen(QColor(255, 0, 0));
         } else if (robotRegions[p]->type == BLUE_ROBOT) {
            painter.setPen(QColor(0, 0, 255));
         } else {
            painter.setPen(QColor(0, 50, 0));
         }

         QLineF lines[4] = {
            QLineF(robotRegions[p]->leftMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->bottomMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->leftMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->topMost * SALIENCY_DENSITY * scale),
            QLineF(robotRegions[p]->leftMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->topMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->rightMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->topMost * SALIENCY_DENSITY * scale),
            QLineF(robotRegions[p]->rightMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->topMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->rightMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->bottomMost * SALIENCY_DENSITY * scale),
            QLineF(robotRegions[p]->leftMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->bottomMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->rightMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->bottomMost * SALIENCY_DENSITY * scale)
            };
            painter.drawLines(lines, 4);
      }
   }

   if (numLineRegions != 0 && numRegions == 0) {
      painter.setPen(QColor(255, 0, 255));
      for (uint16_t p = 0; p < numLineRegions; p++) {
         for (uint16_t i = 0; i < lineRegions[p]->numScanPoints; i++) {
            painter.drawEllipse(
                 lineRegions[p]->startScans[i].first * SALIENCY_DENSITY,
                  lineRegions[p]->startScans[i].second * SALIENCY_DENSITY,
                  2, 2
                  );
            painter.drawEllipse(
                  lineRegions[p]->endScans[i].first * SALIENCY_DENSITY,
                  lineRegions[p]->endScans[i].second * SALIENCY_DENSITY,
                  2, 2
                  );
         }
      }
   }

   if (numRegions != 0) {
      for (uint16_t p = 0; p < numRegions; p++) {
         RegionType status = regions[p].classification;
         if (status == rBALL) {
            painter.setPen(QColor(128, 20, 20));
         } else if (status == rROBOT) {
            painter.setPen(QColor(255, 255, 255));
         } else if (status == rMAYBE_ROBOT) {
            painter.setPen(QColor(255, 255, 0));
         } else if (status == rFIELD_LINE) {
            painter.setPen(QColor(255, 0, 255));
         } else if (status == rCONNECTED_ROBOT) {
            painter.setPen(QColor(0, 255, 0));
         } else {
            painter.setPen(QColor(0, 255, 255));
         }
         if (!(regions[p].deleted)) {
            /*painter.drawEllipse(
                  regions[p].centreGravity.first * SALIENCY_DENSITY,
                  regions[p].centreGravity.second * SALIENCY_DENSITY,
                  2, 2
                  );*/
            // Draw the field lines
            // if (status == rFIELD_LINE) {
               for (uint16_t i = 0; i < regions[p].numScanPoints; i++) {
                  painter.drawEllipse(
                        regions[p].startScans[i].first * SALIENCY_DENSITY,
                        regions[p].startScans[i].second * SALIENCY_DENSITY,
                        2, 2
                        );
                  painter.drawEllipse(
                        regions[p].endScans[i].first * SALIENCY_DENSITY,
                        regions[p].endScans[i].second * SALIENCY_DENSITY,
                        2, 2
                        );
               }
            // }

            QLineF lines[4] = {
               QLineF(regions[p].leftMost * SALIENCY_DENSITY,
                     regions[p].bottomMost.second * SALIENCY_DENSITY,
                     regions[p].leftMost * SALIENCY_DENSITY,
                     regions[p].topMost.second * SALIENCY_DENSITY),
               QLineF(regions[p].leftMost * SALIENCY_DENSITY,
                     regions[p].topMost.second * SALIENCY_DENSITY,
                     regions[p].rightMost * SALIENCY_DENSITY,
                     regions[p].topMost.second * SALIENCY_DENSITY),
               QLineF(regions[p].rightMost * SALIENCY_DENSITY,
                     regions[p].topMost.second * SALIENCY_DENSITY,
                     regions[p].rightMost * SALIENCY_DENSITY,
                     regions[p].bottomMost.second * SALIENCY_DENSITY),
               QLineF(regions[p].leftMost * SALIENCY_DENSITY,
                     regions[p].bottomMost.second * SALIENCY_DENSITY,
                     regions[p].rightMost * SALIENCY_DENSITY,
                     regions[p].bottomMost.second * SALIENCY_DENSITY)
            };
            painter.drawLines(lines, 4);
         }
      }
   }
}
#endif
