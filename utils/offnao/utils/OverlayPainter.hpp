#ifndef OVERLAY_PAINTER_HPP
#define OVERLAY_PAINTER_HPP

#include <QPainter>
#include "types/Point.hpp"

class BallInfo;
class GoalInfo;
class RobotVisionInfo;
class FieldBoundaryInfo;
class FieldFeatureInfo;
class BBox;
class RegionI;


class OverlayPainter : private QPainter
{
   public:

      using QPainter::begin;
      using QPainter::end;
      using QPainter::scale;
      using QPainter::translate;
      using QPainter::setTransform;
      using QPainter::drawImage;

      using QPainter::setPen;

      void setPen(const QPen & pen);

      OverlayPainter();
      OverlayPainter(QPaintDevice * device);
      virtual ~OverlayPainter();

      void drawHorizon(std::pair<int, int> horizon);

      void drawRobotBox(const BBox &region, const QBrush &colour);
      // Draws a region's bounding box on the image.
      void drawRegionOverlay (const RegionI &region);
      void drawRegionBox(const RegionI &region, const QBrush &colour);

      void drawBallOverlay(const BallInfo &ball);
      void drawRobotOverlay(const RobotVisionInfo &robot);
      void drawFieldBoundaryOverlay(const FieldBoundaryInfo &fieldBoundary);
      void drawFieldFeatureOverlay(const FieldFeatureInfo &fieldFeature);
      void drawFieldFeatureOverlay(const FieldFeatureInfo &fieldFeature,
                                   QColor colour);

      void drawPoint(Point &p, QColor colour);
      void drawRectangle(Point &a, Point &b);

      void drawLinePath(const QPainterPath &path, QColor colour);
      void drawPolygon(QPolygon poly, QBrush fill);
};

#endif // OVERLAY_PAINTER_HPP
