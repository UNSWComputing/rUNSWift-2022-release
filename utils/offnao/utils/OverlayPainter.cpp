#include "OverlayPainter.hpp"

#include "perception/vision/VisionDefinitions.hpp"
#include "utils/SPLDefs.hpp"

#include "types/BallInfo.hpp"
#include "types/RobotVisionInfo.hpp"
#include "types/FieldBoundaryInfo.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "perception/vision/Region/Region.hpp"


static QPoint p2q (Point p) { return QPoint (p.x (), p.y ()); }

OverlayPainter::OverlayPainter () : QPainter ()
{
}

OverlayPainter::OverlayPainter (QPaintDevice *device) : QPainter (device)
{
}

OverlayPainter::~OverlayPainter ()
{
}

void OverlayPainter::setPen(const QPen &pen)
{
   QPen newPen(pen);

   int width = pen.width();

   double zoom  = sqrt(transform().determinant());

   newPen.setWidth(width / zoom);

   QPainter::setPen(newPen);
}

void OverlayPainter::drawHorizon(std::pair<int, int> horizon)
{
   save();

   QPoint p1(0         , horizon.first );
   QPoint p2(TOP_IMAGE_COLS, horizon.second);

   setPen(QPen(QBrush("pink"), 5));
   drawLine(p1, p2);

   restore();
}


void OverlayPainter::drawBallOverlay (const BallInfo &ball)
{
   save ();

   QPoint centre = p2q (ball.imageCoords);
   int    radius = ball.radius;

   setPen (QPen(QColor ("black"), 5));
   drawEllipse (centre, radius, radius);

   restore ();
}

void OverlayPainter::drawRobotBox(const BBox &region, const QBrush &colour) {
   const static int PEN_WIDTH = 2;
   this->save();
   QPoint a = p2q (region.a);
   QPoint b = p2q (region.b);

   this->setPen(QPen(colour, PEN_WIDTH));

   this->drawRect(QRect (a,b));

   this->drawLine(a, b);
   int tmp = a.x();
   a.setX(b.x());
   b.setX(tmp);
   this->drawLine(a, b);

   this->restore();
}

void OverlayPainter::drawRobotOverlay (const RobotVisionInfo &robot)
{
    QBrush colour;
    if(robot.type == RobotVisionInfo::rRed) {
          colour = QBrush("pink");
       } else if(robot.type == RobotVisionInfo::rBlue) {
          colour = QBrush("#7f7fff");
       } else {
          colour = QBrush("#00ff00");
       }

   if (robot.cameras == RobotVisionInfo::BOTH_CAMERAS) {
       drawRobotBox(robot.topImageCoords, colour);
       drawRobotBox(robot.botImageCoords, colour);
   } else if (robot.cameras == RobotVisionInfo::TOP_CAMERA) {
       drawRobotBox(robot.topImageCoords, colour);
   } else if (robot.cameras == RobotVisionInfo::BOT_CAMERA) {
       drawRobotBox(robot.botImageCoords, colour);
   } else if (robot.cameras == RobotVisionInfo::OLD_DETECTION) {
       drawRobotBox(robot.imageCoords, colour);
   }


}

void OverlayPainter::drawLinePath(const QPainterPath &path, QColor colour) {
   this->save();
   this->setPen(colour);
   this->drawPath(path);
   this->restore();
}

void OverlayPainter::drawFieldBoundaryOverlay (const FieldBoundaryInfo &fieldBoundary)
{
   QPoint p1, p2;

   /* a*x + b*y + c = 0 */
   double a = fieldBoundary.imageBoundary.t1;
   double b = fieldBoundary.imageBoundary.t2;
   double c = fieldBoundary.imageBoundary.t3;

   /* Avoid divide by 0 */
   if (a == 0 && b == 0) {
      return;
   }

   /* Line is more vertical */
   if (abs (a) > abs (b)) {
      double x, y;

      y = 0;
      x = (-c - b*y) / a;
      p1 = QPoint (x, y);

      y = TOP_IMAGE_ROWS;
      x = (-c - b*y) / a;
      p2 = QPoint (x, y);
   } else {
      double x, y;

      x = 0;
      y = (-c - a*x) / b;
      p1 = QPoint (x, y);

      x = TOP_IMAGE_COLS;
      y = (-c - a*x) / b;
      p2 = QPoint (x, y);
   }

   save ();
   setPen (QPen(QColor ("brown"), 5));

   drawLine (p1, p2);

   restore ();
}

void OverlayPainter::drawFieldFeatureOverlay (const FieldFeatureInfo &fieldFeature)
{
}

void OverlayPainter::drawFieldFeatureOverlay (const FieldFeatureInfo &fieldFeature, QColor colour)
{
//    if (fieldFeature.type == FieldFeatureInfo::fLine)
//    {
//       QPoint p1, p2;

//       /* a*x + b*y + c = 0 */
//       double a = fieldFeature.line.t1;
//       double b = fieldFeature.line.t2;
//       double c = fieldFeature.line.t3;

//       /* Avoid divide by 0 */
//       if (a == 0 && b == 0) {
//          return;
//       }

//       /* Line is more vertical */
//       if (abs (a) > abs (b)) {
//          double x, y;

//          y = 0;
//          x = (-c - b*y) / a;
//          p1 = QPoint (x, y);

//          y = BOT_IMAGE_ROWS;
//          x = (-c - b*y) / a;
//          p2 = QPoint (x, y);
//       } else {
//          double x, y;

//          x = 0;
//          y = (-c - a*x) / b;
//          p1 = QPoint (x, y);

//          x = BOT_IMAGE_COLS;
//          y = (-c - a*x) / b;
//          p2 = QPoint (x, y);
//       }

//       save ();
//       setPen (QColor (colour));

//       drawLine (p1, p2);

//       restore ();
//    }

//    if (fieldFeature.type == FieldFeatureInfo::fCorner) {
//       drawPoint (fieldFeature.corner.p, "black");
//    }

//    if (fieldFeature.type == FieldFeatureInfo::fTJunction) {
//    }

//    if (fieldFeature.type == FieldFeatureInfo::fPenaltySpot) {
//       int w = fieldFeature.penaltyspot.w;
//       int h = fieldFeature.penaltyspot.h;
//       Point centre = fieldFeature.penaltyspot.p;

//       Point tl(centre.x() - w/2, centre.y() - h/2);

//       drawPoint(centre, "green");

//       drawRect(tl.x(), tl.y(), w, h);

//    }

}

/*
Draws a region's bounding box on the image.
*/
void OverlayPainter::drawRegionOverlay (const RegionI &region)
{
    QBrush colour;
    colour = QBrush("#0000FF");
    drawRegionBox(region, colour);
}

void OverlayPainter::drawRegionBox(const RegionI &region, const QBrush &colour) {
   const static int PEN_WIDTH = 2;
   BBox bbox = region.getBoundingBoxRaw();
   this->save();
   this->setPen(QPen(colour, PEN_WIDTH));

   this->drawRect(bbox.a[0], bbox.a[1] + (!region.isTopCamera())*TOP_IMAGE_ROWS,
                                  bbox.b[0]-bbox.a[0]+1, bbox.b[1]-bbox.a[1]+1);

   this->restore();
}

void OverlayPainter::drawPolygon(QPolygon poly, QBrush fill){
	save();
	setBrush (fill);
	QPainter::drawPolygon(poly);
	restore();
}

void OverlayPainter::drawRectangle(Point &a, Point &b){
	save();
	drawRect(a.x(), a.y(), b.x() - a.x(), b.y() - a.y());
	restore();
}

void OverlayPainter::drawPoint (Point &p, QColor colour)
{
   QRect rect (p.x () - 1, p.y () - 1, 3, 3);

   save ();

   setPen ("black");
   setBrush (QBrush (colour, Qt::SolidPattern));
   drawEllipse (rect);

   restore ();
}
