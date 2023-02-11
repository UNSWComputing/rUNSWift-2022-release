#include "FieldPainter.hpp"

#include "types/BallInfo.hpp"
#include "types/RobotVisionInfo.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/AbsCoord.hpp"
#include "utils/angles.hpp"
#include "utils/SPLDefs.hpp"

FieldPainter::FieldPainter (QPaintDevice *device) : QPainter (device)
{
   // determine the size of the bitmap
   int device_width = device->width();
   int device_height = device->height();
   // translate to center of field
   translate(device_width/2, device_height/2);
   // make positive y point upwards
   scale(1,-1);
   // scale so we can draw using field coordinates
   scale((float)device_width/OFFNAO_FULL_FIELD_LENGTH,
         (float)device_height/OFFNAO_FULL_FIELD_WIDTH);
}

FieldPainter::~FieldPainter ()
{
}

void FieldPainter::drawField() {

   QBrush no_brush(Qt::NoBrush);
   QBrush goal_brush(Qt::yellow);

   QPen no_pen(Qt::NoPen);

   QPen field_line_pen(QColor(255, 255, 255));
   field_line_pen.setWidth(FIELD_LINE_WIDTH);
   field_line_pen.setJoinStyle(Qt::MiterJoin);
   field_line_pen.setMiterLimit(0.5);

   QPen marker_pen(QColor(255, 255, 255));
   marker_pen.setWidth(FIELD_LINE_WIDTH);
   marker_pen.setJoinStyle(Qt::MiterJoin);
   marker_pen.setCapStyle(Qt::FlatCap);
   marker_pen.setMiterLimit(0);

   QPen crossbar_pen(Qt::yellow);
   crossbar_pen.setWidth(GOAL_POST_DIAMETER);
   marker_pen.setCapStyle(Qt::FlatCap);
   crossbar_pen.setMiterLimit(0);

   QPen goal_supports_pen(Qt::gray);
   goal_supports_pen.setWidth(GOAL_SUPPORT_DIAMETER);

   save();

   setRenderHint(QPainter::Antialiasing);

   setBrush(no_brush);
   setPen(field_line_pen);

   // Outline
   drawRect(-FIELD_LENGTH/2, -FIELD_WIDTH/2, FIELD_LENGTH, FIELD_WIDTH);
   // Center line
   drawLine(0, FIELD_WIDTH/2, 0, -FIELD_WIDTH/2);
   // Goal boxes
   drawRect(-FIELD_LENGTH/2, -GOAL_BOX_WIDTH/2, GOAL_BOX_LENGTH, GOAL_BOX_WIDTH);
   drawRect(FIELD_LENGTH/2 - GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH/2, GOAL_BOX_LENGTH, GOAL_BOX_WIDTH);
   //Penalty area boxes
   drawRect(-FIELD_LENGTH/2, -PENALTY_AREA_WIDTH/2, PENALTY_AREA_LENGTH, PENALTY_AREA_WIDTH);
   drawRect(FIELD_LENGTH/2 -PENALTY_AREA_LENGTH, - PENALTY_AREA_WIDTH/2, PENALTY_AREA_LENGTH, PENALTY_AREA_WIDTH);
   //Center circle
   drawEllipse(QPoint(0, 0), CENTER_CIRCLE_DIAMETER/2, CENTER_CIRCLE_DIAMETER/2);

   setBrush(no_brush);
   setPen(marker_pen);

   drawLine(-PENALTY_CROSS_DIMENSIONS/2, 0, PENALTY_CROSS_DIMENSIONS/2, 0);

   int penalty_x = FIELD_LENGTH/2 - DIST_GOAL_LINE_TO_PENALTY_CROSS;

   drawLine(penalty_x - PENALTY_CROSS_DIMENSIONS/2, 0, penalty_x + PENALTY_CROSS_DIMENSIONS/2, 0);
   drawLine(penalty_x, -PENALTY_CROSS_DIMENSIONS/2, penalty_x, PENALTY_CROSS_DIMENSIONS/2);

   drawLine(-penalty_x - PENALTY_CROSS_DIMENSIONS/2, 0, -penalty_x + PENALTY_CROSS_DIMENSIONS/2, 0);
   drawLine(-penalty_x, -PENALTY_CROSS_DIMENSIONS/2, -penalty_x, PENALTY_CROSS_DIMENSIONS/2);

   setBrush(no_brush);
   setPen(goal_supports_pen);

   int goal_x = FIELD_LENGTH/2 + FIELD_LINE_WIDTH/2;

   drawLine(-goal_x, -GOAL_WIDTH/2, -goal_x - GOAL_DEPTH, -GOAL_WIDTH/2);
   drawLine(-goal_x,  GOAL_WIDTH/2, -goal_x - GOAL_DEPTH,  GOAL_WIDTH/2);
   drawLine( goal_x, -GOAL_WIDTH/2,  goal_x + GOAL_DEPTH, -GOAL_WIDTH/2);
   drawLine( goal_x,  GOAL_WIDTH/2,  goal_x + GOAL_DEPTH,  GOAL_WIDTH/2);
   drawLine( goal_x + GOAL_DEPTH, -GOAL_WIDTH/2,  goal_x + GOAL_DEPTH, GOAL_WIDTH/2);
   drawLine(-goal_x - GOAL_DEPTH, -GOAL_WIDTH/2, -goal_x - GOAL_DEPTH, GOAL_WIDTH/2);

   setBrush(no_brush);
   setPen(crossbar_pen);

   drawLine(-goal_x, -GOAL_WIDTH/2, -goal_x, GOAL_WIDTH/2);
   drawLine( goal_x, -GOAL_WIDTH/2,  goal_x, GOAL_WIDTH/2);

   setBrush(goal_brush);
   setPen(no_pen);

   drawEllipse(QPoint(-goal_x, -GOAL_WIDTH/2), GOAL_POST_DIAMETER/2, GOAL_POST_DIAMETER/2);
   drawEllipse(QPoint(-goal_x,  GOAL_WIDTH/2), GOAL_POST_DIAMETER/2, GOAL_POST_DIAMETER/2);
   drawEllipse(QPoint( goal_x, -GOAL_WIDTH/2), GOAL_POST_DIAMETER/2, GOAL_POST_DIAMETER/2);
   drawEllipse(QPoint( goal_x,  GOAL_WIDTH/2), GOAL_POST_DIAMETER/2, GOAL_POST_DIAMETER/2);

   restore();
}

void FieldPainter::translateRR(const RRCoord &rr, const AbsCoord &robot) {
   translate(robot.x(), robot.y());
   rotate((const float)RAD2DEG(robot.theta()));
   rotate((const float)RAD2DEG(rr.heading()));
   translate(rr.distance(), 0);
   if (!isnan(rr.orientation())) {
      rotate(RAD2DEG(-rr.orientation()));
   }
}

void FieldPainter::drawBallRR (const BallInfo &ball, const AbsCoord &robot, const QColor &col)
{
   save ();
   translateRR(ball.rr, robot);
   setPen (QColor ("black"));
   setBrush(QBrush(col));
   drawEllipse (QPoint(0, 0), 40, 40);
   restore ();
}

void FieldPainter::drawBallRR (const BallInfo &ball, const AbsCoord &robot)
{
   QColor col = QColor(255, 127, 0);
   drawBallRR(ball, robot, col);
}

void FieldPainter::drawFeatureRR (const FieldFeatureInfo &feat, const AbsCoord &robot)
{
   save();
   setPen(Qt::NoPen);
   setBrush(QBrush("red"));
   translateRR(feat.rr, robot);
   QPen p;
   switch (feat.type) {
      case FieldFeatureInfo::fCorner:
         setPen("black");
         p = pen();
         p.setWidth(50);
         setPen(p);
         save();
         rotate(RAD2DEG(M_PI_4));
         drawRect(25, 25, -300, -50);
         restore();
         save();
         rotate(RAD2DEG(-M_PI_4));
         drawRect(25, 25, -300, -50);
         restore();
         break;
      case FieldFeatureInfo::fTJunction:
         setPen("black");
         p = pen();
         p.setWidth(50);
         setPen(p);
         drawRect(-25, -300, 50, 600);
         drawRect(0, 25, -300, -50);
         break;
      case FieldFeatureInfo::fPenaltySpot:
         setPen("black");
         p = pen();
         p.setWidth(50);
         setPen(p);
         setBrush(Qt::NoBrush);
         drawEllipse (QPoint(-30,-30), 60,60);
         break;
      case FieldFeatureInfo::fCentreCircle:
         if (!isnan(feat.rr.orientation())) {
            setPen("orange");
         }
         else {
            setPen("red");
         }
         p = pen();
         p.setWidth(50);
         setPen(p);
         setBrush(Qt::NoBrush);
         drawEllipse (QPoint(0, 0), CENTER_CIRCLE_DIAMETER / 2, CENTER_CIRCLE_DIAMETER / 2);
         break;
      default:
         break;
   }
   restore();

   if (feat.type == FieldFeatureInfo::fLine)
   {
      save();
      translate(robot.x(), robot.y());
      rotate((const float)RAD2DEG(robot.theta()));
      setPen("red");
      p = pen();
      p.setWidth(50);
      setPen(p);
      setBrush(QBrush("red"));
      drawLine(feat.p1.x(), feat.p1.y(),
               feat.p2.x(), feat.p2.y());
      restore();
   }

}

void FieldPainter::drawFeatureAbs (const AbsCoord &pos, const FieldFeatureInfo &ff) {
   save();
   translate(pos.x(), pos.y());
   rotate(RAD2DEG(pos.theta()));
   setPen(Qt::NoPen);
   setBrush(QBrush("red"));
   QPen p;
   switch (ff.type) {
      case FieldFeatureInfo::fCorner:
         save();
         rotate(RAD2DEG(M_PI_4));
         drawRect(-25, -25, 300, 50);
         restore();
         save();
         rotate(RAD2DEG(-M_PI_4));
         drawRect(-25, -25, 300, 50);
         restore();
         break;
      case FieldFeatureInfo::fTJunction:
         drawRect(-25, -300, 50, 600);
         drawRect(0, -25, 300, 50);
      case FieldFeatureInfo::fPenaltySpot:
         break;
      case FieldFeatureInfo::fCentreCircle:
         save();
         setPen("red");
         p = pen();
         p.setWidth(50);
         setPen(p);
         setBrush(Qt::NoBrush);
         drawEllipse (QPoint(0, 0), 600, 600);
         restore();
         break;
      case FieldFeatureInfo::fLine:
         break;
      default:
         break;
   }
   restore();
}

void FieldPainter::drawPlayerNumber (const AbsCoord &pos, int num, QColor color) {
   save();
   translate(pos.x() - 80, pos.y() - 400);
   scale(1,-1);
   setPen(color);
   setBrush(color);
   QFont font;
   font.setPixelSize(250);
   setFont(font);
   drawText(0, 0, QString::number(num));
   restore();
}

void FieldPainter::drawRobotRole (const AbsCoord &pos, QString role, QColor color) {
   save();
   translate(pos.x() - 80, pos.y() + 200);
   scale(1,-1);
   setPen(color);
   setBrush(color);
   QFont font;
   font.setPixelSize(250);
   setFont(font);
   drawText(0, 0, role);
   restore();
}



void FieldPainter::drawRobotAbs (const AbsCoord &pos, QColor colour, bool ellipse, QColor varColour)
{
   static const int robot_radius = 120;
   QRect roboRect(-robot_radius,-robot_radius,robot_radius*2,robot_radius*2);
   QPen variancePen(varColour);
   variancePen.setWidth(20);

   int linewidth = 80 * pos.weight;

   variancePen.setWidth(linewidth);

   // draw pacman
   save();
   translate(pos.x(), pos.y());
   setPen("black");
   int fov;
   if (pos.var(2,2) > 0) {
      fov = MIN(160, RAD2DEG(sqrt(pos.var(2,2))));
   } else {
      fov = 0;
   }
   setBrush(QBrush(colour));
   rotate((RAD2DEG(pos.theta())-(fov/2)));
   drawPie(roboRect, 0, (360-fov)*16);
   restore();

   /* Converting covariance matrix to ellipse representation
    * Eigenvalues of the matrix are the major and minor axis lengths
    * Eigenvectors are the vectors from the center along the axes (should be perpendicular)
    */
   if (ellipse) {
      Eigen::VectorXf eigenvalues = pos.var.block<2, 2>(0,0).selfadjointView<Eigen::Upper>().eigenvalues();
      int r_major_axis = MAX(sqrt(MAX(eigenvalues[0], eigenvalues[1])), robot_radius);
      int r_minor_axis = MAX(sqrt(MIN(eigenvalues[0], eigenvalues[1])), robot_radius);
      float theta = atan2(2*pos.var(0,1), pos.var(0,0)-pos.var(1,1))/2.0;

      // draw position covariance ellipse
      save();
      translate(pos.x(), pos.y());
      rotate(RAD2DEG(theta));
      QRect varRect(-r_major_axis, -r_minor_axis, r_major_axis*2, r_minor_axis*2);
      setPen(variancePen);
      setBrush(QBrush(Qt::NoBrush));
      drawEllipse(varRect);
      restore();
   }
}

void FieldPainter::drawRobotRR (const RobotVisionInfo &robot, const AbsCoord &robotPos)
{
   save ();
   translateRR(robot.rr, robotPos);
   if(robot.type == RobotVisionInfo::rRed) {
      setBrush(QBrush(Qt::red));
   } else if(robot.type == RobotVisionInfo::rBlue) {
      setBrush(QBrush(Qt::blue));
   } else {
      setBrush(QBrush(Qt::green));
   }
   setPen(QColor(255, 127, 0));
   drawEllipse (QPoint(0, 0), 120, 120);
   restore ();
}

void FieldPainter::drawBallPosRRC(const AbsCoord &ballPos, const AbsCoord &ballVel, bool moving, const AbsCoord &robot)
{
   save();
   translate(robot.x(), robot.y());
   rotate(RAD2DEG(robot.theta()));
   QPoint newBall = QPoint(ballPos.x(), ballPos.y());
   QPoint nextBall = QPoint(ballPos.x() + ballVel.x(), ballPos.y() + ballVel.y());
   setPen(QPen(QColor(255, 0, 0)));
   setBrush(QBrush(QColor(255, 0, 0)));
   drawEllipse(newBall, 40, 40);
   if (moving) drawLine(newBall, nextBall);
   restore();

   drawRRCovEllipse(ballPos, robot);
}

// draw robot relative position covariance ellipse
void FieldPainter::drawRRCovEllipse(const AbsCoord &pos, const AbsCoord &robotPos) {
   QPen variancePen("red");
   variancePen.setWidth(20);

   Eigen::VectorXf eigenvalues = pos.var.block<2, 2>(0,0).selfadjointView<Eigen::Upper>().eigenvalues();
   int r_major_axis = sqrt(MAX(eigenvalues[0], eigenvalues[1]));
   int r_minor_axis = sqrt(MIN(eigenvalues[0], eigenvalues[1]));
   float theta = atan2(2*pos.var(0,1), pos.var(0,0)-pos.var(1,1))/2.0;

   save();
   translate(robotPos.x(), robotPos.y());
   rotate(RAD2DEG(robotPos.theta()));
   translate(pos.x(), pos.y());
   rotate(RAD2DEG(theta));
   QRect varRect(-r_major_axis, -r_minor_axis, r_major_axis*2, r_minor_axis*2);
   setPen(variancePen);
   setBrush(QBrush(Qt::NoBrush));
   drawEllipse(varRect);
   restore();
}

void FieldPainter::drawBallPosAbs(const AbsCoord &ball, QColor colour)
{
   save();
   QPoint newBall = QPoint(ball.x(), ball.y());
   QPen absPen(colour);
   setPen(absPen);
   setBrush(QBrush(colour));
   drawEllipse(newBall, 40, 40);
   restore();

   drawAbsCovEllipse(ball, absPen);
}

void FieldPainter::drawBallVelAbs(const AbsCoord &ballPos, const AbsCoord &ballVel, QColor colour)
{
   save();
   const int numSeconds = 1.0; // how long the velocity line will be (in seconds)
   QPoint lineBasePoint = QPoint(ballPos.x(), ballPos.y());
   QPoint lineEndPoint = QPoint(ballPos.x() + ballVel.x() * numSeconds, ballPos.y() + ballVel.y() * numSeconds);
   QPen absPen(colour);
   setPen(absPen);
   drawLine(lineBasePoint, lineEndPoint);
   restore();
}

// draw absolute position covariance ellipse
void FieldPainter::drawAbsCovEllipse(const AbsCoord &pos, QPen variancePen) {
   variancePen.setWidth(20);

   Eigen::VectorXf eigenvalues = pos.var.block<2, 2>(0,0).selfadjointView<Eigen::Upper>().eigenvalues();
   int r_major_axis = sqrt(MAX(eigenvalues[0], eigenvalues[1]));
   int r_minor_axis = sqrt(MIN(eigenvalues[0], eigenvalues[1]));
   float theta = atan2(2*pos.var(0,1), pos.var(0,0)-pos.var(1,1))/2.0;

   save();
   translate(pos.x(), pos.y());
   rotate(RAD2DEG(theta));
   QRect varRect(-r_major_axis, -r_minor_axis, r_major_axis*2, r_minor_axis*2);
   setPen(variancePen);
   setBrush(QBrush(Qt::NoBrush));
   drawEllipse(varRect);
   restore();
}

void FieldPainter::drawFieldLine(const LineInfo &line) {
   save();
   drawLine(line.p1.x(), line.p1.y(), line.p2.x(), line.p2.y());
   restore();
}

void FieldPainter::drawLineAbs(const AbsCoord &from, const AbsCoord &to, QColor q) {
   save();
   setPen (q);
   drawLine(from.x(), from.y(), to.x(), to.y());
   restore();
}

void FieldPainter::drawBallManoeuvre (const AbsCoord &ball, const AbsCoord &target, float headingError, std::string manoeuvreType, bool manoeuvreHard)
{

   float manoeuvreDist = sqrt((target.y() - ball.y()) * (target.y() - ball.y()) + (target.x() - ball.x()) * (target.x() - ball.x()));
   QRect manoeuvreRect;
   if (manoeuvreHard)
      manoeuvreRect = QRect(-FULL_FIELD_LENGTH, -FULL_FIELD_LENGTH, FULL_FIELD_LENGTH * 2, FULL_FIELD_LENGTH * 2);  // Just draw it really big
   else
      manoeuvreRect = QRect(-manoeuvreDist, -manoeuvreDist, manoeuvreDist * 2, manoeuvreDist * 2);


   // draw a pie
   save();
   translate(ball.x(), ball.y());
   setPen("black");
   QColor c;
   if (manoeuvreType == "KICK")
      c = Qt::green;
   else if (manoeuvreType == "DRIBBLE")
      c = Qt::blue;
   else
      c = Qt::black;
   c.setAlphaF(0.2);
   setBrush(QBrush(c));
   float target_heading = atan2(target.y() - ball.y(), target.x() - ball.x());
   rotate(RAD2DEG(target_heading + headingError));
   drawPie(manoeuvreRect, 0, 16 * (RAD2DEG(headingError) * 2));  // refer to why there is a 16* in this line here: https://doc.qt.io/archives/qt-4.8/qpainter.html#drawPie
   restore();
}
