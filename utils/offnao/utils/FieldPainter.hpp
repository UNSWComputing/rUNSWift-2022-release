#ifndef FIELD_PAINTER_HPP
#define FIELD_PAINTER_HPP

#include <QPainter>

class BallInfo;
class AbsCoord;
class RRCoord;
class FieldFeatureInfo;
class RobotVisionInfo;
class LineInfo;


class FieldPainter : private QPainter
{
   public:
      FieldPainter (QPaintDevice * device);
      virtual ~FieldPainter ();

      void drawField(void);

      void drawBallRR (const BallInfo &ball, const AbsCoord &robot, const QColor &col);
      void drawBallRR(const BallInfo &ball, const AbsCoord &robot);
      void drawFeatureRR (const FieldFeatureInfo &feat, const AbsCoord &robot);
      void drawFeatureAbs (const AbsCoord &pos, const FieldFeatureInfo &ff);
      void drawPlayerNumber(const AbsCoord &pos, int num, QColor color);
      void drawRobotRole (const AbsCoord &pos, QString role, QColor color);
      void drawRobotAbs(const AbsCoord &pos, QColor colour, bool drawEllipse=true, QColor varColour="black");
      void drawRobotRR (const RobotVisionInfo &robot, const AbsCoord &robotPos);

      void drawBallPosRRC (const AbsCoord &ballPos, const AbsCoord &ballVel, bool moving, const AbsCoord &robotPos);
      void drawBallPosAbs (const AbsCoord &ball, QColor colour=QColor(0,0,255));
      void drawBallVelAbs (const AbsCoord &ballPos, const AbsCoord &ballVel, QColor colour=QColor(0,0,255));

      void drawRRCovEllipse (const AbsCoord &pos, const AbsCoord &robotPos);
      void drawAbsCovEllipse (const AbsCoord &pos, QPen variancePen);

      void drawFieldLine(const LineInfo &line);
      void drawLineAbs(const AbsCoord &from, const AbsCoord &to, QColor q);

      void drawBallManoeuvre (const AbsCoord &ball, const AbsCoord &target, float headingError, std::string manoeuvreType, bool manoeuvreHard);

   private:
      void translateRR(const RRCoord &rr, const AbsCoord &robot);
};


#endif // FIELD_PAINTER_HPP
