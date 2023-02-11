#include "fieldView.hpp"
#include "utils/FieldPainter.hpp"
#include "naoData.hpp"
#include "utils/PositioningDefs.hpp"


using namespace std;

FieldView::FieldView() {
   renderPixmap = new QPixmap(640, 480);

   imagePixmap = QPixmap(640, 480);
   imagePixmap.fill(QColor(0, 128, 0));
   FieldPainter painter(&imagePixmap);
   painter.drawField();

   renderPixmap->fill(Qt::darkGray);
   setPixmap(*renderPixmap);
   setMinimumSize(640, 480);
   setMaximumSize(640, 480);
   setAlignment(Qt::AlignTop);
}

FieldView::~FieldView() {
}

// Used by the team tab
void FieldView::redraw(std::vector<AbsCoord> &robotPos,
                       std::vector<AbsCoord> &ballPos){

   *renderPixmap = imagePixmap;
   FieldPainter painter(renderPixmap);

   if(robotPos.size() > 0){
     for (unsigned int i = 0; i < robotPos.size(); ++i) {
        if (!isnan(robotPos[i].theta()) &&
            !isnan(robotPos[i].x()) &&
            !isnan(robotPos[i].y())) {
           painter.drawRobotAbs(robotPos[i], "#ffee00", true, "black");
           painter.drawPlayerNumber(robotPos[i], i+1, "white");
        }
     }
   }

   if (ballPos.size() > 0){
     for (unsigned int i = 0; i < ballPos.size(); ++i) {
        if (!isnan(ballPos[i].x()) &&
            !isnan(ballPos[i].y())) {
           painter.drawBallPosAbs(ballPos[i]);
        }
     }
   }
   setPixmap (*renderPixmap);

}

void FieldView::redraw(std::vector<AbsCoord> &robotPos){

  *renderPixmap = imagePixmap;
   if(robotPos.size() > 0){
     FieldPainter painter(renderPixmap);
     for (unsigned int i = 1; i < robotPos.size(); ++i) {
        if (!isnan(robotPos[i].theta()) && !isnan(robotPos[i].x()) && !isnan(robotPos[i].y())) {
//           painter.drawRobotAbs(robotPos[i], "blue", true);
           painter.drawRobotAbs(robotPos[i], "black", true);
        }
     }

     if (!isnan(robotPos[0].theta()) && !isnan(robotPos[0].x()) && !isnan(robotPos[0].y())) {
        painter.drawRobotAbs(robotPos[0], "#ffee00", true); // pacman yellow

     }
   }
   setPixmap (*renderPixmap);

}


void FieldView::redraw(NaoData *naoData) {
   *renderPixmap = imagePixmap;
   Blackboard *blackboard;

   if (! (naoData && (blackboard = naoData->getCurrentFrame ().blackboard))) {
      setPixmap (*renderPixmap);
      return;
   }

   FieldPainter painter(renderPixmap);

   AbsCoord tbPos = readFrom(stateEstimation, teamBallPos);
   AbsCoord tbVel = readFrom(stateEstimation, teamBallVel);
   painter.drawBallPosAbs(tbPos, QColor("black"));
   painter.drawBallVelAbs(tbPos, tbVel, QColor("black"));

   std::vector<AbsCoord> allRobotPos = readFrom(stateEstimation, allRobotPos);

   for (std::vector<AbsCoord>::iterator it = allRobotPos.begin(); it != allRobotPos.end(); ++it) {
      painter.drawRobotAbs(*it, "#ffee00", true, "black");
   }

   /* Draw features with absolute coordinates */
   const AbsCoord robotPos = readFrom(stateEstimation, robotPos);
   if (!isnan(robotPos.theta()) && !isnan(robotPos.x()) && !isnan(robotPos.y())) {
      painter.drawRobotAbs(robotPos, "#ffee00", true, "white"); // pacman yellow, white variance

      /* Draw filtered absolute ball position */
      AbsCoord bPos = readFrom(stateEstimation, ballPos);
      AbsCoord bVel = readFrom(stateEstimation, ballVel);
      if (!isnan(bPos.x()) && !isnan(bPos.y())){
         painter.drawBallPosAbs(bPos, QColor("yellow"));
         painter.drawBallVelAbs(bPos, bVel, QColor("yellow"));
      }
   }

   /* Draw received ball observations, team mates, roles and player numbers */
   int playerNumber = readFrom(gameController, player_number);
   BroadcastData teamData[ROBOTS_PER_TEAM];
   readArray(receiver, data, teamData);
   std::vector<bool> incapacitated = readFrom(receiver, incapacitated);
   for (int playerIndex = 0; playerIndex < ROBOTS_PER_TEAM; playerIndex++) {
      if (!incapacitated[playerIndex]){

         QColor color = QColor("black"); // default to black
         QString role;

         if (teamData[playerIndex].behaviourSharedData.playingBall){
            // If a robot is playing the ball, color it red (same color as leftEye of a robot playing ball)
            color = QColor("red");
         } else if (teamData[playerIndex].behaviourSharedData.isAssisting){
            // If a robot is assisting, color it blue (same color as leftEye of a robot assisting)
            color = QColor("blue");
         } else {
            // If a robot is not actively, involved in play, mention there position.
            // These roles really shouldn't be defined here, but be read from positioning.
            // If you change the positioning of the robots, you will have to change the code
            // here too, to see the correct color and roles showing up in offnao
            switch(teamData[playerIndex].behaviourSharedData.role){
                case POSITIONING_NONE: color = QColor("black"); role="NA"; break;
                case POSITIONING_AGAINST_KICKING_TEAM_SUPPORTER: color = QColor("magenta"); role="F"; break;
                case POSITIONING_AGAINST_KICKING_TEAM_DEFENDER: color = QColor("black"); role="D"; break;
                case POSITIONING_AGAINST_KICKING_TEAM_UPFIELDER: color = QColor("cyan"); role="U"; break;
                case POSITIONING_FIND_BALL_FINDER: color = QColor("gray"); role="FB"; break;

                case POSITIONING_AGAINST_DRIBBLE_TEAM_RIGHT_SUPPORTER: color = QColor("white"); role = "RS"; break;
                case POSITIONING_AGAINST_DRIBBLE_TEAM_SHOOTER: color = QColor("cyan"); role = "SH"; break;
                case POSITIONING_AGAINST_DRIBBLE_TEAM_LEFT_SUPPORTER: color = QColor("magenta"); role = "LS"; break;
                case POSITIONING_AGAINST_DRIBBLE_TEAM_SWEEPER: color = QColor("black"); role = "SW"; break;
                default: color = QColor("black"); role="NA"; break;
            }
         }

         // If not me, then draw ball position
         if (playerIndex != playerNumber - 1 &&
            !isnan(teamData[playerIndex].ballPosAbs.x()))
         {
            painter.drawBallPosAbs(teamData[playerIndex].ballPosAbs, color);
         }

         // Draw robot position, role and player number
         AbsCoord pos(teamData[playerIndex].robotPos[0],teamData[playerIndex].robotPos[1], teamData[playerIndex].robotPos[2]);
         painter.drawRobotAbs(pos, color, true, "black");
         painter.drawRobotRole(pos, role, color);
         painter.drawPlayerNumber(pos, playerIndex+1, color);
         AbsCoord walkingTo(teamData[playerIndex].behaviourSharedData.walkingToX, teamData[playerIndex].behaviourSharedData.walkingToY, teamData[playerIndex].behaviourSharedData.walkingToH);
         // bit hacky, but if walkingTo is all zeros, it's probably not set in behaviours, so don't draw it
         if (playerIndex != playerNumber - 1 &&
             !(walkingTo.x() == 0 && walkingTo.y() == 0 && walkingTo.theta() == 0))
         {
            // draw where a robot wants to walk to, but with a lower opacity
            color.setAlpha(0x80);
            painter.drawRobotAbs(walkingTo, color);
         }
      }
   }

   AbsCoord pos = robotPos;
   bool drawRR = true;

   /* Draw features with robot-relative coordinates */
   if (drawRR) {

      /* Balls */
      std::vector<BallInfo> balls = readFrom(vision, balls);
      std::vector<BallInfo>::iterator ball_i;
      for (ball_i = balls.begin (); ball_i != balls.end (); ++ball_i) {
         painter.drawBallRR(*ball_i, pos);
      }

      /* Robots */
//       const std::vector<RobotVisionInfo>& robots = readFrom(vision, robots);
//       for(unsigned int robot_i = 0; robot_i < robots.size (); ++ robot_i) {
//          painter.drawRobotRR(robots[robot_i], pos);
//       }

      /* Filtered Robots */
      vector<RobotObstacle> robotObstacles = readFrom(stateEstimation, robotObstacles);
      vector<RobotObstacle>::iterator robotobs_i;
      for(robotobs_i = robotObstacles.begin(); robotobs_i != robotObstacles.end(); ++ robotobs_i) {
         float absX   = robotobs_i->rr.distance() * cos(robotobs_i->rr.heading() + pos.theta());
         float absY   = robotobs_i->rr.distance() * sin(robotobs_i->rr.heading() + pos.theta());
         absX        += pos.x();
         absY        += pos.y();
         AbsCoord absRobot(absX, absY, 0);
         Eigen::Matrix<float, 2, 2> rotation = Eigen::Rotation2D<float>(pos.theta()).toRotationMatrix();
         absRobot.var.block(0,0,2,2) = rotation * robotobs_i->rr.var.block(0,0,2,2) * rotation.inverse();
         absRobot.var(2,2) = 0.f;

         if(robotobs_i->type == RobotVisionInfo::rRed) {
            painter.drawRobotAbs(absRobot, "pink");
         } else if(robotobs_i->type == RobotVisionInfo::rBlue) {
            painter.drawRobotAbs(absRobot, "#7f7fff");
         } else {
            painter.drawRobotAbs(absRobot, "#00ff00");
         }
      }

      /* Field Lines RR */
      std::vector<FieldFeatureInfo> features = readFrom(vision,fieldFeatures);
      std::vector<FieldFeatureInfo>::iterator feature_i;
      for (feature_i = features.begin (); feature_i < features.end ();
            ++ feature_i) {
         painter.drawFeatureRR(*feature_i, pos);
      }
   }

   // Draw behaviour debug information
   int behaviourReadBuf = readFrom(behaviour, readBuf);
   BehaviourDebugInfo bdi = readFrom(behaviour, request[behaviourReadBuf]).behaviourDebugInfo;
   if (bdi.haveBallManoeuvreTarget)
   {
      AbsCoord bPos = readFrom(stateEstimation, ballPos);
      painter.drawBallManoeuvre(bPos, AbsCoord(bdi.ballManoeuvreTargetX, bdi.ballManoeuvreTargetY, 0), bdi.ballManoeuvreHeadingError, bdi.ballManoeuvreType, bdi.ballManoeuvreHard);
      painter.drawBallPosAbs(AbsCoord(bdi.ballManoeuvreTargetX, bdi.ballManoeuvreTargetY, 0), "pink");
   }

   if (bdi.anticipating)
   {
      // Draw where we want to anticipate, with low opacity black (like a shadow)
      painter.drawRobotAbs(
          AbsCoord(bdi.anticipateX, bdi.anticipateY, bdi.anticipateH),
          QColor(0x00, 0x00, 0x00, 0x80), true, QColor(0x00, 0x00, 0x00, 0x80));
   }

   setPixmap (*renderPixmap);
   return;
}
