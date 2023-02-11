#include "variableView.hpp"
#include <QTreeWidgetItem>
#include <QStringList>
#include <QString>
#include <string>
#include <utility>
#include <algorithm>
#include "types/ActionCommand.hpp"
#include "blackboard/Blackboard.hpp"
#include "perception/vision/Vision.hpp"

#include <boost/lexical_cast.hpp>


using namespace std;

VariableView::VariableView() {
   perceptionHeading = new QTreeWidgetItem(this, QStringList(QString("Perception")), 1);
   perceptionHeading->setExpanded(true);
   perceptionAverageFPS  = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Framerate: ")), 1);
   perceptionKinematicsTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Kinematics time: ")), 1);
   perceptionVisionTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Vision time: ")), 1);
   perceptionLocalisationTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Localisation time: ")), 1);
   perceptionBehaviourTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Behaviour time: ")), 1);
   perceptionTotalTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Total time: ")), 1);

   visionHeading = new QTreeWidgetItem(this, QStringList(QString("Vision")), 1);
   visionHeading->setExpanded(true);

   visionTimestamp = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Vision Timestamp = ")), 1);
   visionNumBalls = new QTreeWidgetItem(visionHeading,
         QStringList(QString("# Balls = ")), 1);
   visionBallPos = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Ball Pos = ")), 1);
   visionBallPosRobotRelative  = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Robot relative Ball Pos = ")), 1);

   this->setHeaderLabel(QString("State variables"));

   localisationHeading = new QTreeWidgetItem(this,
         QStringList(QString("State Estimation")), 1);
   localisationHeading->setExpanded(true);
   localisationRobotPosx = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot x = ")), 1);
   localisationRobotPosy = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot y = ")), 1);
   localisationRobotHeading = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot heading = ")), 1);
   localisationRobotTopWeight = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot top weight = ")), 1);
   localisationRobotHypoth = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot num hypoth = ")), 1);
   localisationBallx = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball x = ")), 1);
   localisationBally = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball y = ")), 1);
    localisationBallVelx = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball dx = ")), 1);
   localisationBallVely = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball dy = ")), 1);
   localisationTeamBallx = new QTreeWidgetItem(localisationHeading, QStringList(QString("Team Ball x = ")), 1);
   localisationTeamBally = new QTreeWidgetItem(localisationHeading, QStringList(QString("Team Ball y = ")), 1);
   localisationTeamBallCovariance = new QTreeWidgetItem(localisationHeading, QStringList(QString("Team Ball covariance = ")), 1);
   localisationHavePendingIncomingSharedBundle = new QTreeWidgetItem(localisationHeading,
                                                                     QStringList(QString("hpisb = ")),
                                                                     1);

   motionHeading = new QTreeWidgetItem(this, QStringList(QString("Motion")), 1);
   motionHeading->setExpanded(true);

   motionHeadYaw = new QTreeWidgetItem(motionHeading,
         QStringList(QString("HeadYaw = ")), 1);

   behaviourHeading = new QTreeWidgetItem(this,
                                          QStringList(QString("Behaviour")), 1);
   behaviourBodyRequest = new QTreeWidgetItem(behaviourHeading,
                     QStringList(QString("request.body.actionType = ")), 1);
   bodyBehaviourHierarchy = new QTreeWidgetItem(behaviourHeading,
                     QStringList(QString("Body Behaviour Hierarchy:")), 1);
   headBehaviourHierarchy = new QTreeWidgetItem(behaviourHeading,
                     QStringList(QString("Head Behaviour Hierarchy:")), 1);
   behaviourHeading->setExpanded(true);

   gameControllerHeading = new QTreeWidgetItem(this,
                                          QStringList(QString("GameController")), 1);
   gameControllerPlayerNumber = new QTreeWidgetItem(gameControllerHeading,
                     QStringList(QString("gameController.player_number = ")), 1);
   gameControllerHeading->setExpanded(true);

   receiverHeading = new QTreeWidgetItem(this,
                                          QStringList(QString("Receiver")), 1);
   receiverLastReceived = new QTreeWidgetItem(receiverHeading,
                     QStringList(QString("lastReceived = ")), 1);
   receiverDataSharedStateEstimationBundleRobotPos = new QTreeWidgetItem(receiverHeading,
                     QStringList(QString("data sseb robotPos = ")), 1);
   receiverDataSharedStateEstimationBundleBallPosRRC = new QTreeWidgetItem(receiverHeading,
                     QStringList(QString("data sseb ballPosRRC = ")), 1);
   receiverHeading->setExpanded(true);

   this->addTopLevelItem(visionHeading);
   this->addTopLevelItem(localisationHeading);
   this->addTopLevelItem(motionHeading);
   this->addTopLevelItem(behaviourHeading);
//    this->addTopLevelItem(gameControllerHeading);
}

static void setTextAndReset(QTreeWidgetItem *item, ostream &ostream) {
   stringstream &sstream = (stringstream&)ostream;
   item->setText(0, sstream.str().c_str());
   sstream.str("");
}

void VariableView::redraw(NaoData *naoData) {

   Frame frame = naoData->getCurrentFrame();
   Blackboard *blackboard = (frame.blackboard);
   if(!blackboard) return;

   // struct RobotPos wrapperPos[4];
   // readArray(localisation, robot, wrapperPos);
   AbsCoord pos = readFrom(stateEstimation, robotPos);

   AbsCoord ballPos = readFrom(stateEstimation, ballPos);
   AbsCoord ballVelRRC = readFrom(stateEstimation, ballVelRRC);
   AbsCoord teamBallPos = readFrom(stateEstimation, teamBallPos);
   std::vector<AbsCoord> allRobotPos = readFrom(stateEstimation, allRobotPos);
   std::vector<bool> havePendingIncomingSharedBundle = readFrom(stateEstimation, havePendingIncomingSharedBundle);

   stringstream sstream;
   ::setTextAndReset(localisationRobotPosx,
                     sstream << "Robot x: " << pos.x() << " stdvar: (" << std::sqrt(pos.var(0, 0)) << ")");
   ::setTextAndReset(localisationRobotPosy,
                     sstream << "Robot y: " << pos.y() << " stdvar: (" << std::sqrt(pos.var(1, 1)) << ")");
   ::setTextAndReset(localisationRobotHeading,
                     sstream << "Robot heading: " << pos.theta() << " (" << RAD2DEG(pos.theta()) << " deg) stdvar: ("
                             << std::sqrt(pos.var(2, 2)));
   ::setTextAndReset(localisationRobotTopWeight,
                     sstream << "Number of hypotheses: " << allRobotPos.size());
   ::setTextAndReset(localisationRobotHypoth,
                     sstream << "Weight of top Gaussian: " << pos.weight);
   ::setTextAndReset(localisationBallx,
                     sstream << "Ball x: " << ballPos.x() << " stdvar: (" << std::sqrt(ballPos.var(0, 0)) << ")");
   ::setTextAndReset(localisationBally,
                     sstream << "Ball y: " << ballPos.y() << " stdvar: (" << std::sqrt(ballPos.var(1, 1)) << ")");
   ::setTextAndReset(localisationBallVelx,
                     sstream << "Ball vel x: " << ballVelRRC.x() << " stdvar: (" << std::sqrt(ballVelRRC.var(0, 0))
                             << ")");
   ::setTextAndReset(localisationBallVely,
                     sstream << "Ball vel y: " << ballVelRRC.y() << " stdvar: (" << std::sqrt(ballVelRRC.var(1, 1))
                             << ")");
   ::setTextAndReset(localisationTeamBallx,
                     sstream << "Team Ball x: " << teamBallPos.x() << " stdvar: (" << std::sqrt(teamBallPos.var(0, 0))
                             << ")");
   ::setTextAndReset(localisationTeamBally,
                     sstream << "Team Ball y: " << teamBallPos.y() << " stdvar: (" << std::sqrt(teamBallPos.var(1, 1))
                             << ")");
   ::setTextAndReset(localisationTeamBallCovariance,
                     sstream << "Team Ball covariance:\n" << teamBallPos.var);
   sstream << "hpisb = ";
   std::copy(havePendingIncomingSharedBundle.begin(),
             havePendingIncomingSharedBundle.end(),
             std::ostream_iterator<bool>(sstream, ","));
   ::setTextAndReset(localisationHavePendingIncomingSharedBundle, sstream);

   updateVision(naoData);
   updateBehaviour(naoData);
   ::setTextAndReset(gameControllerPlayerNumber, sstream << "gamecontroller.player_number = " << readFrom(gameController, player_number));

   {
      const BroadcastData *data = readFrom(receiver, data);
      const time_t *lastReceived = readFrom(receiver, lastReceived);

      sstream << "lastReceived = ";
      for (int lr = 0; lr < ROBOTS_PER_TEAM; ++lr) {
         sstream << endl << lr + 1 << ": " << lastReceived[lr];
      }
      ::setTextAndReset(receiverLastReceived, sstream);

      sstream << "data sseb robotPos = ";
      for (int d = 0; d < ROBOTS_PER_TEAM; ++d) {
         const AbsCoord &robotPos = data[d].sharedStateEstimationBundle.robotPos;
         sstream << endl << d + 1 << ": (" << robotPos.x() << ", " << robotPos.y() << ", " << robotPos.theta() << ")";
      }
      ::setTextAndReset(receiverDataSharedStateEstimationBundleRobotPos, sstream);

      sstream << "data sseb ballPosRRC = ";
      for (int d = 0; d < ROBOTS_PER_TEAM; ++d) {
         const AbsCoord &ballPosRRC = data[d].sharedStateEstimationBundle.ballPosRRC;
         sstream << endl << d + 1 << ": (" << ballPosRRC.x() << ", " << ballPosRRC.y() << ")";
      }
      ::setTextAndReset(receiverDataSharedStateEstimationBundleBallPosRRC, sstream);
   }
}

template <class T>
const QString VariableView::createSufPref(string pref, T t, string suff) {
   stringstream s;
   s << pref << t << suff;
   return s.str().c_str();
}

void VariableView::updateBehaviour(NaoData *naoData) {
   Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
   if (!blackboard) return;

   string actionName;
   switch (readFrom(motion, active).body.actionType) {
   case ActionCommand::Body::NONE: actionName = "NONE"; break;
   case ActionCommand::Body::STAND: actionName = "STAND"; break;
   case ActionCommand::Body::WALK: actionName = "WALK"; break;
   case ActionCommand::Body::TURN_DRIBBLE: actionName = "TURN_DRIBBLE"; break;
   case ActionCommand::Body::GETUP_FRONT: actionName = "GETUP_FRONT"; break;
   case ActionCommand::Body::GETUP_BACK: actionName = "GETUP_BACK"; break;
   case ActionCommand::Body::TIP_OVER: actionName = "TIP_OVER"; break;
   case ActionCommand::Body::KICK: actionName = "KICK"; break;
   case ActionCommand::Body::INITIAL: actionName = "INITIAL"; break;
   case ActionCommand::Body::LIMP: actionName = "LIMP"; break;
   case ActionCommand::Body::REF_PICKUP: actionName = "REF_PICKUP"; break;
   case ActionCommand::Body::GOALIE_SIT: actionName = "GOALIE_SIT"; break;
   case ActionCommand::Body::GOALIE_DIVE_RIGHT: actionName = "GOALIE_DIVE_RIGHT"; break;
   case ActionCommand::Body::GOALIE_DIVE_LEFT: actionName = "GOALIE_DIVE_LEFT"; break;
   case ActionCommand::Body::GOALIE_CENTRE: actionName = "GOALIE_CENTRE"; break;
   case ActionCommand::Body::GOALIE_UNCENTRE: actionName = "GOALIE_UNCENTRE"; break;
   case ActionCommand::Body::GOALIE_INITIAL: actionName = "GOALIE_INITIAL"; break;
   case ActionCommand::Body::GOALIE_AFTERSIT_INITIAL: actionName = "GOALIE_AFTERSIT_INITIAL"; break;

   default: actionName = "Other"; break;
   }
   behaviourBodyRequest->setText(0, createSufPref("request.body.actionType = ",
                                                  actionName, ""));

   string strBodyBehaviourHierarchy = readFrom(behaviour,request)[0].behaviourDebugInfo.bodyBehaviourHierarchy;
   string strHeadBehaviourHierarchy = readFrom(behaviour,request)[0].behaviourDebugInfo.headBehaviourHierarchy;
   std::replace(strBodyBehaviourHierarchy.begin(), strBodyBehaviourHierarchy.end(), '.', '\n');
   std::replace(strHeadBehaviourHierarchy.begin(), strHeadBehaviourHierarchy.end(), '.', '\n');
   bodyBehaviourHierarchy->setText(0, createSufPref("Body Behaviour Hierarchy: \n",
                                                  strBodyBehaviourHierarchy, ""));
   headBehaviourHierarchy->setText(0, createSufPref("Head Behaviour Hierarchy: \n",
                                                  strHeadBehaviourHierarchy, ""));
}

void VariableView::updateVision(NaoData *naoData) {

   Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
   if (!blackboard) return;

   std::vector<BallInfo>         balls         = readFrom (vision, balls);
   std::vector<RobotVisionInfo>        robots        = readFrom (vision, robots);
   std::vector<FieldBoundaryInfo>    fieldBoundaries    = readFrom (vision, fieldBoundaries);
   std::vector<FieldFeatureInfo> fieldFeatures = readFrom (vision, fieldFeatures);

   float headYaw  = readFrom(motion, sensors).joints.angles[Joints::HeadYaw];
   uint32_t total = readFrom(perception, total);

   times.push_back(total);
   if (times.size() > 10) times.pop_front();
   int sum = 0;
   for (unsigned int i = 0; i < times.size(); i++) sum += times[i];
   float fps = 1000000.0/total;


   perceptionAverageFPS->setText(0, createSufPref("Framerate: ", fps, " fps"));
   perceptionKinematicsTime->setText(0, createSufPref("Kinematics time: ", readFrom(perception, kinematics), ""));
   perceptionVisionTime->setText(0, createSufPref("Vision time: ", readFrom(perception, vision), ""));
   perceptionLocalisationTime->setText(0, createSufPref("StateEstimation time: ", readFrom(perception, stateEstimation), ""));
   perceptionBehaviourTime->setText(0, createSufPref("Behaviour time: ", readFrom(perception, behaviour), ""));
   perceptionTotalTime->setText(0, createSufPref("Total time: ", total, ""));

   visionTimestamp->setText(0, createSufPref("Timestamp: ", readFrom(vision, timestamp), ""));


   stringstream sBallPos;

   int numBalls = balls.size ();
   if (numBalls == 0) {
   } else {
      Point ballLocation = balls[0].imageCoords;
      sBallPos << "Ball is at (" << ballLocation.x() <<
         ", " << ballLocation.y () << ")";
   }
   visionBallPos->setText(0, sBallPos.str().c_str());

   stringstream s;

   int numBoundaries = fieldBoundaries.size ();
   RANSACLine boundaries[MAX_FIELD_BOUNDARIES];

   for (int i = 0; i < numBoundaries; ++ i) {
      boundaries[i] = fieldBoundaries[i].rrBoundary;
   }


   s << "numBoundaries " << numBoundaries << endl;
   for (int i = 0; i < numBoundaries; i++) {
      s << "Edge 1: " << boundaries[i].t1 << "x + " << boundaries[i].t2 <<
         "y + " << boundaries[i].t3 << endl;
   }

   stringstream sBallPosRelative;
   if(numBalls > 0) {
      sBallPosRelative << "Ball @ " << balls[0].rr.distance() << ", " <<
         RAD2DEG(balls[0].rr.heading()) << ", " << balls[0].rr.heading();
   }
   visionBallPosRobotRelative->setText(0, sBallPosRelative.str().c_str());

   this->visionNumBalls->setText(0,createSufPref("# Balls: ", numBalls, ""));

   stringstream head;
   head.str("");
   head << "HeadYaw = " << headYaw;
   motionHeadYaw->setText (0, head.str ().c_str ());

}
