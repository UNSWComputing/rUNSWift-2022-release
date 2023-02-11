#ifndef VARIABLE_VIEW_HPP
#define VARIABLE_VIEW_HPP

#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "naoData.hpp"


class VariableView : public QTreeWidget {
   public:
      VariableView();
      void redraw(NaoData *naoData);

   private:
      template <class T>
         const QString createSufPref(std::string, T, std::string);

      QTreeWidgetItem *behaviourHeading;
      QTreeWidgetItem *behaviourBodyRequest;
      QTreeWidgetItem *bodyBehaviourHierarchy;
      QTreeWidgetItem *headBehaviourHierarchy;

      QTreeWidgetItem *motionHeading;

      QTreeWidgetItem *gameControllerHeading;
      QTreeWidgetItem *gameControllerPlayerNumber;

      QTreeWidgetItem *perceptionHeading;
      QTreeWidgetItem *perceptionAverageFPS;
      QTreeWidgetItem *perceptionKinematicsTime;
      QTreeWidgetItem *perceptionVisionTime;
      QTreeWidgetItem *perceptionLocalisationTime;
      QTreeWidgetItem *perceptionBehaviourTime;
      QTreeWidgetItem *perceptionTotalTime;


      QTreeWidgetItem *visionTimestamp;
      QTreeWidgetItem *visionFrames;
      QTreeWidgetItem *visionHeading;
      QTreeWidgetItem *visionGoal;
      QTreeWidgetItem *visionBallPos;
      QTreeWidgetItem *visionBallPosRobotRelative;
      QTreeWidgetItem *visionFrameRate;
      QTreeWidgetItem *visionNumBalls;

      QTreeWidgetItem *motionHeadYaw;
      QTreeWidgetItem *kinematicsHeadYaw;

      QTreeWidgetItem *localisationHeading;
      QTreeWidgetItem *localisationRobotPosx;
      QTreeWidgetItem *localisationRobotPosy;
      QTreeWidgetItem *localisationRobotHeading;
      QTreeWidgetItem *localisationRobotTopWeight;
      QTreeWidgetItem *localisationRobotHypoth;
      QTreeWidgetItem *localisationBallx;
      QTreeWidgetItem *localisationBally;
      QTreeWidgetItem *localisationBallVelx;
      QTreeWidgetItem *localisationBallVely;
      QTreeWidgetItem *localisationTeamBallx;
      QTreeWidgetItem *localisationTeamBally;
      QTreeWidgetItem *localisationTeamBallCovariance;
      QTreeWidgetItem *localisationHavePendingIncomingSharedBundle;

      QTreeWidgetItem *receiverHeading;
      QTreeWidgetItem *receiverLastReceived;
      QTreeWidgetItem *receiverDataSharedStateEstimationBundleRobotPos;
      QTreeWidgetItem *receiverDataSharedStateEstimationBundleBallPosRRC;

      std::deque<int> times;
      void updateVision(NaoData *naoData);
      void updateBehaviour(NaoData *naoData);

};

#endif // VARIABLE_VIEW_HPP
