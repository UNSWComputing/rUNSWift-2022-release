#ifndef CAMERA_POSE_TAB_HPP
#define CAMERA_POSE_TAB_HPP

#include <QMenuBar>
#include <QWidget>
#include <QObject>
#include <QEvent>
#include <QLineEdit>
#include <QTextEdit>
#include <QCheckBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QLabel>
#include <QCheckBox>
#include <QTabWidget>
#include <cstdio>
#include <vector>
#include <string>
#include "tabs/tab.hpp"

#include "perception/kinematics/Kinematics.hpp"
#include "perception/vision/VisionDefinitions.hpp"

class Blackboard;


/*
 * This contains the camera pose/mount debuging tab.
 */
class CameraPoseTab : public Tab {
   Q_OBJECT
   public:
      CameraPoseTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);

   private:
      void init();
      void initMenu(QMenuBar *menuBar);

      QGridLayout *layout;
      QGridLayout *optionsLayout;

      QImage lastRendering;

      QPixmap imagePixmap;
      QLabel *camLabel;
      QTextEdit *calibrationOutputBox;
      QLineEdit *offsetYawTopLabel;
      QLineEdit *offsetPitchTopLabel;
      QLineEdit *offsetRollTopLabel;

      QLineEdit *offsetYawBottomLabel;
      QLineEdit *offsetPitchBottomLabel;
      QLineEdit *offsetRollBottomLabel;

      QLineEdit *offsetBodyPitchLabel;

      QRadioButton *whichCamera2;
      QRadioButton *whichCamera3;
      QCheckBox *rawImage;


      float currentOffsetX;
      float currentOffsetY;
      float currentBodyPitchOffset;


      std::string createCommandString(std::string argument, std::string message);
      /* Re-draw the image box from current frame. */
      void redraw();

      /* Draw the saliency image on top of a pixmap */
      void drawSaliency(QImage *image, bool top);

      /* Draw the overlays on top of that pixmap  */
      void drawOverlays(QPixmap *pixmap);

      Kinematics kinematics;

      boost::numeric::ublas::matrix<float> createPoint(float a, float b,
                                                       float c);

      std::vector<boost::numeric::ublas::matrix<float> > fieldLines;
      std::vector<boost::numeric::ublas::matrix<float> > bodyParts;
      Blackboard *blackboard;
      Colour topSaliency[TOP_SALIENCY_ROWS][TOP_SALIENCY_COLS];
      Colour botSaliency[BOT_SALIENCY_ROWS][BOT_SALIENCY_COLS];

      QString instructionString;

      std::string printParams(Parameters &parameters);
   public slots:
      void newNaoData(NaoData *naoData);
      void redrawSlot();
      void incOffset();
      void decOffset();
      void updateOffset();
   signals:
      void sendCommandToRobot(std::vector<std::string> item);
};

#endif // CAMERA_POSE_TAB_HPP
