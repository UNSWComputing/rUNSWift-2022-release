#include <QMenu>
#include <QMenuBar>
#include <qpushbutton.h>
#include <QFileDialog>
#include <QDebug>
#include <QPainter>
#include <QLine>
#include <QLabel>
#include <QPoint>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QTextEdit>
#include <vector>
#include <iostream>
#include <utility>
#include <sstream>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "cameraPoseTab.hpp"
#include "utils/matrix_helpers.hpp"

#include "utils/basic_maths.hpp"
#include "utils/SPLDefs.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "blackboard/Blackboard.hpp"

#include "utils/CPlaneColours.hpp"
#include "naoData.hpp"
#include "tab.hpp"

using namespace std;
using namespace boost::numeric::ublas;

CameraPoseTab::CameraPoseTab(QTabWidget *parent,
      QMenuBar *menuBar, Vision *vision) {
   initMenu(menuBar);
   init();
   this->vision = vision;
   this->parent = parent;

   // set up vector of lines
   // Centre line

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2, -FIELD_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2, -FIELD_WIDTH/2, 0));

   // Far goal box

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH +
            FIELD_LINE_WIDTH/2, GOAL_BOX_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH +
            FIELD_LINE_WIDTH/2, -GOAL_BOX_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, GOAL_BOX_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, -GOAL_BOX_WIDTH/2, 0));

   // lines that connect the goal box to the edge of field

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH,
            GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2,
            GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH,
            GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2,
            GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));

   // Near goal box

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH,
            -GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2,
            -GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH,
            -GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2,
            -GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));

   // other box

   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH +
            FIELD_LINE_WIDTH/2, GOAL_BOX_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH +
            FIELD_LINE_WIDTH/2, -GOAL_BOX_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, GOAL_BOX_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, -GOAL_BOX_WIDTH/2, 0));

   // other box: lines that connect the goal box to the edge of field

   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH,
            GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2,
            GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH,
            GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2,
            GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));


   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH,
            -GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2,
            -GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH,
            -GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2,
            -GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));

   // middle cross

   fieldLines.push_back(createPoint(PENALTY_CROSS_ABS_X - FIELD_LINE_WIDTH,
            FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(PENALTY_CROSS_ABS_X + FIELD_LINE_WIDTH,
            FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(PENALTY_CROSS_ABS_X - FIELD_LINE_WIDTH,
            -FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(PENALTY_CROSS_ABS_X + FIELD_LINE_WIDTH,
            -FIELD_LINE_WIDTH/2, 0));


   fieldLines.push_back(createPoint(PENALTY_CROSS_ABS_X - FIELD_LINE_WIDTH/2,
            -FIELD_LINE_WIDTH, 0));
   fieldLines.push_back(createPoint(PENALTY_CROSS_ABS_X - FIELD_LINE_WIDTH/2,
            +FIELD_LINE_WIDTH, 0));

   fieldLines.push_back(createPoint(PENALTY_CROSS_ABS_X + FIELD_LINE_WIDTH/2,
            -FIELD_LINE_WIDTH, 0));
   fieldLines.push_back(createPoint(PENALTY_CROSS_ABS_X + FIELD_LINE_WIDTH/2,
            +FIELD_LINE_WIDTH, 0));

   // Split up these lines since for some reason they show up oddly when a complete line

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, 0, 0));

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, 0, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, -FIELD_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, 0, 0));

   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, 0, 0));
   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, -FIELD_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, 0, 0));

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, 0, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, -FIELD_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, 0, 0));

   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, 0, 0));
   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, -FIELD_WIDTH/2, 0));

   //side lines

   // Split up these lines since for some reason they show up oddly when a complete line

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, FIELD_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(0, FIELD_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(0, FIELD_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, FIELD_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, FIELD_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(0, FIELD_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(0, FIELD_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, FIELD_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, -FIELD_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(0, -FIELD_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(0, -FIELD_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, -FIELD_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, -FIELD_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(0, -FIELD_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(0, -FIELD_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, -FIELD_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));

   // generate centre circle
   int PARTS = 20;
   for(int i = 0; i < PARTS; i++) {
      float angle = (i * 360 / PARTS) * M_PI/180.0;
      float n_angle = ((i+1) * 360 / PARTS) * M_PI/180.0;

      int sx = cos(angle) * (CENTER_CIRCLE_DIAMETER/2.0 +
            FIELD_LINE_WIDTH/2);
      int sy = sin(angle) * (CENTER_CIRCLE_DIAMETER/2.0 +
            FIELD_LINE_WIDTH/2);

      int nx = cos(n_angle) * (CENTER_CIRCLE_DIAMETER/2.0 +
            FIELD_LINE_WIDTH/2);
      int ny = sin(n_angle) * (CENTER_CIRCLE_DIAMETER/2.0 +
            FIELD_LINE_WIDTH/2);
      fieldLines.push_back(createPoint(sx, sy, 0));
      fieldLines.push_back(createPoint(nx, ny, 0));

      sx = cos(angle) * (CENTER_CIRCLE_DIAMETER/2.0 -
            FIELD_LINE_WIDTH/2);
      sy = sin(angle) * (CENTER_CIRCLE_DIAMETER/2.0 -
            FIELD_LINE_WIDTH/2);

      nx = cos(n_angle) * (CENTER_CIRCLE_DIAMETER/2.0 -
            FIELD_LINE_WIDTH/2);
      ny = sin(n_angle) * (CENTER_CIRCLE_DIAMETER/2.0 -
            FIELD_LINE_WIDTH/2);
      fieldLines.push_back(createPoint(sx, sy, 0));
      fieldLines.push_back(createPoint(nx, ny, 0));
   }
    bodyParts.push_back(createPoint(-44, -39.5, Limbs::NeckOffsetZ -32 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-34, -54, Limbs::NeckOffsetZ - 32 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-38, -70, Limbs::NeckOffsetZ - 25 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-33, -90, Limbs::NeckOffsetZ + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-22, -95, Limbs::NeckOffsetZ + 9 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-8, -90, Limbs::NeckOffsetZ + 15.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(10, -90, Limbs::NeckOffsetZ + 17.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(25, -90, Limbs::NeckOffsetZ + 8.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(34, -81, Limbs::NeckOffsetZ -2.2 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(43, -83, Limbs::NeckOffsetZ - 21 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(31, -61, Limbs::NeckOffsetZ -35 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(50, -39, Limbs::NeckOffsetZ - 48 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(55, -20, Limbs::NeckOffsetZ - 48 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(55, 0, Limbs::NeckOffsetZ - 45 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(55, 20, Limbs::NeckOffsetZ - 48 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(50, 39, Limbs::NeckOffsetZ - 48 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(31, 61, Limbs::NeckOffsetZ -35 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(43, 83, Limbs::NeckOffsetZ - 21 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(34, 81, Limbs::NeckOffsetZ -2.2 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(25, 90, Limbs::NeckOffsetZ + 8.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(10, 90, Limbs::NeckOffsetZ + 17.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-8, 90, Limbs::NeckOffsetZ + 15.5 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-22, 95, Limbs::NeckOffsetZ + 9 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-33, 90, Limbs::NeckOffsetZ + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-38, 70, Limbs::NeckOffsetZ - 25 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-34, 54, Limbs::NeckOffsetZ - 32 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
    bodyParts.push_back(createPoint(-44, 39.5, Limbs::NeckOffsetZ -32 + Limbs::HipOffsetZ - Limbs::HipOffsetY));
}


void CameraPoseTab::initMenu(QMenuBar *menuBar) {
}

void CameraPoseTab::init() {
   layout = new QGridLayout();
   this->setLayout(layout);

   // We use the size of the (smaller) bottom camera image and
   // scale the (larger) top camera image down so it fits within the screen
   imagePixmap = QPixmap(BOT_IMAGE_COLS, BOT_IMAGE_ROWS);
   imagePixmap.fill(Qt::darkGray);
   camLabel  = new QLabel();
   camLabel->setPixmap(imagePixmap);

   layout->addWidget(camLabel, 0, 0, 1, 1);

   optionsLayout = new QGridLayout();
   layout->addLayout(optionsLayout, 0, 1, 1, 1, Qt::AlignTop);

   instructionString += "1) Place robot on T junction (one intersecting with the centre circle at (0, 3000))\n";
   instructionString += "2) Face robots feet and head towards the centre circle\n";
   instructionString += "3) Check image to see if field lines match\n";
   instructionString += "4) Turn the head to check more field lines\n";
   instructionString += "5) Adjust offset if they do not match\n";
   instructionString += "6) Update config file for robot when they match\n";

   QLabel *instructionsBox = new QLabel(instructionString);
   optionsLayout->addWidget(instructionsBox, 0, 0, 1, 1);

   QGridLayout *configureLayout = new QGridLayout();
   offsetYawTopLabel = new QLineEdit("0.0");
   offsetPitchTopLabel = new QLineEdit("0.0");
   offsetRollTopLabel = new QLineEdit("0.0");

   offsetYawBottomLabel = new QLineEdit("0.0");
   offsetPitchBottomLabel = new QLineEdit("0.0");
   offsetRollBottomLabel = new QLineEdit("0.0");
   offsetBodyPitchLabel = new QLineEdit("0.0");

   offsetYawTopLabel->setMaximumSize(50, 30);
   offsetPitchTopLabel->setMaximumSize(50, 30);
   offsetRollTopLabel->setMaximumSize(50, 30);

   offsetYawBottomLabel->setMaximumSize(50, 30);
   offsetPitchBottomLabel->setMaximumSize(50, 30);
   offsetRollBottomLabel->setMaximumSize(50, 30);
   offsetBodyPitchLabel->setMaximumSize(50, 30);


   calibrationOutputBox = new QTextEdit("");
   offsetBodyPitchLabel = new QLineEdit("0.0");

   calibrationOutputBox->setMaximumSize(400, 400);
   offsetBodyPitchLabel->setMaximumSize(50, 30);

   QPushButton *sendButton = new QPushButton("send");
   whichCamera2 = new QRadioButton(tr("BOTTOM Camera"));
   whichCamera3 = new QRadioButton(tr("TOP Camera"));
   rawImage = new QCheckBox(tr("Raw Image"));
   whichCamera2->setChecked(true);
   QVBoxLayout *vbox = new QVBoxLayout;
   vbox->addWidget(whichCamera2);
   vbox->addWidget(whichCamera3);
   vbox->addWidget(rawImage);
   vbox->addWidget(calibrationOutputBox);
   // QPushButton *decButton = new QPushButton("-");
   // decButton->setMaximumSize(30,30);
   // incButton->setMaximumSize(30,30);

   //configureLayout->addWidget(decButton, 0, 0, 1, 1);
   configureLayout->addWidget(new QLabel("Bottom Camera Yaw"), 0, 0, 1, 1);
   configureLayout->addWidget(offsetYawBottomLabel, 0, 1, 1, 1);
   configureLayout->addWidget(new QLabel("Bottom Camera Pitch"), 1, 0, 1, 1);
   configureLayout->addWidget(offsetPitchBottomLabel, 1, 1, 1, 1);

   configureLayout->addWidget(new QLabel("Bottom Camera Roll"), 2, 0, 1, 1);
   configureLayout->addWidget(offsetRollBottomLabel, 2, 1, 1, 1);


   configureLayout->addWidget(new QLabel("Top Camera Yaw"), 3, 0, 1, 1);
   configureLayout->addWidget(offsetYawTopLabel, 3, 1, 1, 1);

   configureLayout->addWidget(new QLabel("Top Camera Pitch"), 4, 0, 1, 1);
   configureLayout->addWidget(offsetPitchTopLabel, 4, 1, 1, 1);

   configureLayout->addWidget(new QLabel("Top Camera Roll"), 5, 0, 1, 1);
   configureLayout->addWidget(offsetRollTopLabel, 5, 1, 1, 1);


   configureLayout->addWidget(new QLabel("Body Pitch"), 6, 0, 1, 1);
   configureLayout->addWidget(offsetBodyPitchLabel, 6, 1, 1, 1);

   configureLayout->addWidget(sendButton, 7, 0, 1, 2);
   configureLayout->addLayout(vbox, 8, 0, 1, 2, Qt::AlignTop);
   optionsLayout->addLayout(configureLayout, 1, 0, 1, 1,
                            Qt::AlignLeft);

   connect(sendButton, SIGNAL(clicked()), this, SLOT(updateOffset()));

   camLabel->setAlignment(Qt::AlignTop);
   camLabel->setMinimumSize(BOT_IMAGE_COLS, BOT_IMAGE_ROWS);
   camLabel->setMaximumSize(BOT_IMAGE_COLS, BOT_IMAGE_ROWS);
   camLabel->installEventFilter(this);
}

void CameraPoseTab::redraw() {
   bool top = whichCamera3->isChecked();
   if (top) {
      lastRendering =  QImage(TOP_IMAGE_COLS,
                              TOP_IMAGE_ROWS, QImage::Format_RGB32);
   } else {
      lastRendering =  QImage(BOT_IMAGE_COLS,
                              BOT_IMAGE_ROWS, QImage::Format_RGB32);
   }

   if (rawImage->isChecked () && (topFrame != NULL && botFrame != NULL)) {
         drawImage (&lastRendering, top);
   } else {
         drawSaliency (&lastRendering, top);
   }

   if (top) {
      imagePixmap = QPixmap(QPixmap::fromImage(
         lastRendering.scaled(BOT_IMAGE_COLS, BOT_IMAGE_ROWS)));
   } else {
      imagePixmap = QPixmap(QPixmap::fromImage(lastRendering));
   }
   drawOverlays(&imagePixmap);

   camLabel->setPixmap(imagePixmap);
   Parameters p = readFrom(kinematics, parameters);
   std::string s = printParams(p);
   calibrationOutputBox->setText(QString(s.c_str()));
}

void CameraPoseTab::drawSaliency(QImage *image, bool top){
   int s_row, s_col;
   if (top) {
      for (unsigned int row = 0; row < TOP_IMAGE_ROWS; ++row) {
         for (unsigned int col = 0; col < TOP_IMAGE_COLS; ++col) {
            s_row = row / TOP_SALIENCY_DENSITY;
            s_col = col / TOP_SALIENCY_DENSITY;
            image->setPixel(col, row,
          CPLANE_COLOURS[topSaliency[s_row][s_col]].rgb());
         }
      }
   } else {
      for (unsigned int row = 0; row < BOT_IMAGE_ROWS; ++row) {
         for (unsigned int col = 0; col < BOT_IMAGE_COLS; ++col) {
            s_row = row / BOT_SALIENCY_DENSITY;
            s_col = col / BOT_SALIENCY_DENSITY;
            image->setPixel(col, row,
          CPLANE_COLOURS[botSaliency[s_row][s_col]].rgb());
         }
      }
   }
}

boost::numeric::ublas::matrix<float> CameraPoseTab::createPoint(float a,
      float b,
      float c) {
   matrix<float> point(4, 1);
   point(0, 0) = a;
   point(1, 0) = b;
   point(2, 0) = c;
   point(3, 0) = 1;
   return point;
}

void CameraPoseTab::drawOverlays(QPixmap *pixmap) {
   QPainter painter(pixmap);

   // set up robot abs position and heading
   //QPoint position(0, FIELD_WIDTH/2);
   QPoint position(0, FIELD_WIDTH/2 + 105 + FIELD_LINE_WIDTH / 2.0);
   float heading = DEG2RAD(90);

   if (!blackboard) return;


   SensorValues sensorValues;
   for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
      sensorValues.joints.angles[i] = 0;
   }
   sensorValues.joints.angles[Joints::HeadPitch] = DEG2RAD(-10);
   if(blackboard) {
      sensorValues = readFrom(motion, sensors);
      kinematics.parameters = readFrom(kinematics, parameters);
   }
   kinematics.setSensorValues(sensorValues);
   Kinematics::Chain foot = kinematics.determineSupportChain();

   kinematics.updateDHChain();
   matrix<float> c2w;
   if (whichCamera3->isChecked()) {
      c2w = kinematics.createCameraToWorldTransform(foot, true);
   } else {
      c2w = kinematics.createCameraToWorldTransform(foot, false);
   }

   matrix<float> w2c = c2w;

   invertMatrix(c2w, w2c);
   matrix<float> ctest(4, 1);
   ctest(0, 0) = 0;
   ctest(1, 0) = 0;
   ctest(2, 0) = 0;
   ctest(3, 0) = 1;

   // add on translation to position
   matrix<float> pmatrix(4, 4);
   pmatrix(0, 0) = 1;
   pmatrix(0, 1) = 0;
   pmatrix(0, 2) = 0;
   pmatrix(0, 3) = -position.x();

   pmatrix(1, 0) = 0;
   pmatrix(1, 1) = 1;
   pmatrix(1, 2) = 0;
   pmatrix(1, 3) = -position.y();

   pmatrix(2, 0) = 0;
   pmatrix(2, 1) = 0;
   pmatrix(2, 2) = 1;
   pmatrix(2, 3) = 0;

   pmatrix(3, 0) = 0;
   pmatrix(3, 1) = 0;
   pmatrix(3, 2) = 0;
   pmatrix(3, 3) = 1;

   matrix<float> rmatrix = rotateZMatrix(heading);
   w2c = prod(w2c, rmatrix);
   w2c = prod(w2c, pmatrix);

   matrix<float> cmatrix = w2c;


   // finally set up projection matrix
   float ex = 0;
   float ey = 0;
   float ez = 1.0 / tan(blackboard->vision.horizontalFieldOfView / 2);
   matrix<float> projection(4, 4);
   projection(0, 0) = 1;
   projection(0, 1) = 0;
   projection(0, 2) = 0;
   projection(0, 3) = -ex;

   projection(1, 0) = 0;
   projection(1, 1) = 1;
   projection(1, 2) = 0;
   projection(1, 3) = -ey;

   projection(2, 0) = 0;
   projection(2, 1) = 0;
   projection(2, 2) = 1;
   projection(2, 3) = 0;

   projection(3, 0) = 0;
   projection(3, 1) = 0;
   projection(3, 2) = 1.0/ez;
   projection(3, 3) = 0;

   matrix<float> transform = prod(projection, w2c);
   std::vector<matrix<float> > imageLines;
   for(unsigned int i = 0; i < fieldLines.size(); i++) {
      matrix<float> pixel = prod(transform, fieldLines[i]);
      pixel(0, 0) /= ABS(pixel(3, 0));
      pixel(1, 0) /= ABS(pixel(3, 0));
      pixel(2, 0) = 0;
      float xscale = BOT_IMAGE_COLS/2;
      float yscale = BOT_IMAGE_ROWS/2;
      pixel(0, 0) = (pixel(0, 0)) * xscale + xscale;
      pixel(1, 0) = (pixel(1, 0)) * xscale + yscale;

      imageLines.push_back(pixel);
   }

   for(unsigned int i = 0; i < fieldLines.size(); i += 2) {
      if (imageLines[i](3, 0) >= 0 || imageLines[i+1](3, 0) >= 0) {
         painter.setPen("red");
         painter.drawLine(imageLines[i](0, 0),
               imageLines[i](1, 0),
               imageLines[i+1](0, 0),
               imageLines[i+1](1, 0));
            }
   }
   // Body Parts
   matrix<float> c2t;
   if (whichCamera3->isChecked())
   {
      c2t = kinematics.evaluateDHChain(Kinematics::BODY, Kinematics::CAMERA, foot, true);
   }
   else
   {
      c2t = kinematics.evaluateDHChain(Kinematics::BODY, Kinematics::CAMERA, foot, false);
   }

   matrix<float> t2c = c2t;
   invertMatrix(c2t, t2c);


   std::vector<matrix<float> > bodyImageLines;
   for (unsigned int i = 0; i < bodyParts.size(); i++)
   {
      matrix<float> product = prod(t2c, bodyParts[i]);
      matrix<float> pixel(4,1);

      pixel(0,0) = BOT_IMAGE_COLS/2 + BOT_IMAGE_COLS/2 * (atan2(product(0, 0), product(2, 0)) / (blackboard->vision.horizontalFieldOfView/2));
      pixel(1,0) = BOT_IMAGE_ROWS/2 + BOT_IMAGE_ROWS/2 * (atan2(product(1, 0), product(2, 0)) / (blackboard->vision.verticalFieldOfView/2));
      pixel(2,0) = 0;
      pixel(3,0) = 1;

      bodyImageLines.push_back(pixel);
   }

   // draw lines
   for (unsigned int i = 0; i < bodyImageLines.size() - 1; i += 1)
   {
      if (bodyImageLines[i](3, 0) >= 0 || bodyImageLines[i + 1](3, 0) >= 0)
      {
            painter.setPen(QColor("red"));
            painter.drawLine(bodyImageLines[i](0, 0),
                           bodyImageLines[i](1, 0),
                           bodyImageLines[i + 1](0, 0),
                           bodyImageLines[i + 1](1, 0));
      }
   }

   // draw points
   for (unsigned int i = 0; i < bodyImageLines.size(); i += 1)
   {
      if (bodyImageLines[i](3, 0) >= 0)
      {
            painter.setPen(QColor("red"));
            painter.drawRect(bodyImageLines[i](0, 0), bodyImageLines[i](1, 0), 10, 10);
      }
   }
}

void CameraPoseTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      imagePixmap.fill(Qt::darkGray);
      camLabel->setPixmap(imagePixmap);
      blackboard = NULL;
   } else {
      if (parent->currentIndex() == parent->indexOf(this)) {
         blackboard = naoData->getCurrentFrame().blackboard;
         topFrame = readFrom(vision, topFrame);
         botFrame = readFrom(vision, botFrame);
         if (!(rawImage->isChecked() && topFrame)) {
            if (readFrom(vision, topSaliency))
               memcpy(topSaliency, readFrom(vision, topSaliency),
                  TOP_SALIENCY_ROWS*TOP_SALIENCY_COLS*sizeof(Colour));
         }
         if (!(rawImage->isChecked() && botFrame)) {
            if (readFrom(vision, botSaliency))
               memcpy(botSaliency, readFrom(vision, botSaliency),
                  BOT_SALIENCY_ROWS*BOT_SALIENCY_COLS*sizeof(Colour));
         }
         redraw();
      }
   }
}

void CameraPoseTab::redrawSlot() {
   redraw();
}

void CameraPoseTab::updateOffset() {
   if (blackboard) {
      std::vector<std::string> commands;
      commands.push_back(createCommandString("--kinematics.cameraYawBottom=", offsetYawBottomLabel->text().toStdString()));
      commands.push_back(createCommandString("--kinematics.cameraPitchBottom=", offsetPitchBottomLabel->text().toStdString()));
      commands.push_back(createCommandString("--kinematics.cameraRollBottom=", offsetRollBottomLabel->text().toStdString()));
      commands.push_back(createCommandString("--kinematics.cameraYawTop=", offsetYawTopLabel->text().toStdString()));
      commands.push_back(createCommandString("--kinematics.cameraPitchTop=", offsetPitchTopLabel->text().toStdString()));
      commands.push_back(createCommandString("--kinematics.cameraRollTop=", offsetRollTopLabel->text().toStdString()));
      commands.push_back(createCommandString("--kinematics.bodyPitch=", offsetBodyPitchLabel->text().toStdString()));
      emit sendCommandToRobot(commands);
   }
}

std::string CameraPoseTab::createCommandString(std::string argument, std::string message) {
   stringstream ss;
   ss << argument;
   ss << message;
   return ss.str();
}

void CameraPoseTab::incOffset() {
}

void CameraPoseTab::decOffset() {
}

std::string CameraPoseTab::printParams(Parameters &parameters) {
   std::stringstream s;
   std::vector<std::pair<std::string, float> > plist;
   plist.push_back(std::make_pair(
                      "cameraYawBottom", parameters.cameraYawBottom));
   plist.push_back(std::make_pair(
                      "cameraPitchBottom", parameters.cameraPitchBottom));
   plist.push_back(std::make_pair(
                      "cameraRollBottom", parameters.cameraRollBottom));
   plist.push_back(std::make_pair(
                      "cameraYawTop", parameters.cameraYawTop));
   plist.push_back(std::make_pair(
                      "cameraPitchTop", parameters.cameraPitchTop));
   plist.push_back(std::make_pair(
                      "cameraRollTop", parameters.cameraRollTop));


   plist.push_back(std::make_pair(
                      "bodyPitch", parameters.bodyPitch));

   for (unsigned int i = 0; i < plist.size(); i++) {
      s << plist[i].first << "=" << plist[i].second << std::endl;
   }
   return s.str();
}
