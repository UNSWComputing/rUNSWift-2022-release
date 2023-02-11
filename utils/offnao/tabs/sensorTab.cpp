#include <QMenu>
#include <QMenuBar>
#include <QDebug>

#include <iostream>
#include <fstream>
#include <sstream>
#include "sensorTab.hpp"
#include "utils/basic_maths.hpp"

#include "types/SensorValues.hpp"
#include "blackboard/Blackboard.hpp"
#include "perception/vision/Vision.hpp"
#include "naoData.hpp"

using namespace std;


#define POS_FILE_HEADER "# HY    HP    LSP   LSR   LEY   LER   LWY   LHYP  LHR   LHP   LKP   LAP   LAR   RHR   RHP   RKP   RAP   RAR   RSP   RSR   REY   RER   RWY   LH    RH    DUR"

#define COL_WIDTH 6


SensorTab::SensorTab(QTabWidget *parent, QMenuBar *menuBar,
      Vision *vision)  {
   initMenu(menuBar);
   init();
   this->vision = vision;
   this->parent = parent;
}


void SensorTab::initMenu(QMenuBar *) {
}

void SensorTab::init() {
   layout = new QGridLayout(this);
   setLayout(layout);
   layout->setAlignment(layout, Qt::AlignTop);
   layout->setHorizontalSpacing(5);
   layout->setHorizontalSpacing(5);

   jointValueLabel = new QLabel();
   jointValueLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
   jointValueLabel->setFont(QFont("Monospace"));

   sensorValueLabel = new QLabel();
   sensorValueLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
   sensorValueLabel->setFont(QFont("Monospace"));

   genPos = new QPushButton("Generate Pos");

   updateDataView(NULL);
   layout->addWidget(jointValueLabel, 0, 0, 1, 1);
   layout->addWidget(sensorValueLabel, 0, 1, 1, 1);
   layout->addWidget(genPos, 0, 3, 1, 1);

   connect(genPos, SIGNAL(released()), this, SLOT(generatePosFile()));
}

void SensorTab::updateDataView(Blackboard *blackboard) {
   stringstream jointStream;
   SensorValues s;

   if(blackboard != NULL) {
      s = readFrom(motion, sensors);
   }
   joints.clear();
   for(unsigned int i = 0;
       i < sizeof(Joints::jointCodes)/sizeof(Joints::JointCode);
       i++) {
      jointStream.width(20);
      jointStream << Joints::jointNames[i] << ": ";
      if(blackboard == NULL) {
         jointStream << "";
      } else {
         jointStream << s.joints.angles[i] << "  (" <<
                        RAD2DEG(s.joints.angles[i]) << ") "
                        << s.joints.currents[i];
         joints.push_back(static_cast<int>(RAD2DEG(s.joints.angles[i])));
      }
      jointStream << std::endl;
   }
   jointValueLabel->setText(QString(jointStream.str().c_str()));

   stringstream sensorStream;
   for(unsigned int i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i) {
      sensorStream.width(25);
      sensorStream << Sensors::sensorNames[i] << ": ";
      if(blackboard == NULL) {
         sensorStream << "";
      } else {
         sensorStream << s.sensors[i];
      }
      sensorStream << std::endl;
   }
   sensorStream << std::endl;
   sensorStream << "LFoot total: " << s.sensors[Sensors::LFoot_FSR_FrontLeft] +
                                      s.sensors[Sensors::LFoot_FSR_FrontRight] +
                                      s.sensors[Sensors::LFoot_FSR_RearLeft] +
                                      s.sensors[Sensors::LFoot_FSR_RearRight]
                                   << std::endl;
   sensorStream << "RFoot total: " << s.sensors[Sensors::RFoot_FSR_FrontLeft]
                                    + s.sensors[Sensors::RFoot_FSR_FrontRight]
                                    + s.sensors[Sensors::RFoot_FSR_RearLeft]
                                    + s.sensors[Sensors::RFoot_FSR_RearRight]
                                   << std::endl;
   sensorStream << "Total weight: " << s.sensors[Sensors::LFoot_FSR_FrontLeft]
                                     + s.sensors[Sensors::LFoot_FSR_FrontRight]
                                     + s.sensors[Sensors::LFoot_FSR_RearLeft]
                                     + s.sensors[Sensors::LFoot_FSR_RearRight]
                                     + s.sensors[Sensors::RFoot_FSR_FrontLeft]
                                     + s.sensors[Sensors::RFoot_FSR_FrontRight]
                                     + s.sensors[Sensors::RFoot_FSR_RearLeft]
                                     + s.sensors[Sensors::RFoot_FSR_RearRight]
                                    << std::endl;
   sensorValueLabel->setText(QString(sensorStream.str().c_str()));
}

void SensorTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {  // clean up display, as read is finished
      if (parent->currentIndex() == parent->indexOf(this)) {
         updateDataView(NULL);
      }
   } else if (naoData->getFramesTotal() != 0) {
      Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
      if (parent->currentIndex() == parent->indexOf(this)) {
         updateDataView(blackboard);
      }
   }
}

void SensorTab::generatePosFile() {
   // Create new file to hold the current pose
   std::ofstream fs("current_pose.pos", std::ios_base::app);

   if (!fs) {
      std::cerr << "Cannot open the output file." << std::endl;
      return;
   }

   fs << POS_FILE_HEADER;
   fs << std::endl;
   fs << "! "; // initial padding

   // move LH to end of file
   float LH = joints[7];
   unsigned int i;
   for(i = 7; i < joints.size()-2; i++) {
      joints[i] = joints[i+1];
   }
   joints[i] = LH;

   // Add joint values to the file
   for (unsigned int i = 0; i < joints.size(); i ++) {
      stringstream ss;
      ss << joints[i];
      std::string value = ss.str();

      // Pad the columns of the file to the right size
      while (value.length() < COL_WIDTH) {
         value += " ";
      }

      // Add the column to the line of the file
      fs << value;
   }

   // Add a default duration of 1s
   fs << "1000" << std::endl;

   fs.close();
}

