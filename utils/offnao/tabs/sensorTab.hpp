#ifndef SENSOR_TAB_HPP
#define SENSOR_TAB_HPP

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>

#include "tabs/tab.hpp"


class Vision;
class Blackboard;

class SensorTab : public Tab {
   Q_OBJECT
   public:
      SensorTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void updateDataView(Blackboard *blackboard);
      QGridLayout *layout;
      QLabel *jointValueLabel;
      QLabel *sensorValueLabel;
      QLabel *sonarValueLabel;
      QPushButton *genPos;
      std::vector<int> joints;
   public slots:
      void newNaoData(NaoData *naoData);
      void generatePosFile();
};

#endif // SENSOR_TAB_HPP
