#ifndef TEMPERATURE_TAB_HPP
#define TEMPERATURE_TAB_HPP

#include <QMenuBar>
#include <QGridLayout>
#include "tabs/tab.hpp"
#include "utils/body.hpp"

class Vision;
class SensorValues;
class MultiPlot;

class TemperatureTab : public Tab {
   Q_OBJECT
   public:
      TemperatureTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void updatePlots(SensorValues &s, bool left=false);
      void updatePlots(std::vector<SensorValues> &s, bool left=false);
      QGridLayout *layout;
      MultiPlot* temperaturePlots[Joints::NUMBER_OF_JOINTS];
      int last_frame;
   public slots:
      void newNaoData(NaoData *naoData);
      void readerClosed();
};

#endif // TEMPERATURE_TAB_HPP
