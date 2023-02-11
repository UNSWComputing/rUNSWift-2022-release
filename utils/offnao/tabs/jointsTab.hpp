#ifndef JOINTS_TAB_HPP
#define JOINTS_TAB_HPP

#include "tabs/tab.hpp"
#include "types/JointValues.hpp"

class Vision;
struct QTabWidget;
struct QGridLayout;
class MultiPlot;
struct QMenuBar;
class SensorValues;

class JointsTab : public Tab {
   Q_OBJECT
   public:
      JointsTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void updatePlots(SensorValues &s, JointValues &jr, bool left=false);
      void updatePlots(std::vector<SensorValues> &s, std::vector<JointValues> &jr, bool left=false);
      QGridLayout *layout;
      MultiPlot* jointPlots[Joints::NUMBER_OF_JOINTS];
      int last_frame;
   public slots:
      void newNaoData(NaoData *naoData);
      void readerClosed();
};


#endif // JOINTS_TAB_HPP
