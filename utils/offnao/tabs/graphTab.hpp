#ifndef GRAPH_TAB_HPP
#define GRAPH_TAB_HPP

#include "tabs/tab.hpp"

class Vision;
class QGridLayout;
class MultiPlot;
class SensorValues;
class Odometry;


class GraphTab : public Tab {
   Q_OBJECT
   public:
      GraphTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void updatePlots(SensorValues &s, Odometry &o, bool left=false);
      void updatePlots(std::vector<SensorValues> &s, std::vector<Odometry> &o, bool left=false);
      QGridLayout *layout;
      MultiPlot* accPlots[3];
      MultiPlot* gyrPlots[3];
      MultiPlot* anglePlots[2];
      MultiPlot* odomPlot;
      int last_frame;
   public slots:
      void newNaoData(NaoData *naoData);
      void readerClosed();
};


#endif // GRAPH_TAB_HPP
