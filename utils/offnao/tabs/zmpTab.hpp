#ifndef ZMP_TAB_HPP
#define ZMP_TAB_HPP

#include <QGridLayout>
#include "tabs/tab.hpp"

class DataPlot;
class XYZPlot;

class ZMPTab : public Tab {
   Q_OBJECT
   public:
      ZMPTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      QGridLayout *layout;
      DataPlot* coronal_plot;
      DataPlot* sagittal_plot;
      XYZPlot* com_x_plot;
      XYZPlot* com_y_plot;
      XYZPlot* zmp_x_plot;
      XYZPlot* zmp_y_plot;
      int last_frame;
   public slots:
      void newNaoData(NaoData *naoData);
      void readerClosed();
};

#endif // ZMP_TAB_HPP
