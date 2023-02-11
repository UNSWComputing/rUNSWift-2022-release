#ifndef WALK_TAB_HPP
#define WALK_TAB_HPP

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <QObject>
#include <QGridLayout>

#include "tabs/tab.hpp"


class WalkTab : public Tab {
   Q_OBJECT
   public:
      WalkTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      QGridLayout *layout;
      QwtPlot *bodyPlot[2];
      QwtPlotCurve *curve[2];
      QwtPlotCurve *axis[2][2];
      std::list<float> theta[2], dTheta[2];
      int lastCurrent;
      static const unsigned int numEntries;
      double atheta[2][5], aDTheta[2][5];
      double zero[100], line[100];

   public slots:
      void newNaoData(NaoData *naoData);
      void readerClosed();
};


#endif // WALK_TAB_HPP
