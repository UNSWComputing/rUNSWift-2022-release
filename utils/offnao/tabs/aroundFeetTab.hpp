#ifndef AROUND_FEET_TAB_HPP
#define AROUND_FEET_TAB_HPP

#include "tabs/tab.hpp"
#include <QLabel>
#include <QPolygon>

class QwtPlotGrid;

class AroundFeetTab : public Tab {
   Q_OBJECT
   public:
      AroundFeetTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);

      QLabel label;
      QPixmap pixmap;
      QPixmap renderPixmap;

      QPolygon polyLeftFoot;
      QPolygon polyRightFoot;

      std::vector<std::vector<char> > ludMap;

      /* Redraw the image box */
      void redraw();

   public slots:
      void newNaoData(NaoData *naoData);
};

#endif // AROUND_FEET_TAB_HPP
