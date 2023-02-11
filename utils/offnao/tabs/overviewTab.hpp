#ifndef OVERVIEW_TAB_HPP
#define OVERVIEW_TAB_HPP

#include <QGridLayout>
#include <QLabel>

#include "tabs/tab.hpp"
#include "tabs/variableView.hpp"
#include "fieldView.hpp"

class Vision;

class Blackboard;

/*
 * This is the default/initial tab that shows on entering Off-Nao.
 * As the name suggests it gives an overview of most of the important
 * aspects of the nao. Good for general gameplay watching etc.
 *
 * Also will be used for localization debugging unless the localization
 * people need more info.
 */
class OverviewTab : public Tab {
   Q_OBJECT
   public:
      OverviewTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision
                  );
      QPixmap *renderPixmap;
      QLabel *renderWidget;
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void redraw();

      /*  Draw the image on top of a pixmap */
      void drawImage(QImage *topImage, QImage *botImage);
      void drawOverlays(QPixmap *topImage, QPixmap *botImage);

      QString naoName;

      QGridLayout *layout;

      /* These variables are used to present the debug variables from the nao*/
      VariableView variableView;
      FieldView fieldView;

      /* Variables for vision */
      QPixmap topImagePixmap;
      QLabel *topCamLabel;
      QPixmap botImagePixmap;
      QLabel *botCamLabel;

      // Data
      Colour topSaliency[TOP_SALIENCY_ROWS][TOP_SALIENCY_COLS];
      Colour botSaliency[BOT_SALIENCY_ROWS][BOT_SALIENCY_COLS];
      Blackboard *blackboard;

   public slots:
      void newNaoData(NaoData *naoData);
      void setNao(const QString &naoName);
};


#endif // OVERVIEW_TAB_HPP
