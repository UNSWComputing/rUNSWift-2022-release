#pragma once

#include <QObject>
#include <QWidget>
#include <QMenuBar>
#include <QString>
#include <QRgb>
#include <QPaintDevice>
#include <QTabWidget>

#include <vector>
#include <utility>
#include <stdint.h>


class Vision;
class BallInfo;
class RobotVisionInfo;
class FieldBoundaryInfo;
class FieldFeatureInfo;
class RegionI;
class NaoData;

class Tab : public QWidget {
   Q_OBJECT

   public:
      Tab() : vision(0), currentFrame(0) {}
      Tab(QTabWidget *parent) : QWidget(parent), parent(parent), vision(0),
                                topFrame(0), botFrame(0) {}

   protected:
      /* Returns the RGB value of the pixel at row, col
       * @arg: num_cols is used to vary the image resolution
       */
      virtual QRgb getRGB(unsigned int col, unsigned int row,
                          const uint8_t *yuv, int num_cols);

      /* Draw the raw image on top of a pixmap */
      void drawImage(QImage *image, bool top);

      QTabWidget *parent;

      /*  vision module from libsoccer */
      Vision *vision;

      /* Current working image
       * If you working with vision you need a frame.
       */
      const uint8_t *currentFrame;
      const uint8_t *topFrame;
      const uint8_t *botFrame;

      /*
       * Generic draw overlays function
       * Supply Null to any of the arguments if you do not wish to draw
       * a particular overlay. topImage is always required.
       */
      void drawOverlaysGeneric (QPaintDevice      *topImage,
            QPaintDevice                          *botImage,
            const std::pair<int, int>             *horizon,
            const std::vector<BallInfo>           *balls,
            const std::vector<RobotVisionInfo>          *robots,
            const std::vector<FieldBoundaryInfo>  *fieldBoundaries,
            const std::vector<FieldFeatureInfo>   *fieldFeatures,
            const std::vector<RegionI>            *regions,
            float scale
      );

      virtual void tabSelected();
      virtual void tabDeselected();

   signals:
      void showMessage(const QString &, int timeout = 0);

   public slots:
      virtual void newNaoData(NaoData *naoData) = 0;
      virtual void readerClosed() {}

   friend class Visualiser;
};
