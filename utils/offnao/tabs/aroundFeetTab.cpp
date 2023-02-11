#include "aroundFeetTab.hpp"
#include <QGridLayout>
#include <QPainter>
#include <fstream>
#include <sstream>
#include "types/FeetPosition.hpp"
#include "blackboard/Blackboard.hpp"
#include "naoData.hpp"

using namespace std;

#define PIXMAP_WIDTH 640.0
#define PIXMAP_HEIGHT 640.0
#define PIXMAP_W_TO_H_RATIO PIXMAP_WIDTH / PIXMAP_HEIGHT

#define MIN_X -100.0 // mm
#define MAX_X 400.0 // mm
#define MIN_Y -PIXMAP_W_TO_H_RATIO * (MAX_X - MIN_X) / 2
#define MAX_Y PIXMAP_W_TO_H_RATIO * (MAX_X - MIN_X) / 2
#define GRID_SPACING 20.0 // mm

#define HIP_OFFSET 50.0 // mm

#define MM_PER_CM 10 // mm


/* Painter specific to aroundfeet tab */
class AroundFeetPainter : public QPainter
{
  public:
   AroundFeetPainter(QPaintDevice *device);
   void drawLeftFoot(const FootPosition &leftFootPosition, const QPolygon &polyLeftFoot);
   void drawRightFoot(const FootPosition &rightFootPosition, const QPolygon &polyRightFoot);
   void drawGrids();
   void drawAxes();
   void drawVisionBall(float x, float y);
   void drawEgoBall(float x, float y);
   void drawLud(std::vector<std::vector<char> > ludMap);
  private:
   static QPolygon polyLeftFoot;
   static QPolygon polyRightFoot;
};


AroundFeetTab::AroundFeetTab(QTabWidget *parent, QMenuBar *menuBar,
      Vision *vision) {
   initMenu(menuBar);
   this->vision = vision;
   this->parent = parent;

   init();
}

void AroundFeetTab::initMenu(QMenuBar *) {
}

void AroundFeetTab::init() {

   // create layout
   QGridLayout *layout = new QGridLayout();
   setLayout(layout);
   layout->setAlignment(Qt::AlignTop);
   layout->addWidget(&label, 0, 0);

   // generate initial pixmap and fill with grey
   pixmap = QPixmap(PIXMAP_WIDTH, PIXMAP_HEIGHT);
   pixmap.fill(QColor(163, 163, 163));

   // Create painter
   AroundFeetPainter painter(&pixmap);

   // Draw grid, using GRID_SPACING
   painter.drawGrids();

   // Draw x and y axes with thicker line
   painter.drawAxes();

   // Draw line up data map
   std::string ludFileName(std::string(getenv("RUNSWIFT_CHECKOUT_DIR")) + std::string("/image/home/nao/data/line_up_data.lud"));
   std::ifstream ludIn(ludFileName.c_str());
   if (!ludIn.is_open()) {
      std::cerr << "(aroundFeetTab) could not open " << ludFileName << endl;
   } else {
      // read file into 2d char array
      while (!ludIn.eof()) {
         std::vector<char> lineData;
         std::string line;
         std::getline(ludIn, line);

         std::istringstream ludss(line);
         char data;
         while (ludss >> data)
         {
            lineData.push_back(data);
         }
         if (!lineData.empty())
           ludMap.push_back(lineData);
      }

      painter.drawLud(ludMap);
   }

   // set the pixmap
   label.setPixmap(pixmap);

   // generate left foot polygon
   QVector<QPoint> pointsLeftFoot;
   pointsLeftFoot.push_back(QPoint(0, 40));
   pointsLeftFoot.push_back(QPoint(10, 40));
   pointsLeftFoot.push_back(QPoint(20, 41));
   pointsLeftFoot.push_back(QPoint(30, 43));
   pointsLeftFoot.push_back(QPoint(40, 47));
   pointsLeftFoot.push_back(QPoint(50, 50));
   pointsLeftFoot.push_back(QPoint(60, 49));
   pointsLeftFoot.push_back(QPoint(70, 47));
   pointsLeftFoot.push_back(QPoint(80, 42));
   pointsLeftFoot.push_back(QPoint(90, 32));
   pointsLeftFoot.push_back(QPoint(95, 25));
   pointsLeftFoot.push_back(QPoint(100, 15));
   pointsLeftFoot.push_back(QPoint(102, 5));
   pointsLeftFoot.push_back(QPoint(102, -5));
   pointsLeftFoot.push_back(QPoint(100, -15));
   pointsLeftFoot.push_back(QPoint(95, -25));
   pointsLeftFoot.push_back(QPoint(85, -33));
   pointsLeftFoot.push_back(QPoint(75, -37));
   pointsLeftFoot.push_back(QPoint(65, -39));
   pointsLeftFoot.push_back(QPoint(55, -39));
   pointsLeftFoot.push_back(QPoint(45, -37));
   pointsLeftFoot.push_back(QPoint(35, -36));
   pointsLeftFoot.push_back(QPoint(25, -34));
   pointsLeftFoot.push_back(QPoint(15, -33));
   pointsLeftFoot.push_back(QPoint(5, -33));
   pointsLeftFoot.push_back(QPoint(-5, -33));
   pointsLeftFoot.push_back(QPoint(-15, -34));
   pointsLeftFoot.push_back(QPoint(-25, -34));
   pointsLeftFoot.push_back(QPoint(-33, -35));
   pointsLeftFoot.push_back(QPoint(-40, -32));
   pointsLeftFoot.push_back(QPoint(-50, -21));
   pointsLeftFoot.push_back(QPoint(-55, -10));
   pointsLeftFoot.push_back(QPoint(-57, 5));
   pointsLeftFoot.push_back(QPoint(-56, 15));
   pointsLeftFoot.push_back(QPoint(-54, 25));
   pointsLeftFoot.push_back(QPoint(-49, 35));
   pointsLeftFoot.push_back(QPoint(-41, 45));
   pointsLeftFoot.push_back(QPoint(-32, 52));
   pointsLeftFoot.push_back(QPoint(-27, 51));
   pointsLeftFoot.push_back(QPoint(-20, 47));
   pointsLeftFoot.push_back(QPoint(-13, 42));
   polyLeftFoot = QPolygon(pointsLeftFoot);

   // generate right foot polygon (by modifying the pointsLeftFoot)
   QVector<QPoint> pointsRightFoot;
   for (QVector<QPoint>::const_iterator it = pointsLeftFoot.begin(); it != pointsLeftFoot.end(); ++it)
   {
      pointsRightFoot.push_back(QPoint(it->x(), -it->y())); // negate the sign of y
   }
   polyRightFoot = QPolygon(pointsRightFoot);
}

void AroundFeetTab::newNaoData(NaoData *naoData) {

   // Get new blackboard, if it exists
   Blackboard *blackboard;
   if (! (naoData && (blackboard = naoData->getCurrentFrame ().blackboard))) {
      return;
   }

   // copy the original pixmap, and create a painter
   renderPixmap = pixmap;
   AroundFeetPainter painter(&renderPixmap);

   // If we have vision balls, draw them
   if (blackboard->vision.balls.size() > 0)
   {
      RRCoord &visionBallRR = blackboard->vision.balls[0].rr;
      float x = visionBallRR.distance() * cosf(visionBallRR.heading());
      float y = visionBallRR.distance() * sinf(visionBallRR.heading());
      painter.drawVisionBall(x, y);
   }

   // Draw ego ball
   AbsCoord egoBallRRC = blackboard->stateEstimation.ballPosRRC;
   painter.drawEgoBall(egoBallRRC.x(), egoBallRRC.y());
   painter.drawPoint(egoBallRRC.x(), egoBallRRC.y());

   // Draw feet
   FeetPosition &feetPosition = blackboard->motion.motionDebugInfo.feetPosition;
   painter.drawLeftFoot(feetPosition.left, polyLeftFoot);
   painter.drawRightFoot(feetPosition.right, polyRightFoot);

   // Finally, set the pixmap!
   label.setPixmap(renderPixmap);
}

/* Around Feet Painter class functions */

AroundFeetPainter::AroundFeetPainter(QPaintDevice *device) : QPainter (device)
{
   // translate, scale and rotate to match robot's coordinates
   translate(PIXMAP_WIDTH/2, PIXMAP_HEIGHT * (MAX_X / (MAX_X - MIN_X)));
   rotate(-90);
   scale(1, -1);
   float scalingFactor = PIXMAP_HEIGHT/(MAX_X - MIN_X);
   scale(scalingFactor, scalingFactor);
}

void AroundFeetPainter::drawGrids()
{
   setPen(QColor(102, 102, 102));

   // draw horizontal lines
   for (int i = 1; i < MAX_X / GRID_SPACING; ++i)
   {
      float x = i * GRID_SPACING;
      drawLine(x, MIN_Y, x, MAX_Y);
   }
   for (int i = -1; i > MIN_X / GRID_SPACING; --i)
   {
      float x = i * GRID_SPACING;
      drawLine(x, MIN_Y, x, MAX_Y);
   }

   // draw vertical lines
   for (int i = 1; i < MAX_Y / GRID_SPACING; ++i)
   {
      float y = i * GRID_SPACING;
      drawLine(MIN_X, y, MAX_X, y);
   }
   for (int i = -1; i > MIN_Y / GRID_SPACING; --i)
   {
      float y = i * GRID_SPACING;
      drawLine(MIN_X, y, MAX_X, y);
   }
}

void AroundFeetPainter::drawAxes()
{
   setPen(QPen(QColor(30, 30, 30), 3));
   drawLine(MIN_X, 0, MAX_X, 0);
   drawLine(0, MIN_Y, 0, MAX_Y);
}

void AroundFeetPainter::drawLeftFoot(const FootPosition &leftFootPosition, const QPolygon &polyLeftFoot)
{
   setBrush(QBrush(QColor(100, 100, 100, 100)));
   setPen(QColor(80, 80, 80));

   save();
   translate(leftFootPosition.x, HIP_OFFSET + leftFootPosition.y);
   rotate(leftFootPosition.theta);
   drawPolygon(polyLeftFoot);
   restore();
}

void AroundFeetPainter::drawRightFoot(const FootPosition &rightFootPosition, const QPolygon &polyRightFoot)
{
   setBrush(QBrush(QColor(100, 100, 100, 100)));
   setPen(QColor(80, 80, 80));

   save();
   translate(rightFootPosition.x, -HIP_OFFSET + rightFootPosition.y);
   rotate(rightFootPosition.theta);
   drawPolygon(polyRightFoot);
   restore();
}

void AroundFeetPainter::drawVisionBall(float x, float y)
{
   setBrush(QBrush(QColor(255, 131, 109, 190)));
   setPen(QColor(255, 64, 30));
   drawEllipse(QPointF(x, y), BALL_RADIUS, BALL_RADIUS);
}

void AroundFeetPainter::drawEgoBall(float x, float y)
{
   setBrush(QBrush(QColor(102, 170, 255, 190)));
   setPen(QColor(17, 124, 255));
   drawEllipse(QPointF(x, y), BALL_RADIUS, BALL_RADIUS);
   drawEllipse(QPointF(x, y), 5, 5);  // visualisation of centre of ball
}

void AroundFeetPainter::drawLud(std::vector<std::vector<char> > ludMap)
{
   // display on map where the zone is
   int ludRows = ludMap.size();
   int ludCols = ludRows > 0 ? ludMap[0].size() : 0;
   setPen(QPen(QColor(0, 0, 0, 0))); // transparent pen
   for (int i = 0; i < ludRows; ++i)
   {
      for (int j = 0; j < ludCols; ++j)
      {
         char data = ludMap[i][j];
         if (data == '/' || data == 'o' || data == '-')
         {
            QColor color;
            switch(data)
            {
            case '/':
               color = QColor(118, 188, 132, 130);
               break;
            case 'o':
               color = QColor(0, 173, 34, 130);
               break;
            case '-':
               color = QColor(255, 255, 255, 90);
               break;
            }

            setBrush(color);
            int x = MM_PER_CM * (ludRows - i - 1);
            int y = MM_PER_CM * (ludCols - j - 1);
            drawRect(QRect(x, y, 10, 10));
         }
      }
   }
}
