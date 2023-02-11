#include "tabs/plots.hpp"
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include "utils/angles.hpp"
#include "types/SensorValues.hpp"
#include "types/XYZ_Coord.hpp"


#if QWT_VERSION >= 0x060000
#define setRawData setRawSamples
#endif

void DataPlot::alignScales() {
   // The code below shows how to align the scales to
   // the canvas frame, but is also a good example demonstrating
   // why the spreaded API needs polishing.

   setFrameStyle(QFrame::Box | QFrame::Plain);
   setLineWidth(1);

   for (int i = 0; i < QwtPlot::axisCnt; i++) {
      QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
      if (scaleWidget)
         scaleWidget->setMargin(0);

      QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
      if (scaleDraw)
         scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
   }
}

void XYZPlot::alignScales() {
   // The code below shows how to align the scales to
   // the canvas frame, but is also a good example demonstrating
   // why the spreaded API needs polishing.

   setFrameStyle(QFrame::Box | QFrame::Plain);
   setLineWidth(1);

   for (int i = 0; i < QwtPlot::axisCnt; i++) {
      QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
      if (scaleWidget)
         scaleWidget->setMargin(0);

      QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
      if (scaleDraw)
         scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
   }
}

void DataPlot::push(std::vector<SensorValues> &sensors, bool left) {
   if (left)
      for (std::vector<SensorValues>::reverse_iterator it = sensors.rbegin();
           it != sensors.rend(); it++)
         push(*it, left);
   else
      for (std::vector<SensorValues>::iterator it = sensors.begin();
           it != sensors.end(); it++)
         push(*it, left);
}

void XYZPlot::push(std::vector<XYZ_Coord> &coms, bool left) {
   if (left)
      for (std::vector<XYZ_Coord>::reverse_iterator it = coms.rbegin();
           it != coms.rend(); it++)
         push(*it, left);
   else
      for (std::vector<XYZ_Coord>::iterator it = coms.begin();
           it != coms.end(); it++)
         push(*it, left);
}

DataPlot::DataPlot(QWidget *parent):
   QwtPlot(parent) {

#if QWT_VERSION >= 0x060000
   // We don't need the cache here
   ((QwtPlotCanvas*)canvas())->setPaintAttribute(QwtPlotCanvas::BackingStore, false);
#else
   // Disable polygon clipping
   QwtPainter::setDeviceClipping(false);

   // We don't need the cache here
   ((QwtPlotCanvas*)canvas())->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
   ((QwtPlotCanvas*)canvas())->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);
#endif

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
   /*
      Qt::WA_PaintOnScreen is only supported for X11, but leads
      to substantial bugs with Qt 4.2.x/Windows
   */
   canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

   alignScales();

   for (int i = 0; i < PLOT_SIZE; ++i) {
      t[i] = i;
   }
}

XYZPlot::XYZPlot(QWidget *parent):
   QwtPlot(parent) {

#if QWT_VERSION >= 0x060000
   // We don't need the cache here
   ((QwtPlotCanvas*)canvas())->setPaintAttribute(QwtPlotCanvas::BackingStore, false);
#else
   // Disable polygon clipping
   QwtPainter::setDeviceClipping(false);

   // We don't need the cache here
   ((QwtPlotCanvas*)canvas())->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
   ((QwtPlotCanvas*)canvas())->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);
#endif

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
   /*
      Qt::WA_PaintOnScreen is only supported for X11, but leads
      to substantial bugs with Qt 4.2.x/Windows
   */
   canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

   alignScales();

   for (int i = 0; i < PLOT_SIZE; ++i) {
      t[i] = i;
   }
}

MultiPlot::MultiPlot(QWidget *parent, const std::string &title, u_int size, int min, int max)
   : DataPlot(parent) {
   datum = new double*[size];
   for (u_int i = 0; i < size; ++i) {
      datum[i] = new double[PLOT_SIZE];
      for (u_int j = 0; j < PLOT_SIZE; j++) {
         datum[i][j] = 0.0;
      }
      QwtPlotCurve *compareCurve = new QwtPlotCurve();
      compareCurve->attach(this);
      compareCurve->setRawData(t, datum[i], PLOT_SIZE);
      Qt::GlobalColor colour = static_cast<Qt::GlobalColor>(Qt::red + i);
      compareCurve->setPen(QPen(colour));
   }

   QwtPlotGrid *grid = new QwtPlotGrid();
   grid->enableX(true);
   grid->enableY(true);
   QPen transparentBlackPen(QColor(0x70, 0x70, 0x70, 0x20));
   grid->setPen(transparentBlackPen);
   grid->attach(this);

   setTitle(QString::fromUtf8(title.c_str()));

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, min, max);
}

void MultiPlot::push(std::vector<float> values, bool left) {
   if (left)
      for (u_int i = 0; i < values.size(); ++i) {
         for (u_int j = PLOT_SIZE-1; j > 0; --j) {
            datum[i][j] = datum[i][j - 1];
         }
      }
   else
      for (u_int i = 0; i < values.size(); ++i) {
         for (u_int j = 0; j < PLOT_SIZE - 1; ++j) {
            datum[i][j] = datum[i][j + 1];
         }
      }
   for (u_int i = 0; i < values.size(); ++i) {
      datum[i][left ? 0 : PLOT_SIZE - 1] = values[i];
   }
   replot();
}

void MultiPlot::push(SensorValues &sensors, bool left) {}


CoronalZMPPlot::CoronalZMPPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i) {
      data_l[i] = 0.0;
      data_r[i] = 0.0;
      data_t[i] = 0.0;
   }
   setTitle("Coronal Plane");

   QwtPlotCurve *curveL = new QwtPlotCurve("Left");
   curveL->attach(this);
   curveL->setRawData(t, data_l, PLOT_SIZE);
   curveL->setPen(QPen(Qt::red));

   QwtPlotCurve *curveR = new QwtPlotCurve("Right");
   curveR->attach(this);
   curveR->setRawData(t, data_r, PLOT_SIZE);
   curveR->setPen(QPen(Qt::blue));

   QwtPlotCurve *curveT = new QwtPlotCurve("Total");
   curveT->attach(this);
   curveT->setRawData(t, data_t, PLOT_SIZE);

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void CoronalZMPPlot::push(SensorValues &sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i) {
         data_l[i] = data_l[i-1];
         data_r[i] = data_r[i-1];
         data_t[i] = data_t[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i) {
         data_l[i] = data_l[i+1];
         data_r[i] = data_r[i+1];
         data_t[i] = data_t[i+1];
      }
   float total_weight = (sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::LFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_RearRight]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::RFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_RearRight]);
   if (total_weight != 0)
      data_t[left ? 0 : PLOT_SIZE - 1] =
         (80.0f*sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
        + 30.0f*sensors.sensors[Sensors::LFoot_FSR_FrontRight]
        + 80.0f*sensors.sensors[Sensors::LFoot_FSR_RearLeft]
        + 30.0f*sensors.sensors[Sensors::LFoot_FSR_RearRight]
        - 30.0f*sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
        - 80.0f*sensors.sensors[Sensors::RFoot_FSR_FrontRight]
        - 30.0f*sensors.sensors[Sensors::RFoot_FSR_RearLeft]
        - 80.0f*sensors.sensors[Sensors::RFoot_FSR_RearRight]) / total_weight;
   else
      data_t[left ? 0 : PLOT_SIZE - 1] = 0.0f;
   data_l[left ? 0 : PLOT_SIZE - 1] = 1000 *
#ifdef CTC_2_1
      sensors.sensors[Sensors::LFoot_FSR_CenterOfPressure_Y];
#else
      0;
#endif
   data_r[left ? 0 : PLOT_SIZE - 1] = 1000 *
#ifdef CTC_2_1
      sensors.sensors[Sensors::RFoot_FSR_CenterOfPressure_Y];
#else
      0;
#endif
   replot();
}

SagittalZMPPlot::SagittalZMPPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("Sagittal Plane");

   QwtPlotCurve *curve = new QwtPlotCurve("SagittalZMP");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void SagittalZMPPlot::push(SensorValues &sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];
   float total_weight = (sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::LFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_RearRight]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::RFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_RearRight]);
   if (total_weight != 0)
      data[left ? 0 : PLOT_SIZE - 1] =
         (50.0f*sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
        + 50.0f*sensors.sensors[Sensors::LFoot_FSR_FrontRight]
        - 50.0f*sensors.sensors[Sensors::LFoot_FSR_RearLeft]
        - 50.0f*sensors.sensors[Sensors::LFoot_FSR_RearRight]
        + 50.0f*sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
        + 50.0f*sensors.sensors[Sensors::RFoot_FSR_FrontRight]
        - 50.0f*sensors.sensors[Sensors::RFoot_FSR_RearLeft]
        - 50.0f*sensors.sensors[Sensors::RFoot_FSR_RearRight]) / total_weight;
   else
      data[left ? 0 : PLOT_SIZE - 1] = 0.0f;
   replot();
}

COMxPlot::COMxPlot(QWidget *parent)
   : XYZPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("COM relative to foot touching ground x");

   QwtPlotCurve *curve = new QwtPlotCurve("COMx");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void COMxPlot::push(XYZ_Coord &coord, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];

   data[left ? 0 : PLOT_SIZE - 1] = coord.x;

   replot();
}

COMyPlot::COMyPlot(QWidget *parent)
   : XYZPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("COM relative to foot touching ground y");

   QwtPlotCurve *curve = new QwtPlotCurve("COMy");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void COMyPlot::push(XYZ_Coord &coord, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];

   data[left ? 0 : PLOT_SIZE - 1] = coord.y;

   replot();
}
