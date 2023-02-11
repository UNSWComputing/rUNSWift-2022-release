#ifndef PLOTS_HPP
#define PLOTS_HPP

#include <qwt_plot.h>
#include <vector>

class SensorValues;
class Odometry;
class XYZ_Coord;

// PLOT_SIZE ~= 30s at 30fps or 9s at 100fps
#define PLOT_SIZE 101

class DataPlot : public QwtPlot {
   Q_OBJECT

   public:
      DataPlot(QWidget* = NULL);
      // put one frame in front of
      virtual void push(SensorValues &sensors, bool left=false) = 0;
      void push(std::vector<SensorValues> &sensors, bool left=false);

   protected:
      void alignScales();
      double t[PLOT_SIZE];
};

class MultiPlot : public DataPlot {
   Q_OBJECT

   public:
      MultiPlot(QWidget* parent, const std::string &title, u_int size, int min, int max);
      void push(std::vector<float> values, bool left=false);
      virtual void push(SensorValues &sensors, bool left=false);

   private:
      double **datum;
};

class XYZPlot : public QwtPlot {
   Q_OBJECT

   public:
      XYZPlot(QWidget* = NULL);
      // put one frame in front of
      virtual void push(XYZ_Coord &coord, bool left=false) = 0;
      void push(std::vector<XYZ_Coord> &coords, bool left=false);

   protected:
      void alignScales();
      double t[PLOT_SIZE];
};

class CoronalZMPPlot : public DataPlot {
   Q_OBJECT

   public:
      CoronalZMPPlot(QWidget* = NULL);
      virtual void push(SensorValues &sensors, bool left=false);
   private:
      double data_l[PLOT_SIZE];
      double data_r[PLOT_SIZE];
      double data_t[PLOT_SIZE];
};
class SagittalZMPPlot : public DataPlot {
   Q_OBJECT

   public:
      SagittalZMPPlot(QWidget* = NULL);
      virtual void push(SensorValues &sensors, bool left=false);
   private:
      double data[PLOT_SIZE];
};
class COMxPlot : public XYZPlot {
   Q_OBJECT

   public:
      COMxPlot(QWidget* = NULL);
      virtual void push(XYZ_Coord &com, bool left=false);
   private:
      double data[PLOT_SIZE];
};
class COMyPlot : public XYZPlot {
   Q_OBJECT

   public:
      COMyPlot(QWidget* = NULL);
      virtual void push(XYZ_Coord &com, bool left=false);
   private:
      double data[PLOT_SIZE];
};

#endif // PLOTS_HPP
