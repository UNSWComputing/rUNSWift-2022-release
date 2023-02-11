#ifndef FIELD_VIEW_HPP
#define FIELD_VIEW_HPP

#include <QLabel>

struct NaoData;
class AbsCoord;

class FieldView: public QLabel {
   public:
      FieldView();
      ~FieldView();

      void redraw(NaoData *naoData);

      // Function to just draw a single robot coordinate on field
      void redraw(std::vector<AbsCoord> &robotPos);

      void redraw(std::vector<AbsCoord> &robotPos,
                  std::vector<AbsCoord> &ballPos);

   private:
      QPixmap imagePixmap;
      QPixmap *renderPixmap;
};

#endif // FIELD_VIEW_HPP
