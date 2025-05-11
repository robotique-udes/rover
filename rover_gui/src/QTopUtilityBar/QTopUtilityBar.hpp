#ifndef QTOP_UTILITY_BAR
#define QTOP_UTILITY_BAR

#include "UI_TopUtilityBar.h"
class QTopUtilityBar : public QWidget
{
    Q_OBJECT

  public:
    QTopUtilityBar(QWidget* parent_);

  private:
    Ui::TopUtilityBar _ui;
        
};


#endif //QTOP_UTILITY_BAR