#ifndef QUTILITY_BAR_BOTTOM_HPP
#define QUTILITY_BAR_BOTTOM_HPP

#include "UI_UtilityBarBottom.h"

class QUtilityBarBottom : public QWidget
{
    Q_OBJECT

  public:
    QUtilityBarBottom(QWidget* parent_);

  signals:
    void seeHistory();

  private:
    Ui::UtilityBarBottom _ui;
};

#endif //QUTILITY_BAR_BOTTOM_HPP