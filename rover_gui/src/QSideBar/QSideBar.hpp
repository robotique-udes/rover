#ifndef QSIDE_BAR_HPP
#define QSIDE_BAR_HPP

#include "UI_SideBar.h"

class QSideBar : public QWidget
{
    Q_OBJECT

  public:
    QSideBar(QWidget* parent_);

  signals:
    void switchPage(int pageIndex);

  private:
    Ui::SideBar _ui;
};

#endif  // QSIDE_BAR_HPP
