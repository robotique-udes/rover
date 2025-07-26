#include "UI_LightsController.h"
#include <qpushbutton.h>

class QLightsController : public QWidget
{
  public:
    explicit QLightsController(QWidget* parent_):
        QWidget(parent_)
    {
        _ui.setupUi(this);
    }

    QPushButton& getToggleFrontLightsButton()
    {
        return *_ui._pb_lights;
    }

  private:
    Ui::LightsController _ui;
};
