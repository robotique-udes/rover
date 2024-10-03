// Ui files
#include "ui_login_example_widget.h"

class QLogin : public QWidget
{
public:
    QLogin()
    {
        ui.setupUi(this);
    }

    ~QLogin() {}

private:
    Ui::Login ui;
};