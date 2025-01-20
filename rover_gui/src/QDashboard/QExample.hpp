#ifndef __QEXAMPLE_HPP__
#define __QEXAMPLE_HPP__

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

#include "UI_Example.h"

class QExample : public QWidget
{
	Q_OBJECT

	public:
		QExample(QWidget* parent_) : QWidget(parent_)
		{
			_ui.setupUi(this);
		}

        ~QExample(){};

    private:
        Ui::Example _ui;
};

#endif