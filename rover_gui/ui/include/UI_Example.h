/********************************************************************************
** Form generated from reading UI file 'Example.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EXAMPLE_H
#define UI_EXAMPLE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Example
{
public:
    QGridLayout *gridLayout;
    QLabel *label_2;
    QLabel *label;
    QLabel *lb_longitude;
    QLabel *lb_latitude;
    QLabel *label_5;

    void setupUi(QWidget *Example)
    {
        if (Example->objectName().isEmpty())
            Example->setObjectName(QString::fromUtf8("Example"));
        Example->resize(400, 150);
        Example->setMinimumSize(QSize(400, 150));
        Example->setMaximumSize(QSize(400, 150));
        gridLayout = new QGridLayout(Example);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_2 = new QLabel(Example);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        QFont font;
        font.setBold(false);
        label_2->setFont(font);

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        label = new QLabel(Example);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font1;
        font1.setPointSize(11);
        label->setFont(font1);
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 0, 0, 1, 4);

        lb_longitude = new QLabel(Example);
        lb_longitude->setObjectName(QString::fromUtf8("lb_longitude"));
        lb_longitude->setFont(font);

        gridLayout->addWidget(lb_longitude, 1, 3, 1, 1);

        lb_latitude = new QLabel(Example);
        lb_latitude->setObjectName(QString::fromUtf8("lb_latitude"));

        gridLayout->addWidget(lb_latitude, 1, 1, 1, 1);

        label_5 = new QLabel(Example);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setFont(font);

        gridLayout->addWidget(label_5, 1, 2, 1, 1);


        retranslateUi(Example);

        QMetaObject::connectSlotsByName(Example);
    } // setupUi

    void retranslateUi(QWidget *Example)
    {
        Example->setWindowTitle(QCoreApplication::translate("Example", "Form", nullptr));
        label_2->setText(QCoreApplication::translate("Example", "Latitude : ", nullptr));
        label->setText(QCoreApplication::translate("Example", "Ceci est un example de widget connect\303\251 \303\240 ROS", nullptr));
        lb_longitude->setText(QCoreApplication::translate("Example", "69.9999", nullptr));
        lb_latitude->setText(QCoreApplication::translate("Example", "69.9999", nullptr));
        label_5->setText(QCoreApplication::translate("Example", "Longitude : ", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Example: public Ui_Example {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EXAMPLE_H
