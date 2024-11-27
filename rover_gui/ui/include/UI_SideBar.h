/********************************************************************************
** Form generated from reading UI file 'SideBar.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SIDEBAR_H
#define UI_SIDEBAR_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SideBar
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QPushButton *pb_dashboard;
    QPushButton *pb_navigation;
    QPushButton *pb_science;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *SideBar)
    {
        if (SideBar->objectName().isEmpty())
            SideBar->setObjectName(QString::fromUtf8("SideBar"));
        SideBar->resize(180, 791);
        SideBar->setMinimumSize(QSize(180, 0));
        SideBar->setMaximumSize(QSize(180, 16777215));
        gridLayout = new QGridLayout(SideBar);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(SideBar);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font;
        font.setPointSize(24);
        font.setBold(true);
        font.setWeight(QFont::Weight::Bold);
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(label);

        pb_dashboard = new QPushButton(SideBar);
        pb_dashboard->setObjectName(QString::fromUtf8("pb_dashboard"));

        verticalLayout->addWidget(pb_dashboard);

        pb_navigation = new QPushButton(SideBar);
        pb_navigation->setObjectName(QString::fromUtf8("pb_navigation"));

        verticalLayout->addWidget(pb_navigation);

        pb_science = new QPushButton(SideBar);
        pb_science->setObjectName(QString::fromUtf8("pb_science"));

        verticalLayout->addWidget(pb_science);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);


        retranslateUi(SideBar);

        QMetaObject::connectSlotsByName(SideBar);
    } // setupUi

    void retranslateUi(QWidget *SideBar)
    {
        SideBar->setWindowTitle(QCoreApplication::translate("SideBar", "Form", nullptr));
        label->setText(QCoreApplication::translate("SideBar", "Rover GUI", nullptr));
        pb_dashboard->setText(QCoreApplication::translate("SideBar", "Dashboard", nullptr));
        pb_navigation->setText(QCoreApplication::translate("SideBar", "Navigation", nullptr));
        pb_science->setText(QCoreApplication::translate("SideBar", "Science", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SideBar: public Ui_SideBar {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SIDEBAR_H
