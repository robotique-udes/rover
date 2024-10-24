/********************************************************************************
** Form generated from reading UI file 'Notification.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NOTIFICATION_H
#define UI_NOTIFICATION_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Notification
{
  public:
    QGridLayout* gridLayout;
    QVBoxLayout* verticalLayout;
    QHBoxLayout* horizontalLayout;
    QLabel* lb_sender;
    QSpacerItem* horizontalSpacer;
    QPushButton* pb_close;
    QScrollArea* scrollArea;
    QWidget* scrollAreaWidgetContents;
    QGridLayout* gridLayout_2;
    QLabel* lb_message;

    void setupUi(QWidget* Notification)
    {
        if (Notification->objectName().isEmpty())
            Notification->setObjectName(QString::fromUtf8("Notification"));
        Notification->resize(400, 150);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Notification->sizePolicy().hasHeightForWidth());
        Notification->setSizePolicy(sizePolicy);
        Notification->setMinimumSize(QSize(400, 150));
        Notification->setMaximumSize(QSize(471, 271));
        Notification->setStyleSheet(QString::fromUtf8(""));
        gridLayout = new QGridLayout(Notification);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(0);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        lb_sender = new QLabel(Notification);
        lb_sender->setObjectName(QString::fromUtf8("lb_sender"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(lb_sender->sizePolicy().hasHeightForWidth());
        lb_sender->setSizePolicy(sizePolicy1);
        lb_sender->setMaximumSize(QSize(16777215, 30));

        horizontalLayout->addWidget(lb_sender);

        horizontalSpacer = new QSpacerItem(10, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        pb_close = new QPushButton(Notification);
        pb_close->setObjectName(QString::fromUtf8("pb_close"));
        pb_close->setMinimumSize(QSize(30, 30));
        pb_close->setMaximumSize(QSize(30, 30));
        QIcon icon;
        QString iconThemeName = QString::fromUtf8("window-close");
        if (QIcon::hasThemeIcon(iconThemeName))
        {
            icon = QIcon::fromTheme(iconThemeName);
        }
        else
        {
            icon.addFile(QString::fromUtf8("."), QSize(), QIcon::Normal, QIcon::Off);
        }
        pb_close->setIcon(icon);
        pb_close->setIconSize(QSize(20, 20));
        pb_close->setAutoDefault(false);
        pb_close->setFlat(true);

        horizontalLayout->addWidget(pb_close);

        verticalLayout->addLayout(horizontalLayout);

        scrollArea = new QScrollArea(Notification);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        sizePolicy1.setHeightForWidth(scrollArea->sizePolicy().hasHeightForWidth());
        scrollArea->setSizePolicy(sizePolicy1);
        scrollArea->setMaximumSize(QSize(16777215, 16000000));
        scrollArea->setAutoFillBackground(false);
        scrollArea->setFrameShape(QFrame::NoFrame);
        scrollArea->setFrameShadow(QFrame::Sunken);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 380, 98));
        sizePolicy1.setHeightForWidth(scrollAreaWidgetContents->sizePolicy().hasHeightForWidth());
        scrollAreaWidgetContents->setSizePolicy(sizePolicy1);
        gridLayout_2 = new QGridLayout(scrollAreaWidgetContents);
        gridLayout_2->setSpacing(0);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        lb_message = new QLabel(scrollAreaWidgetContents);
        lb_message->setObjectName(QString::fromUtf8("lb_message"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Maximum);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(lb_message->sizePolicy().hasHeightForWidth());
        lb_message->setSizePolicy(sizePolicy2);
        lb_message->setMinimumSize(QSize(0, 0));
        lb_message->setTextFormat(Qt::RichText);
        lb_message->setScaledContents(false);
        lb_message->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
        lb_message->setWordWrap(true);
        lb_message->setOpenExternalLinks(true);

        gridLayout_2->addWidget(lb_message, 0, 0, 1, 1);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout->addWidget(scrollArea);

        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);

        retranslateUi(Notification);

        pb_close->setDefault(false);

        QMetaObject::connectSlotsByName(Notification);
    }  // setupUi

    void retranslateUi(QWidget* Notification)
    {
        Notification->setWindowTitle(QCoreApplication::translate("Notification", "Form", nullptr));
        lb_sender->setText(QCoreApplication::translate("Notification", "QSshFileSystem.cpp (326):", nullptr));
        pb_close->setText(QString());
        lb_message->setText(QCoreApplication::translate("Notification",
                                                        "<html><head/><body><p>Short message</p><p><br/></p></body></html>",
                                                        nullptr));
    }  // retranslateUi
};

namespace Ui
{
    class Notification : public Ui_Notification
    {
    };
}  // namespace Ui

QT_END_NAMESPACE

#endif  // UI_NOTIFICATION_H
