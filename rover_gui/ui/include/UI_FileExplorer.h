/********************************************************************************
** Form generated from reading UI file 'FileExplorer.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FILEEXPLORER_H
#define UI_FILEEXPLORER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FileExplorer
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_3;
    QLineEdit *le_user;
    QLineEdit *le_hostIP;
    QCheckBox *cb_showHiddenFile;
    QPushButton *pb_refresh;
    QTreeView *tv_fileExplorer;

    void setupUi(QWidget *FileExplorer)
    {
        if (FileExplorer->objectName().isEmpty())
            FileExplorer->setObjectName(QString::fromUtf8("FileExplorer"));
        FileExplorer->resize(560, 359);
        FileExplorer->setStyleSheet(QString::fromUtf8("    QWidget {\n"
"        background-color: #2e2e2e;\n"
"        color: #ffffff;\n"
"    }\n"
"\n"
"    QMenuBar {\n"
"        background-color: #3c3f41;\n"
"        color: #ffffff;\n"
"    }\n"
"\n"
"    QMenu {\n"
"        background-color: #3c3f41;\n"
"        color: #ffffff;\n"
"    }\n"
"\n"
"    QMenu::item {\n"
"        padding: 5px 30px;\n"
"    }\n"
"\n"
"    QMenu::item:selected {\n"
"        background-color: #4a4e54;\n"
"    }\n"
"\n"
"    QPushButton {\n"
"        background-color: #3c3f41;\n"
"        border: 1px solid #4b4e52;\n"
"        border-radius: 5px;\n"
"        padding: 5px 10px;\n"
"    }\n"
"\n"
"    QPushButton:hover {\n"
"        background-color: #4d4d4d;\n"
"    }\n"
"\n"
"    QLineEdit {\n"
"        background-color: #3c3f41;\n"
"        color: #ffffff;\n"
"        border: 1px solid #4b4e52;\n"
"        border-radius: 5px;\n"
"        padding: 5px;\n"
"    }\n"
"\n"
"    QTextEdit {\n"
"        background-color: #3c3f41;\n"
"        color: #ffffff;\n"
"        border: 1px solid #"
                        "4b4e52;\n"
"        border-radius: 5px;\n"
"    }\n"
"\n"
"    QLabel {\n"
"        color: #ffffff;\n"
"    }\n"
"\n"
"    QScrollBar:vertical {\n"
"        background: #2e2e2e;\n"
"        width: 10px;\n"
"    }\n"
"\n"
"    QScrollBar::handle:vertical {\n"
"        background: #4b4e52;\n"
"        border-radius: 5px;\n"
"    }\n"
"\n"
"    QScrollBar::add-line:vertical,\n"
"    QScrollBar::sub-line:vertical {\n"
"        background: none;\n"
"    }\n"
"\n"
"    QCheckBox {\n"
"        color: #ffffff;\n"
"    }\n"
"\n"
"    QCheckBox::indicator {\n"
"        background-color: #3c3f41;\n"
"        border: 1px solid #4b4e52;\n"
"    }\n"
"\n"
"    QCheckBox::indicator:checked {\n"
"        background-color: #4d4d4d;\n"
"    }\n"
"\n"
"    QRadioButton {\n"
"        color: #ffffff;\n"
"    }\n"
"\n"
"    QRadioButton::indicator {\n"
"        background-color: #3c3f41;\n"
"        border: 1px solid #4b4e52;\n"
"    }\n"
"\n"
"    QRadioButton::indicator:checked {\n"
"        background-color: #4d4d4d;\n"
"    }\n"
""
                        "\n"
"    QTabWidget::pane {\n"
"        background-color: #2e2e2e;\n"
"    }\n"
"\n"
"    QTabBar::tab {\n"
"        background-color: #3c3f41;\n"
"        color: #ffffff;\n"
"        padding: 10px;\n"
"    }\n"
"\n"
"    QTabBar::tab:selected {\n"
"        background-color: #4d4d4d;\n"
"    }\n"
"\n"
"    QProgressBar {\n"
"        background-color: #4b4e52;\n"
"        color: #ffffff;\n"
"    }\n"
"\n"
"    QProgressBar::chunk {\n"
"        background-color: #5cb85c;\n"
"    }\n"
"\n"
"    QStatusBar {\n"
"        background-color: #3c3f41;\n"
"        color: #ffffff;\n"
"    }"));
        gridLayout = new QGridLayout(FileExplorer);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        le_user = new QLineEdit(FileExplorer);
        le_user->setObjectName(QString::fromUtf8("le_user"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(le_user->sizePolicy().hasHeightForWidth());
        le_user->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(le_user);

        le_hostIP = new QLineEdit(FileExplorer);
        le_hostIP->setObjectName(QString::fromUtf8("le_hostIP"));
        sizePolicy.setHeightForWidth(le_hostIP->sizePolicy().hasHeightForWidth());
        le_hostIP->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(le_hostIP);

        cb_showHiddenFile = new QCheckBox(FileExplorer);
        cb_showHiddenFile->setObjectName(QString::fromUtf8("cb_showHiddenFile"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(cb_showHiddenFile->sizePolicy().hasHeightForWidth());
        cb_showHiddenFile->setSizePolicy(sizePolicy1);
        cb_showHiddenFile->setLayoutDirection(Qt::RightToLeft);

        horizontalLayout_3->addWidget(cb_showHiddenFile);

        pb_refresh = new QPushButton(FileExplorer);
        pb_refresh->setObjectName(QString::fromUtf8("pb_refresh"));
        sizePolicy.setHeightForWidth(pb_refresh->sizePolicy().hasHeightForWidth());
        pb_refresh->setSizePolicy(sizePolicy);
        pb_refresh->setMinimumSize(QSize(0, 0));
        pb_refresh->setMaximumSize(QSize(200, 16777215));

        horizontalLayout_3->addWidget(pb_refresh);


        verticalLayout_3->addLayout(horizontalLayout_3);

        tv_fileExplorer = new QTreeView(FileExplorer);
        tv_fileExplorer->setObjectName(QString::fromUtf8("tv_fileExplorer"));
        tv_fileExplorer->setMinimumSize(QSize(0, 300));
        tv_fileExplorer->setTabKeyNavigation(true);
        tv_fileExplorer->setIndentation(0);
        tv_fileExplorer->header()->setVisible(true);

        verticalLayout_3->addWidget(tv_fileExplorer);


        gridLayout->addLayout(verticalLayout_3, 0, 0, 1, 1);


        retranslateUi(FileExplorer);

        QMetaObject::connectSlotsByName(FileExplorer);
    } // setupUi

    void retranslateUi(QWidget *FileExplorer)
    {
        FileExplorer->setWindowTitle(QCoreApplication::translate("FileExplorer", "Form", nullptr));
        le_user->setText(QString());
        le_user->setPlaceholderText(QCoreApplication::translate("FileExplorer", "User", nullptr));
        le_hostIP->setText(QString());
        le_hostIP->setPlaceholderText(QCoreApplication::translate("FileExplorer", "Host IP", nullptr));
        cb_showHiddenFile->setText(QCoreApplication::translate("FileExplorer", "Show hidden files", nullptr));
        pb_refresh->setText(QCoreApplication::translate("FileExplorer", "Refresh", nullptr));
    } // retranslateUi

};

namespace Ui {
    class FileExplorer: public Ui_FileExplorer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FILEEXPLORER_H
