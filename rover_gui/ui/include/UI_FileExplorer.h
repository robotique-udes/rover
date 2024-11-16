/********************************************************************************
** Form generated from reading UI file 'FileExplorer.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FILEEXPLORER_H
#define UI_FILEEXPLORER_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "../../src/QSshFileExplorer/QTreeViewExplorer.hpp"

QT_BEGIN_NAMESPACE

class Ui_FileExplorer
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_3;
    QLineEdit *le_user;
    QLineEdit *le_hostIP;
    QSpacerItem *horizontalSpacer;
    QCheckBox *cb_showHiddenFile;
    QPushButton *pb_refresh;
    QHBoxLayout *horizontalLayout;
    QLineEdit *le_path;
    QPushButton *pb_pathCopy;
    QTreeViewExplorer *tv_fileExplorer;
    QHBoxLayout *horizontalLayout_2;
    QProgressBar *progressBar;
    QPushButton *pb_cancelCurrentTask;
    QPushButton *pb_cancelAllTasks;

    void setupUi(QWidget *FileExplorer)
    {
        if (FileExplorer->objectName().isEmpty())
            FileExplorer->setObjectName(QString::fromUtf8("FileExplorer"));
        FileExplorer->resize(874, 530);
        FileExplorer->setCursor(QCursor(Qt::ArrowCursor));
        FileExplorer->setStyleSheet(QString::fromUtf8("QWidget {\n"
"    background-color: #2e2e2e;\n"
"    color: #ffffff;\n"
"}\n"
"\n"
"QMenuBar {\n"
"    background-color: #3c3f41;\n"
"    color: #ffffff;\n"
"}\n"
"\n"
"QMenu {\n"
"    background-color: #3c3f41;\n"
"    color: #ffffff;\n"
"}\n"
"\n"
"QMenu::item {\n"
"    padding: 5px 30px;\n"
"}\n"
"\n"
"QMenu::item:selected {\n"
"    background-color: #4a4e54;\n"
"}\n"
"\n"
"QPushButton {\n"
"    background-color: #3c3f41;\n"
"    border: 1px solid #4b4e52;\n"
"    border-radius: 5px;\n"
"    padding: 5px 10px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #4d4d4d;\n"
"}\n"
"\n"
"QLineEdit {\n"
"    background-color: #3c3f41;\n"
"    color: #ffffff;\n"
"    border: 1px solid #4b4e52;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QTextEdit {\n"
"    background-color: #3c3f41;\n"
"    color: #ffffff;\n"
"    border: 1px solid #4b4e52;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: #ffffff;\n"
"}\n"
"\n"
"QScrollBar:vertical {\n"
"    background: #2e2e2e;\n"
""
                        "    width: 10px;\n"
"}\n"
"\n"
"QScrollBar::handle:vertical {\n"
"    background: #4b4e52;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QScrollBar::add-line:vertical,\n"
"QScrollBar::sub-line:vertical {\n"
"    background: none;\n"
"}\n"
"\n"
"QCheckBox {\n"
"    color: #ffffff;\n"
"}\n"
"\n"
"QRadioButton {\n"
"    color: #ffffff;\n"
"}\n"
"\n"
"QRadioButton::indicator {\n"
"    background-color: #3c3f41;\n"
"    border: 1px solid #4b4e52;\n"
"}\n"
"\n"
"QRadioButton::indicator:checked {\n"
"    background-color: #4d4d4d;\n"
"}\n"
"\n"
"QTabWidget::pane {\n"
"    background-color: #2e2e2e;\n"
"}\n"
"\n"
"QTabBar::tab {\n"
"    background-color: #3c3f41;\n"
"    color: #ffffff;\n"
"    padding: 10px;\n"
"}\n"
"\n"
"QTabBar::tab:selected {\n"
"    background-color: #4d4d4d;\n"
"}\n"
"\n"
"QProgressBar {\n"
"    background-color: #2e2e2e; \n"
"    border: 2px solid #444444;\n"
"    border-radius: 2px;\n"
"    text-align: center;\n"
"    color: white;\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background-color: #"
                        "4caf50;\n"
"    border-radius: 2px;\n"
"}\n"
"\n"
"QProgressBar::text {\n"
"    color: white; \n"
"    font-weight: bold;\n"
"}\n"
"\n"
"QStatusBar {\n"
"    background-color: #3c3f41;\n"
"    color: #ffffff;\n"
"}"));
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

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        cb_showHiddenFile = new QCheckBox(FileExplorer);
        cb_showHiddenFile->setObjectName(QString::fromUtf8("cb_showHiddenFile"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(cb_showHiddenFile->sizePolicy().hasHeightForWidth());
        cb_showHiddenFile->setSizePolicy(sizePolicy1);
        cb_showHiddenFile->setCursor(QCursor(Qt::PointingHandCursor));
        cb_showHiddenFile->setLayoutDirection(Qt::RightToLeft);
        cb_showHiddenFile->setChecked(true);

        horizontalLayout_3->addWidget(cb_showHiddenFile);

        pb_refresh = new QPushButton(FileExplorer);
        pb_refresh->setObjectName(QString::fromUtf8("pb_refresh"));
        sizePolicy.setHeightForWidth(pb_refresh->sizePolicy().hasHeightForWidth());
        pb_refresh->setSizePolicy(sizePolicy);
        pb_refresh->setMinimumSize(QSize(0, 0));
        pb_refresh->setMaximumSize(QSize(200, 16777215));
        pb_refresh->setCursor(QCursor(Qt::PointingHandCursor));

        horizontalLayout_3->addWidget(pb_refresh);


        verticalLayout_3->addLayout(horizontalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        le_path = new QLineEdit(FileExplorer);
        le_path->setObjectName(QString::fromUtf8("le_path"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(le_path->sizePolicy().hasHeightForWidth());
        le_path->setSizePolicy(sizePolicy2);
        le_path->setReadOnly(false);
        le_path->setClearButtonEnabled(false);

        horizontalLayout->addWidget(le_path);

        pb_pathCopy = new QPushButton(FileExplorer);
        pb_pathCopy->setObjectName(QString::fromUtf8("pb_pathCopy"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(pb_pathCopy->sizePolicy().hasHeightForWidth());
        pb_pathCopy->setSizePolicy(sizePolicy3);
        pb_pathCopy->setMinimumSize(QSize(0, 0));
        pb_pathCopy->setCursor(QCursor(Qt::PointingHandCursor));
        QIcon icon;
        QString iconThemeName = QString::fromUtf8("edit-copy");
        if (QIcon::hasThemeIcon(iconThemeName)) {
            icon = QIcon::fromTheme(iconThemeName);
        } else {
            icon.addFile(QString::fromUtf8("."), QSize(), QIcon::Normal, QIcon::Off);
        }
        pb_pathCopy->setIcon(icon);
        pb_pathCopy->setIconSize(QSize(20, 20));
        pb_pathCopy->setAutoDefault(false);
        pb_pathCopy->setFlat(false);

        horizontalLayout->addWidget(pb_pathCopy);


        verticalLayout_3->addLayout(horizontalLayout);

        tv_fileExplorer = new QTreeViewExplorer(FileExplorer);
        tv_fileExplorer->setObjectName(QString::fromUtf8("tv_fileExplorer"));
        tv_fileExplorer->setMinimumSize(QSize(0, 300));
        tv_fileExplorer->viewport()->setProperty("cursor", QVariant(QCursor(Qt::PointingHandCursor)));
        tv_fileExplorer->setContextMenuPolicy(Qt::CustomContextMenu);
        tv_fileExplorer->setFrameShadow(QFrame::Sunken);
        tv_fileExplorer->setEditTriggers(QAbstractItemView::DoubleClicked);
        tv_fileExplorer->setTabKeyNavigation(true);
        tv_fileExplorer->setDragDropMode(QAbstractItemView::DragDrop);
        tv_fileExplorer->setDefaultDropAction(Qt::CopyAction);
        tv_fileExplorer->setAlternatingRowColors(false);
        tv_fileExplorer->setSelectionMode(QAbstractItemView::NoSelection);
        tv_fileExplorer->setSelectionBehavior(QAbstractItemView::SelectRows);
        tv_fileExplorer->setIndentation(0);
        tv_fileExplorer->setExpandsOnDoubleClick(false);
        tv_fileExplorer->header()->setVisible(true);

        verticalLayout_3->addWidget(tv_fileExplorer);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        progressBar = new QProgressBar(FileExplorer);
        progressBar->setObjectName(QString::fromUtf8("progressBar"));
        progressBar->setAutoFillBackground(false);
        progressBar->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
        progressBar->setMaximum(100);
        progressBar->setValue(25);
        progressBar->setTextVisible(true);
        progressBar->setOrientation(Qt::Horizontal);
        progressBar->setInvertedAppearance(false);
        progressBar->setTextDirection(QProgressBar::TopToBottom);

        horizontalLayout_2->addWidget(progressBar);

        pb_cancelCurrentTask = new QPushButton(FileExplorer);
        pb_cancelCurrentTask->setObjectName(QString::fromUtf8("pb_cancelCurrentTask"));
        sizePolicy.setHeightForWidth(pb_cancelCurrentTask->sizePolicy().hasHeightForWidth());
        pb_cancelCurrentTask->setSizePolicy(sizePolicy);
        pb_cancelCurrentTask->setMinimumSize(QSize(0, 0));
        pb_cancelCurrentTask->setMaximumSize(QSize(200, 16777215));
        pb_cancelCurrentTask->setCursor(QCursor(Qt::PointingHandCursor));

        horizontalLayout_2->addWidget(pb_cancelCurrentTask);

        pb_cancelAllTasks = new QPushButton(FileExplorer);
        pb_cancelAllTasks->setObjectName(QString::fromUtf8("pb_cancelAllTasks"));
        sizePolicy.setHeightForWidth(pb_cancelAllTasks->sizePolicy().hasHeightForWidth());
        pb_cancelAllTasks->setSizePolicy(sizePolicy);
        pb_cancelAllTasks->setMinimumSize(QSize(0, 0));
        pb_cancelAllTasks->setMaximumSize(QSize(200, 16777215));
        pb_cancelAllTasks->setCursor(QCursor(Qt::PointingHandCursor));

        horizontalLayout_2->addWidget(pb_cancelAllTasks);


        verticalLayout_3->addLayout(horizontalLayout_2);


        gridLayout->addLayout(verticalLayout_3, 0, 0, 1, 1);


        retranslateUi(FileExplorer);

        pb_pathCopy->setDefault(false);


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
        le_path->setText(QString());
        le_path->setPlaceholderText(QCoreApplication::translate("FileExplorer", "/home/user", nullptr));
        pb_pathCopy->setText(QString());
#if QT_CONFIG(shortcut)
        pb_pathCopy->setShortcut(QString());
#endif // QT_CONFIG(shortcut)
        pb_cancelCurrentTask->setText(QCoreApplication::translate("FileExplorer", "Cancel Current Task", nullptr));
        pb_cancelAllTasks->setText(QCoreApplication::translate("FileExplorer", "Cancel All Tasks", nullptr));
    } // retranslateUi

};

namespace Ui {
    class FileExplorer: public Ui_FileExplorer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FILEEXPLORER_H
