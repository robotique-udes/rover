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
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
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
    QSpacerItem *horizontalSpacer;
    QCheckBox *cb_showHiddenFile;
    QPushButton *pb_refresh;
    QHBoxLayout *horizontalLayout;
    QLineEdit *le_path;
    QPushButton *pb_pathCopy;
    QTreeView *tv_fileExplorer;

    void setupUi(QWidget *FileExplorer)
    {
        if (FileExplorer->objectName().isEmpty())
            FileExplorer->setObjectName(QString::fromUtf8("FileExplorer"));
        FileExplorer->resize(736, 419);
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
"    background-color: #4b4e52;\n"
"    color: #ffffff;\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background-color: #5cb85c;\n"
"}\n"
"\n"
"QStatusBar {\n"
"    background-color: #3c3f41;\n"
"    color: #fff"
                        "fff;\n"
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

        tv_fileExplorer = new QTreeView(FileExplorer);
        tv_fileExplorer->setObjectName(QString::fromUtf8("tv_fileExplorer"));
        tv_fileExplorer->setMinimumSize(QSize(0, 300));
        tv_fileExplorer->viewport()->setProperty("cursor", QVariant(QCursor(Qt::PointingHandCursor)));
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
    } // retranslateUi

};

namespace Ui {
    class FileExplorer: public Ui_FileExplorer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FILEEXPLORER_H
