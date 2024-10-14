#include <QGridLayout>

#include "QSshFileExplorerWidget.hpp"

class QFileTransferWidget : public QWidget
{
  public:
    QFileTransferWidget(QWidget* parent_): _mainLayout(this), _roverFileSystem(parent_)
    {
        _roverFileSystem.getUI().le_user->setText("rover");
        _roverFileSystem.getUI().le_hostIP->setText("192.168.144.20");
        _roverFileSystem.getUI().cb_showHiddenFile->setChecked(true);
        
        _mainLayout.addWidget(&_roverFileSystem, 0, 0, 1, 1);
    }

    ~QFileTransferWidget() {}

  private:
    QGridLayout _mainLayout;
    QSshFileExplorerWidget _roverFileSystem;
};
