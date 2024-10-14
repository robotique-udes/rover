#include <QGridLayout>

#include "QSshFileExplorerWidget.hpp"

class QFileTransferWidget : public QWidget
{
  public:
    QFileTransferWidget(QWidget* parent_): _mainLayout(this), _localFileSystem("phil", "localhost", "/home/phil", parent_)
    {
        _localFileSystem.getUI().cb_showHiddenFile->setChecked(false);
        
        _mainLayout.addWidget(&_localFileSystem, 0, 0, 1, 1);
    }

    ~QFileTransferWidget() {}

  private:
    QGridLayout _mainLayout;
    QSshFileExplorerWidget _localFileSystem;
};
