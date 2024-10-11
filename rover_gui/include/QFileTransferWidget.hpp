#include <QGridLayout>

#include "QSshFileExplorer/QSshFileExplorerWidget.hpp"

class QFileTransferWidget : public QWidget
{
  public:
    QFileTransferWidget(QWidget* parent_): _mainLayout(this), _roverFileSystem(parent_)
    {
        _mainLayout.addWidget(&_roverFileSystem, 0, 0, 1, 1);
    }

    ~QFileTransferWidget() {}

  private:
    QGridLayout _mainLayout;
    QSshFileExplorerWidget _roverFileSystem;
};
