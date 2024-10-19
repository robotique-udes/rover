#include <QGridLayout>

#include "QSshFileExplorerWidget.hpp"

class QFileTransferWidget : public QWidget
{
  public:
    QFileTransferWidget(QWidget* parent_);
    ~QFileTransferWidget();

  private:
    QGridLayout _mainLayout;
    QSshFileExplorerWidget _localFileSystem;
    QSshFileExplorerWidget _roverFileSystem;
};
