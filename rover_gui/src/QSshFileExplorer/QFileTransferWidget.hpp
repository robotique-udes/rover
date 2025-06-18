#include <QGridLayout>
#include <QSplitter>

#include "QSshFileExplorerWidget.hpp"

class QFileTransferWidget : public QWidget
{
  public:
    explicit QFileTransferWidget(QWidget* parent_);

  private:
    QGridLayout _mainLayout;
    QSplitter _splitter;
    QSshFileExplorerWidget _localFileSystem;
    QSshFileExplorerWidget _roverFileSystem;
};
