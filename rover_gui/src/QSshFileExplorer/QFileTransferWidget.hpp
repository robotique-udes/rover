#include <QGridLayout>
#include <QSplitter>

#include "QSshFileExplorerWidget.hpp"

class QFileTransferWidget : public QWidget
{
  public:
    QFileTransferWidget(QWidget* parent_);
    ~QFileTransferWidget();

  private:
    QGridLayout _mainLayout;
    QSplitter _splitter;
    QSshFileExplorerWidget _localFileSystem;
    QSshFileExplorerWidget _roverFileSystem;
};
