// Ui files
#include "ui_file_transfer_widget.h"

// Custom objects
#include "q_ssh_file_explorer.hpp"

class QFileTransferWidget : public QWidget
{
  public:
    QFileTransferWidget()
    {
        ui.setupUi(this);
        pRoverFileSystem = new QSshFileExplorer(*ui.tv_rover);
    }

    ~QFileTransferWidget() {}

  private:
    Ui::FileTransfer ui;
    QSshFileExplorer* pRoverFileSystem;
};
