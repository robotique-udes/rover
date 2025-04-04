#ifndef STREAM_DIALOG_HPP
#define STREAM_DIALOG_HPP

#include <QDialog>
#include <QLineEdit>
#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QPushButton>

// Dialog for adding or editing streams
class StreamDialog : public QDialog
{
    Q_OBJECT
    
public:
    StreamDialog(const QString& title, const QString& currentName = "", 
                 const QString& currentUrl = "", QWidget* parent = nullptr)
        : QDialog(parent)
    {
        setWindowTitle(title);
        
        QVBoxLayout* layout = new QVBoxLayout(this);
        
        // Create form layout
        QFormLayout* formLayout = new QFormLayout();
        
        _nameEdit = new QLineEdit(currentName);
        formLayout->addRow("Stream Name:", _nameEdit);
        
        _urlEdit = new QLineEdit(currentUrl);
        _urlEdit->setPlaceholderText("rtsp://example.com/stream");
        formLayout->addRow("RTSP URL:", _urlEdit);
        
        // Add to main layout
        layout->addLayout(formLayout);
        
        // Add buttons
        QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
        
        layout->addWidget(buttonBox);
        
        // Set minimum width
        setMinimumWidth(400);
    }
    
    QString getStreamName() const { return _nameEdit->text(); }
    QString getStreamUrl() const { return _urlEdit->text(); }
    
private:
    QLineEdit* _nameEdit;
    QLineEdit* _urlEdit;
};

#endif // STREAM_DIALOG_HPP