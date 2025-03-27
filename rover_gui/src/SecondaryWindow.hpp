#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>

class SecondaryWindow : public QMainWindow
{
  public:
    explicit SecondaryWindow();

  private:
    QWidget _centralWidget;
    QVBoxLayout _layout;

    QLabel _tempLabel;
    //Q _navigationWidget;

};

#endif  // SECONDARY_WINDOW_HPP
