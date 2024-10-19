#pragma once

#include <QString>

// Here some style presets can be added for different app themes
constexpr const char* STYLE_DARK_MODE = R"(
QWidget {
     background-color: #2e2e2e;
    color: #ffffff;
}

QMenuBar {
    background-color: #3c3f41;
    color: #ffffff;
}

QMenu {
    background-color: #3c3f41;
    color: #ffffff;
}

QMenu::item {
    padding: 5px 30px;
}

QMenu::item:selected {
    background-color: #4a4e54;
}

QPushButton {
    background-color: #3c3f41;
    border: 1px solid #4b4e52;
    border-radius: 5px;
    padding: 5px 10px;
}

QPushButton:hover {
    background-color: #4d4d4d;
}

QLineEdit {
    background-color: #3c3f41;
    color: #ffffff;
    border: 1px solid #4b4e52;
    border-radius: 5px;
    padding: 5px;
}

QTextEdit {
    background-color: #3c3f41;
    color: #ffffff;
    border: 1px solid #4b4e52;
    border-radius: 5px;
}

QLabel {
    color: #ffffff;
}

QScrollBar:vertical {
    background: #2e2e2e;
    width: 10px;
}

QScrollBar::handle:vertical {
    background: #4b4e52;
    border-radius: 5px;
}

QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical {
    background: none;
}

QCheckBox {
    color: #ffffff;
}

QRadioButton {
    color: #ffffff;
}

QRadioButton::indicator {
    background-color: #3c3f41;
    border: 1px solid #4b4e52;
}

QRadioButton::indicator:checked {
    background-color: #4d4d4d;
}

QTabWidget::pane {
    background-color: #2e2e2e;
}

QTabBar::tab {
    background-color: #3c3f41;
    color: #ffffff;
    padding: 10px;
}

QTabBar::tab:selected {
    background-color: #4d4d4d;
}

QProgressBar {
    background-color: #4b4e52;
    color: #ffffff;
}

QProgressBar::chunk {
    background-color: #5cb85c;
}

QStatusBar {
    background-color: #3c3f41;
    color: #ffffff;
}
)";

constexpr const char* lightMode = "";
