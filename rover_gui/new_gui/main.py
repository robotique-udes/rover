#https://www.youtube.com/watch?v=jWxNfb7Hng8

from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QTabWidget, QToolBox
from PyQt5 import uic

from pages_functions.home import Home
from pages_functions.dashboard import Dashboard
from pages_functions.navigation import Navigation

from static import resource_rc

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        uic.loadUi("ui/main_window.ui", self)

        self.pb_home : QPushButton
        self.pb_dashboard : QPushButton
        self.pb_navigation : QPushButton
        self.tab_widget : QTabWidget
        self.tool_box : QToolBox

        self.pb_home = self.findChild(QPushButton, "pb_home")
        self.pb_dashboard = self.findChild(QPushButton, "pb_dashboard")
        self.pb_navigation = self.findChild(QPushButton, "pb_navigation")
        self.tab_widget = self.findChild(QTabWidget, "tab_widget")
        self.tool_box = self.findChild(QToolBox, "tool_box")


        self.menu_btns_dict = {
            self.pb_home: Home,
            self.pb_dashboard: Dashboard,
            self.pb_navigation: Navigation
        }

        self.show_home_window()

        self.tab_widget.tabCloseRequested.connect(self.close_tab)

        self.pb_home.clicked.connect(self.show_selected_window)
        self.pb_dashboard.clicked.connect(self.show_selected_window)
        self.pb_navigation.clicked.connect(self.show_selected_window)

    def show_home_window(self):
        result = self.open_tab_flag(self.pb_home.text())
        self.set_btn_checked(self.pb_home)

        if result[0]:
            self.tab_widget.setCurrentIndex(result[1])
        else:
            tab_title = self.pb_home.text()
            curIndex = self.tab_widget.addTab(Home(), tab_title)
            self.tab_widget.setCurrentIndex(curIndex)
            self.tab_widget.setVisible(True)

    def show_selected_window(self):
        button = self.sender()

        result = self.open_tab_flag(button.text())
        self.set_btn_checked(button)

        if(result[0]):
            self.tab_widget.setCurrentIndex(result[1])
        else: 
            tab_title = button.text()
            curIndex = self.tab_widget.addTab(self.menu_btns_dict[button](), tab_title)
            self.tab_widget.setCurrentIndex(curIndex)
            self.tab_widget.setVisible(True)

    def close_tab(self, index):
        self.tab_widget.removeTab(index)
        if self.tab_widget.count() == 0:
            self.tool_box.setCurrentIndex(0)
            self.show_home_window()

    def set_btn_checked(self, btn):
        for button in self.menu_btns_dict.keys():
            if button != btn:
                button.setChecked(False)
            else:
                button.setChecked(True)

    def open_tab_flag(self, btn_text):
        open_tab_count = self.tab_widget.count()

        for i in range(open_tab_count):
            tab_title = self.tab_widget.tabText(i)
            if tab_title == btn_text:
                return True, i
            else:
                continue

        return False,

    
if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())