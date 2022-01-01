from PySide6 import QtWidgets, QtGui
from explorer_app.select_colors_widget import SelectColorsWidget  # type: ignore
from explorer_app.navmesh_explorer_main_widget import NavmeshExplorerMain  # type: ignore


class NavmeshExplorerApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(NavmeshExplorerApp, self).__init__()
        self._main = NavmeshExplorerMain()
        self.setCentralWidget(self._main)
        self.setWindowTitle("Navmesh Explorer")

        menubar = self.menuBar()
        file_menu = menubar.addMenu("&File")
        options_menu = menubar.addMenu("&Options")

        # exit action
        exit_action = QtGui.QAction('&Exit', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.setStatusTip('Exit application')
        exit_action.triggered.connect(self.close)
        # load action
        load_action = QtGui.QAction('&Load Level', self)
        load_action.setShortcut('Ctrl+L')
        load_action.setStatusTip('Load navmesh data')
        load_action.triggered.connect(self.m_load_action_call)

        colors_action = QtGui.QAction('&Colors Window', self)
        colors_action.setShortcut('Ctrl+C')
        colors_action.setStatusTip('Define colors')
        colors_action.triggered.connect(self.m_colors_action_call)

        # connect actions to menu
        file_menu.addAction(load_action)
        file_menu.addAction(exit_action)

        options_menu.addAction(colors_action)

        # create hiden docs
        # color selection
        self._colors_doc = QtWidgets.QDockWidget("Colors", self)
        self._colors_doc.setWidget(SelectColorsWidget(self._main.set_colors))
        self._colors_doc.setFloating(1)
        self._colors_doc.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        self._colors_doc.setFeatures(QtWidgets.QDockWidget.DockWidgetClosable)
        self._colors_doc.hide()

    def m_load_action_call(self):
        dialog = QtWidgets.QFileDialog(self)
        dialog.setFileMode(QtWidgets.QFileDialog.ExistingFiles)
        dialog.setNameFilter("Text (*.txt)")
        dialog.setViewMode(QtWidgets.QFileDialog.Detail)
        if dialog.exec():
            selected_files = dialog.selectedFiles()
            if len(selected_files) > 0:
                self._main.set_navmesh_text_data(selected_files[0])

    def m_colors_action_call(self):
        if self._colors_doc.isVisible() is False:
            pos = self._colors_doc.pos()
            self._colors_doc.show()
            self._colors_doc.move(pos)

    def closeEvent(self, event):
        self._main.terminate_threads()


if __name__ == "__main__":
    app = QtWidgets.QApplication()
    main = NavmeshExplorerApp()
    main.resize(800, 600)
    main.show()
    app.exec()
