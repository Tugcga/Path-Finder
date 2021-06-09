from PySide6 import QtWidgets, QtCore, QtGui


class ColorWidget(QtWidgets.QWidget):
    def __init__(self, color, change_callback):  # color is a 4-tuple
        super(ColorWidget, self).__init__()
        self._color = color
        self._qt_color = QtGui.QColor(*color)
        self._update = change_callback
        self.setMinimumWidth(50)

    def paintEvent(self, event):
        size = self.size()
        painter = QtGui.QPainter(self)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(self._qt_color)
        painter.drawRect(0, 0, size.width(), size.height())

    def mousePressEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        if x > 0 and x < self.size().width() and y > 0 and y < self.size().height():
            color_dialog = QtWidgets.QColorDialog()
            new_color = color_dialog.getColor(self._qt_color, options=QtWidgets.QColorDialog.ShowAlphaChannel)
            if new_color.isValid():
                self._color = (new_color.red(), new_color.green(), new_color.blue(), new_color.alpha())
                self._qt_color = QtGui.QColor(*self._color)
                self._update(self._color)
                self.repaint()


class SelectColorsWidget(QtWidgets.QWidget):
    def __init__(self, set_callback):
        super(SelectColorsWidget, self).__init__()
        self.m_set_colors = set_callback
        self._layout = QtWidgets.QFormLayout(self)
        self._colors = {"background": (171, 168, 166, 255),
                        "back_dark_lines": (151, 148, 146, 255),
                        "back_light_lines": (161, 158, 156, 255),
                        "poly_border": (0, 125, 0, 64),
                        "poly_int": (102, 198, 102, 120),
                        "path": (247, 220, 112, 255)}
        self._sizes = {"point_size": 3, "path_size": 3, "dark_grid_size": 80, "light_grid_size": 20}

        self._layout.addRow(QtWidgets.QLabel("Background"), ColorWidget(self._colors["background"], self.m_update_background))
        self._layout.addRow(QtWidgets.QLabel("Background Dark Grid"), ColorWidget(self._colors["back_dark_lines"], self.m_update_back_dark_lines))
        self._layout.addRow(QtWidgets.QLabel("Background Light Grid"), ColorWidget(self._colors["back_light_lines"], self.m_update_back_light_lines))
        self._layout.addRow(QtWidgets.QLabel("Polygon Borders"), ColorWidget(self._colors["poly_border"], self.m_update_poly_border))
        self._layout.addRow(QtWidgets.QLabel("Polygon Interior"), ColorWidget(self._colors["poly_int"], self.m_update_poly_int))
        self._layout.addRow(QtWidgets.QLabel("Points and Path"), ColorWidget(self._colors["path"], self.m_update_path))

        self.update_colors()

    def update_colors(self):
        self.m_set_colors(background=self._colors["background"],
                          back_dark_lines=self._colors["back_dark_lines"],
                          back_light_lines=self._colors["back_light_lines"],
                          poly_border=self._colors["poly_border"],
                          poly_int=self._colors["poly_int"],
                          path=self._colors["path"],
                          point_size=self._sizes["point_size"],
                          path_size=self._sizes["path_size"],
                          dark_grid_size=self._sizes["dark_grid_size"],
                          light_grid_size=self._sizes["light_grid_size"])

    def m_update_background(self, color):
        self._colors["background"] = color
        self.update_colors()

    def m_update_back_dark_lines(self, color):
        self._colors["back_dark_lines"] = color
        self.update_colors()

    def m_update_back_light_lines(self, color):
        self._colors["back_light_lines"] = color
        self.update_colors()

    def m_update_poly_border(self, color):
        self._colors["poly_border"] = color
        self.update_colors()

    def m_update_poly_int(self, color):
        self._colors["poly_int"] = color
        self.update_colors()

    def m_update_path(self, color):
        self._colors["path"] = color
        self.update_colors()
