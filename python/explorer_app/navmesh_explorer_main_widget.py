from PySide6 import QtWidgets, QtCore, QtGui
from explorer_app.utilities import PointTransformer, read_level_data
import navmesh as nm


class NavmeshExplorerMain(QtWidgets.QWidget):
    def __init__(self):
        super(NavmeshExplorerMain, self).__init__()
        self._is_active = False
        self._left_point = None
        self._right_point = None
        self._tfm = None
        self._path = None
        self._navmesh = None
        self._layout = QtWidgets.QVBoxLayout(self)
        self._label_non_init = QtWidgets.QLabel("No navmesh data. Load level.")
        self._label_non_init.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self._label_non_init.setAlignment(QtCore.Qt.AlignCenter)
        self._layout.addWidget(self._label_non_init)

    def set_navmesh_text_data(self, file_path):
        self._left_point = None
        self._right_point = None
        self._tfm = None
        self._path = None

        self._navmesh_vertices, self._navmesh_polygons = read_level_data(file_path)
        # init navmesh by this data
        self._navmesh = nm.Navmesh(self._navmesh_vertices, self._navmesh_polygons)
        if len(self._navmesh_polygons) > 0:
            # find center of the points and it bounding box
            self._navmesh_x_min = float("inf")
            self._navmesh_y_min = float("inf")
            self._navmesh_x_max = -float("inf")
            self._navmesh_y_max = -float("inf")
            for i in range(len(self._navmesh_vertices)):
                x = self._navmesh_vertices[i][0]
                y = self._navmesh_vertices[i][2]
                if x < self._navmesh_x_min:
                    self._navmesh_x_min = x
                if x > self._navmesh_x_max:
                    self._navmesh_x_max = x
                if y < self._navmesh_y_min:
                    self._navmesh_y_min = y
                if y > self._navmesh_y_max:
                    self._navmesh_y_max = y
            graph_width = self._navmesh_x_max - self._navmesh_x_min
            graph_height = self._navmesh_y_max - self._navmesh_y_min

            self._navmesh_x_min -= graph_width / 10.0
            self._navmesh_x_max += graph_width / 10.0

            self._navmesh_y_min -= graph_height / 10.0
            self._navmesh_y_max += graph_height / 10.0

            self._graph_aspect = (self._navmesh_x_max - self._navmesh_x_min) / (self._navmesh_y_max - self._navmesh_y_min)
            self._label_non_init.setVisible(False)
            self._is_active = True
            self.repaint()
        else:
            self._is_active = False
            self.repaint()

    def set_colors(self, background, back_dark_lines, back_light_lines, poly_border, poly_int, path, point_size, path_size, dark_grid_size, light_grid_size):
        # all colors are 4-tuples
        self._background = QtGui.QColor(*background)
        self._back_dark_lines = QtGui.QColor(*back_dark_lines)
        self._back_light_lines = QtGui.QColor(*back_light_lines)
        self._poly_border = QtGui.QColor(*poly_border)
        self._poly_int = QtGui.QColor(*poly_int)
        self._path_color = QtGui.QColor(*path)
        self._point_size = point_size
        self._path_size = path_size
        self._dark_grid_size = dark_grid_size
        self._light_grid_size = light_grid_size

        self.repaint()

    def mousePressEvent(self, event):
        if self._is_active and self._tfm is not None:
            x = event.x()
            y = event.y()
            is_left = event.button() == QtCore.Qt.LeftButton
            is_right = event.button() == QtCore.Qt.RightButton
            if is_left:
                # we should convert canvas point coordinates to graph coordinates
                self._left_point = self._tfm.transform_inverse((x, y))
            if is_right:
                self._right_point = self._tfm.transform_inverse((x, y))

            if self._left_point is not None and self._right_point is not None:
                # calculate the path
                path = self._navmesh.serach_path((self._left_point[0], 0.0, self._left_point[1]), (self._right_point[0], 0.0, self._right_point[1]))
                self._path = [(v[0], v[2]) for v in path]  # convert to 2d

            if is_left or is_right:
                self.repaint()

    def draw_grid(self, painter, canvas_width, canvas_height, grid_size):
        lines = []
        # vertical lines
        steps = canvas_width // grid_size
        for i in range(steps):
            c = (i - steps // 2) * grid_size + canvas_width // 2
            lines.append(QtCore.QLineF(c, 0, c, canvas_height))
        # horizontal lines
        steps = canvas_height // grid_size
        for i in range(steps):
            c = (i - steps // 2) * grid_size + canvas_height // 2
            lines.append(QtCore.QLineF(0, c, canvas_width, c))
        painter.drawLines(lines)

    def paintEvent(self, event):
        # draw background
        canvas_width = self.size().width()
        canvas_height = self.size().height()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(0, 0, canvas_width, canvas_height, self._background)
        # draw the grid
        # draw from canvas center
        painter.setPen(self._back_light_lines)
        self.draw_grid(painter, canvas_width, canvas_height, self._light_grid_size)
        painter.setPen(self._back_dark_lines)
        self.draw_grid(painter, canvas_width, canvas_height, self._dark_grid_size)
        if self._is_active:
            if self._graph_aspect > canvas_width / canvas_height:
                # use x size for drawings, add spaces to y
                x_shift = 0
                y_shift = (canvas_height - canvas_width / self._graph_aspect) / 2
            else:
                # add blank spaces to x
                y_shift = 0
                x_shift = (canvas_width - canvas_height * self._graph_aspect) / 2

            # we should transform points from float scale to canvas scale and shift it to the center
            canvas_points = []
            self._tfm = PointTransformer((self._navmesh_x_min, self._navmesh_y_min),
                                         (self._navmesh_x_max, self._navmesh_y_max),
                                         (x_shift, y_shift),
                                         self._graph_aspect,
                                         canvas_width, canvas_height)
            for i in range(len(self._navmesh_vertices)):
                canvas_points.append(self._tfm.transform((self._navmesh_vertices[i][0], self._navmesh_vertices[i][2])))
            # draw triangles
            painter.setPen(self._poly_border)
            painter.setBrush(QtGui.QBrush(self._poly_int))
            for i in range(len(self._navmesh_polygons)):
                polyline_points = []
                for v in self._navmesh_polygons[i]:
                    v_a = canvas_points[v]
                    polyline_points.append(QtCore.QPointF(v_a[0], v_a[1]))
                polyline_points.append(polyline_points[0])
                polyline = QtGui.QPolygonF(polyline_points)
                painter.drawPolygon(polyline)
            # the path
            if self._path is not None:
                painter.setPen(QtGui.QPen(self._path_color, self._path_size, QtGui.Qt.SolidLine, QtGui.Qt.RoundCap, QtGui.Qt.RoundJoin))
                canvas_path = []
                for p in self._path:
                    c_p = self._tfm.transform(p)
                    canvas_path.append(QtCore.QPointF(c_p[0], c_p[1]))
                painter.drawPolyline(QtGui.QPolygonF(canvas_path))
            # also draw points
            painter.setPen(QtGui.QPen(self._path_color, self._point_size, QtGui.Qt.SolidLine, QtGui.Qt.RoundCap, QtGui.Qt.RoundJoin))
            painter.setBrush(QtGui.QBrush(self._path_color))
            if self._left_point is not None:
                painter.drawEllipse(QtCore.QPoint(*self._tfm.transform(self._left_point)), 3, 3)
            if self._right_point is not None:
                painter.drawEllipse(QtCore.QPoint(*self._tfm.transform(self._right_point)), 3, 3)
