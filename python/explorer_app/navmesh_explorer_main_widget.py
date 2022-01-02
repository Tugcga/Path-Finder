from PySide6 import QtWidgets, QtCore, QtGui
import math, random
from explorer_app.utilities import PointTransformer, read_level_data
from explorer_app.simulate_thread import SimulateThread
from pathfinder import PathFinder


class NavmeshExplorerMain(QtWidgets.QWidget):
    def __init__(self):
        super(NavmeshExplorerMain, self).__init__()
        self._is_active = False
        self._simulate_thread = None
        self._left_point = None
        self._right_point = None
        self._tfm = None
        # self._path = None
        self._path_finder = None
        self._layout = QtWidgets.QVBoxLayout(self)
        self._label_non_init = QtWidgets.QLabel("No navmesh data. Load level.")
        self._label_non_init.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self._label_non_init.setAlignment(QtCore.Qt.AlignCenter)
        self._layout.addWidget(self._label_non_init)
        self._thread_manager = QtCore.QThreadPool()
        self._agent_positions = []
        self._agent_paths = []

    def set_navmesh_text_data(self, file_path):
        self._left_point = None
        self._right_point = None
        self._tfm = None
        # self._path = None

        self._navmesh_vertices, self._navmesh_polygons = read_level_data(file_path)
        # init navmesh by this data
        self._path_finder = PathFinder(self._navmesh_vertices, self._navmesh_polygons, time_horizon=0.5, time_horizon_obst=0.1, move_agents=True)
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
            self.start_simulation()
            self.repaint()
        else:
            self._is_active = False
            self.stop_simulation()
            self.repaint()

    def start_simulation(self):
        if self._simulate_thread is not None:
            self.stop_simulation()
        self._simulate_thread = SimulateThread(self._path_finder, self, delta_time=1.0/30.0)
        self._simulate_thread.start()

    def stop_simulation(self):
        self._simulate_thread.terminate()
        self._simulate_thread.wait()
        self._simulate_thread = None

    def terminate_threads(self):
        if self._simulate_thread is not None:
            self._simulate_thread.terminate()
            self._simulate_thread.wait()

    def set_colors(self, background, back_dark_lines, back_light_lines, poly_border, poly_int, path, agents, point_size, path_size, dark_grid_size, light_grid_size):
        # all colors are 4-tuples
        self._background = QtGui.QColor(*background)
        self._back_dark_lines = QtGui.QColor(*back_dark_lines)
        self._back_light_lines = QtGui.QColor(*back_light_lines)
        self._poly_border = QtGui.QColor(*poly_border)
        self._poly_int = QtGui.QColor(*poly_int)
        self._path_color = QtGui.QColor(*path)
        self._agents_color = QtGui.QColor(*agents)
        agents_center = [v for v in agents]
        agents_center[3] = 255
        self._agents_center_color = QtGui.QColor(*agents_center)
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
            is_middle = event.button() == QtCore.Qt.MiddleButton
            if is_left:
                # we should convert canvas point coordinates to graph coordinates
                self._left_point = self._tfm.transform_inverse((x, y))
                # add agent to left click point
                self._path_finder.add_agent((self._left_point[0], 0.0, self._left_point[1]), self._path_finder.get_default_agent_radius(), 2.0)
            if is_right:
                self._right_point = self._tfm.transform_inverse((x, y))

            if self._left_point is not None and self._right_point is not None:
                # next we should calculate path for each agent and set it
                for a_id in self._path_finder.get_agents_id():
                    self._path_finder.set_agent_destination(a_id, (self._right_point[0], 0.0, self._right_point[1]))

            if is_left or is_right:
                self.repaint()

            if is_middle:
                agents_ids = self._path_finder.get_agents_id()
                # select the random one
                if len(agents_ids) > 0:
                    v = agents_ids[random.randint(0, len(agents_ids)) - 1]
                    self._path_finder.delete_agent(v)

    def update_agents(self, positions, paths, activity):
        '''positions is array of 2-tuples
        paths is array of arrays of 2-tuples
        '''
        self._agent_positions = positions
        self._agent_paths = paths
        self._agent_activity = activity
        self.update()

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
            # draw each agents path
            painter.setPen(QtGui.QPen(self._path_color, self._path_size, QtGui.Qt.SolidLine, QtGui.Qt.RoundCap, QtGui.Qt.RoundJoin))
            for a in range(len(self._agent_paths)):
                path = self._agent_paths[a]
                if self._agent_activity[a] and len(path) > 1:
                    canvas_path = []
                    for p in path:
                        c_p = self._tfm.transform(p)
                        canvas_path.append(QtCore.QPointF(c_p[0], c_p[1]))
                    painter.drawPolyline(QtGui.QPolygonF(canvas_path))
            # also draw points
            painter.setPen(QtGui.QPen(self._path_color, self._point_size, QtGui.Qt.SolidLine, QtGui.Qt.RoundCap, QtGui.Qt.RoundJoin))
            painter.setBrush(QtGui.QBrush(self._path_color))
            if self._right_point is not None:
                painter.drawEllipse(QtCore.QPoint(*self._tfm.transform(self._right_point)), 3, 3)

            # draw agents
            draw_radius = self._path_finder.get_default_agent_radius()
            c1 = self._tfm.transform((0.0, 0.0))
            c2 = self._tfm.transform((0.0 + draw_radius, 0.0))
            draw_radius_canvas = int(math.sqrt((c1[0] - c2[0])**2 + (c1[1] - c2[1])**2))
            painter.setPen(QtGui.QPen(self._agents_color))
            painter.setBrush(QtGui.QBrush(self._agents_color))
            for a_position in self._agent_positions:
                painter.drawEllipse(QtCore.QPoint(*self._tfm.transform(a_position)), draw_radius_canvas, draw_radius_canvas)
            # and also agent centers
            painter.setBrush(QtGui.QBrush(self._agents_center_color))
            for a_position in self._agent_positions:
                painter.drawEllipse(QtCore.QPoint(*self._tfm.transform(a_position)), 3, 3)
