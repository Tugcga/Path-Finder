class PointTransformer():
    def __init__(self, min_corner, max_corner, shift, aspect, width, height):
        self._min_corner = min_corner  # this is tuple
        self._max_corner = max_corner  # this is tuple
        self._shift = shift  # and this tuple too
        self._aspect = aspect
        self._width = width
        self._height = height

        self._x_coeff = self._height * self._aspect if self._aspect <= self._width / self._height else self._width
        self._y_coeff = self._height if self._aspect <= self._width / self._height else self._width / self._aspect

    def transform(self, point):
        return ((point[0] - self._min_corner[0]) * self._x_coeff / (self._max_corner[0] - self._min_corner[0]) + self._shift[0],
                (point[1] - self._min_corner[1]) * self._y_coeff / (self._max_corner[1] - self._min_corner[1]) + self._shift[1])

    def transform_inverse(self, point):
        return ((point[0] - self._shift[0]) * (self._max_corner[0] - self._min_corner[0]) / self._x_coeff + self._min_corner[0],
                (point[1] - self._shift[1]) * (self._max_corner[1] - self._min_corner[1]) / self._y_coeff + self._min_corner[1])


def read_level_data(file_path):
    with open(file_path, "r") as file:
        file_text = file.read()
        lines = file_text.split("\n")
        if len(lines) == 3:  # polygons data
            vertices_raw = [float(v) for v in lines[0].split(" ")]
            polygons_raw = [int(v) for v in lines[1].split(" ")]
            sizes = [int(v) for v in lines[2].split(" ")]

            verts_count = len(vertices_raw) // 3
            vertices = []
            for i in range(verts_count):
                vertices.append((vertices_raw[3*i], vertices_raw[3*i + 1], vertices_raw[3*i + 2]))

            i = 0
            polygons = []
            for s in sizes:
                polygon = []
                for j in range(s):
                    polygon.append(polygons_raw[i])
                    i += 1
                polygons.append(polygon)
            return vertices, polygons
        else:  # may be triangles data
            vertices_raw = [float(v) for v in lines[0].split(" ")]
            triangles_raw = [int(v) for v in lines[1].split(" ")]
            verts_count = len(vertices_raw) // 3
            vertices = []
            for i in range(verts_count):
                vertices.append((vertices_raw[3*i], vertices_raw[3*i + 1], vertices_raw[3*i + 2]))

            tris_count = len(triangles_raw) // 3
            polygons = []
            for i in range(tris_count):
                polygons.append([triangles_raw[3*i], triangles_raw[3*i + 1], triangles_raw[3*i + 2]])
            return vertices, polygons
