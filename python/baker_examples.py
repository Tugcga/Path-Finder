from pathfinder import navmesh_baker as nmb
import pathfinder

def example_cube_and_plane():
    # create baker object
    baker = nmb.NavmeshBaker()

    # add geometry
    # plane
    baker.add_geometry([(-4.0, 0.0, -4.0), (-4.0, 0.0, 4.0), (4.0, 0.0, 4.0), (4.0, 0.0, -4.0)],
                       [[0, 1, 2, 3]])
    # cube
    baker.add_geometry([(-1.0, 0.0, -1.0), (-1.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 0.0, -1.0), (-1.0, 1.0, -1.0), (-1.0, 1.0, 1.0), (1.0, 1.0, 1.0), (1.0, 1.0, -1.0)],
                       [[0, 3, 2, 1], [2, 6, 5, 1], [4, 5, 6, 7], [0, 4, 7, 3], [2, 3, 7, 6], [0, 1, 5, 4]])

    # bake with default parameters
    is_bake: bool = baker.bake()
    if is_bake:
        vertices, polygons = baker.get_polygonization()
        # save polygonal description to the binary file
        baker.save_to_binary("cube_and_plane.nm")
    else:
        print("Fail to bake navigation mesh")

def example_stair():
    vertices = [(-6.0, 0.0, -6.0), (-6.0, 0.0, -2.0), (-2.0, 0.0, -6.0), (-2.0, 0.0, -2.0), (-6.0, 2.0, 2.0), (-6.0, 2.0, 6.0), (-2.0, 2.0, 2.0), (-2.0, 2.0, 6.0), (2.0, 4.0, 2.0), (2.0, 4.0, 6.0), (6.0, 4.0, 2.0), (6.0, 4.0, 6.0), (2.0, 6.0, -6.0), (2.0, 6.0, -2.0), (6.0, 6.0, -6.0), (6.0, 6.0, -2.0), (-6.0, 8.0, -6.0), (-6.0, 8.0, -2.0), (-2.0, 8.0, -6.0), (-2.0, 8.0, -2.0), (-6.0, 10.0, 2.0), (-6.0, 10.0, 6.0), (-2.0, 10.0, 2.0), (-2.0, 10.0, 6.0)]
    polygons = [[0, 1, 3, 2], [4, 5, 7, 6], [8, 9, 11, 10], [12, 13, 15, 14], [16, 17, 19, 18], [20, 21, 23, 22], [3, 1, 4, 6], [6, 7, 9, 8], [8, 10, 15, 13], [13, 12, 18, 19], [19, 17, 20, 22]]
    # create baker object
    baker = nmb.NavmeshBaker()
    # add geometry
    baker.add_geometry(vertices, polygons)
    is_bake: bool = baker.bake()
    if is_bake:
        nm_vertices, nm_polygons = baker.get_polygonization()
        print(nm_vertices)
        print(nm_polygons)
    else:
        print("Fail to bake navigation mesh")

def example_planes():
    vertices = [(-4.0, 0.0, -4.0), (-4.0, 0.0, 4.0), (4.0, 0.0, -4.0), (4.0, 0.0, 4.0), (-2.4492935982947064e-16, 8.0, -4.0), (-2.4492935982947064e-16, 8.0, 4.0), (2.4492935982947064e-16, 0.0, -4.0), (2.4492935982947064e-16, 0.0, 4.0)]
    polygons = [[0, 1, 3, 2], [4, 5, 7, 6]]
    baker = nmb.NavmeshBaker()
    baker.add_geometry(vertices, polygons)
    is_bake: bool = baker.bake()
    if is_bake:
        nm_vertices, nm_polygons = baker.get_polygonization()
        print(nm_vertices)
        print(nm_polygons)
    else:
        print("Fail to bake navigation mesh")

if __name__ == "__main__":
    example_cube_and_plane()
    # read saved data from the file and output vertices and polygons
    print(pathfinder.read_from_binary("cube_and_plane.nm"))
