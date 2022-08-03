fn main() {
    let vertices = vec![0.0, 0.0, 0.0, 
                        4.0, 0.0, 0.0,
                        4.0, 0.0, 4.0,
                        0.0, 0.0, 4.0];
    let triangles = vec![0, 1, 2,
                         0, 2, 3];
    let pf = pathfinder::Pathfinder::new(&vertices, &triangles);
    let p = pf.search_path(0.0, 0.0, 0.0, 3.0, 0.0, 3.0);
}