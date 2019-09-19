# PLY Orientation and scale normalization

Normalizes the origin and scale of the input pointcloud so that the origin is at the centroid of the cloud and all points are within 1 unit of the origin. The orientation is then normalized so that the cloud's strongest line is along the Z axis. Since there are 2 possible directions for a line the result is visualized to know which way it went. If the direction is opposite that of the desired, add the `-f` argument and rerun the program. This will flip the output direction.

### Compiling

Run the collowing commands:
```
mkdir build
cd build
cmake ..
make
```

### Runing

Run with:
```
./normalize path/to/input/file.ply path/to/output/file.ply [-f]
```

