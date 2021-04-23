# lanelet map loader package

## usage
put in catkin_ws/src and build it
```
roslaunch map_loader lanelet2_map_loader.launch
```
## osm map file generation 
./Lanelet2/lanelet2_maps/README.md

helpful video tutorial:

* https://www.youtube.com/watch?v=3Yk8b8SB81o
* https://www.youtube.com/watch?v=em6H3DM63DM

## TODO
* parse map metafile (origin) instead of hardcoded in map_loader/src/lanelet2_map_loader/lanelet2_map_loader.cpp#L64
* statif tf publisher with position and orientation shift


