\defgroup mapper Mapper \ingroup mobility

The \ref mapper node is responsible for maintaining a representation of the environment, which is in turn used by the \ref planner node for planning segments. Internally, the \ref mapper stores a collection of keep-in and keep-out zones, as well as a map of the environment, which it builds by incrementally fusing measurements from \picoflexx sensor(s). Ultimately, both representations are reduced by the \ref mapper to a collection of free-space polygons, in which the \ref planner searches for a suitable path.

Observation: as of now, transformation of a map to free-space polygons is not yet implemented.

The mapper node can be divided in the following tasks:

* `Trajectory Validator` - Validates planned trajectories based on keep-in and keep-out zones.
* `Octomapper` - Maps Astrobee's surroundings using an octomap (octree-based map);
* `Sentinel` - Checks for imminent collisions and notifies the controller if an imminent collision has been detected;

## Trajectory Validator

The keep-in and keep-out zones describe safe and unsafe areas for flight respectively. Each zone is a cuboid in 3-space, and is fully-defined by the coordinates of two diagonally opposite vertices. The union of all keep-in zones minus the union of all keep-out zones describes the free space in which safe flight can occur. The default zones are provided as JSON formatted files with suffix `.json` in the `zones` directory of the \ref astrobee folder.

An example of a keep-in zone JSON file might look like this:

	{
	  "sequence" : [ [ -1.0, -1.0, -3.0, 1.0, 1.0, 3.0 ]  ] ,
	  "dateCreated" : "1475516840",	
	  "notes" : "Keep-in zone",
	  "author" : "John Smith",
	  "name" : "GraniteTableWorkspace",
	  "safe" : true
	}

Whereas, a keep-out zone JSON file might contain this:

	{
	  "sequence" : [ [ -1.0, -0.3, -3, -0.6, 1.0, 3.0 ], [0.3,-0.3,-3.0,1.0,0.3,3.0] ] ,
	  "dateCreated" : "1475516840",
	  "notes" : "Keep-out zone",
	  "author" : "John Smith",
	  "name" : "GraniteTableDockAndClutter",
	  "safe" : false
	}

Note that the "sequence" field takes an array of 6-vectors, and not just a single 6-vector. Each element of this array represents a zone, with each vector denoting the two coordinates that fully-define the cuboid. A consequence of this design choice is that multiple keep-in or keep-out zones can be specified in a single file (but you cannot mix keep-outs and keep-ins in one file).

After loading and parsing these JSON files, the resulting data structure is serialized into a binary file called `0.bin`. The digit zero tells the \ref mapper system that the binary structure contains the default set of zones. At any point an operator can upload a new set of zones using the `SetZones` service call on the ROS namespace `~/mob/mapper/set_zones`. Calling this service will result in the creation of a new file `%lu.bin` in the `zones` file, where `%lu` is a unsigned long Unix timestamp for the zones. When the \ref mapper node is next started, it will search for the `*.bin` file with the latest timestamp, and load it by default.

Please refer to the definition of \ref ff_msgs_SetZones for more information on how to update zones using a ROS service call.

The \ref mapper node publishes [visualization_msgs::MarkerArrays](http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/MarkerArray.html) on the namespace `~/mob/mapper/zones`. When listened to in rviz, these marker arrays render keep-in and keep-out zones as semi-transparent green and red cuboids. The two example zones should be rendered as shown below:

![alt text](../images/mobility/zones.png "How the RVIZ user interface draws zones")

## Octomapper

The Octomapper portion of the \ref mapper creates a 3D occupancy map of the environment that maps known areas probabilistically. The Octomapperr builds a representation of the environment (clutter topology) using measurements from the \ref picoflexx depth sensor, using an Octomap. Mapping is done in the world frame, and the TF2 library is used to transform from the sensor frame to the world frame based on dynamic transforms published by the EKF, and static transforms published by \ref framestore.
It is based on the following work:

```
@article{hornung2013octomap,
  title={OctoMap: An efficient probabilistic 3D mapping framework based on octrees},
  author={Hornung, Armin and Wurm, Kai M and Bennewitz, Maren and Stachniss, Cyrill and Burgard, Wolfram},
  journal={Autonomous Robots},
  volume={34},
  number={3},
  pages={189--206},
  year={2013},
  publisher={Springer}
}
```

![alt text](../images/mobility/iss_sim_360_octomap.png "An example of an Octomap obtained after an Astrobee 360-degree rotation around its z-axis. The rainbow color scheme displays different heights.")

The Octomap library stores data in an Octree data structure. This implies in efficient memory allocation, since only known
areas are stored in memory. Beyond that, this library enhances memory allocation efficiency by exploiting tree pruning methods.

In addition to that, Octomap constantly updates occupancy probabilities for every voxel (3D pixel) in the tree. Therefore, at every new point cloud measurement, the occupancy probabilities are updated through a Bayesian update.

Most of the code used in the \ref mapper uses the API provided in the Octomap library. However, some extra functionality was added:

* `Map inflation (C-expansion)` - This is important for collision checking and path planning.
* `Fading memory` - Since the main goal of the map is to be used for collision detection (and possibly local path planning),
there is no need to store old information in the map. Hence, the \ref mapper has a fading memory method that reduces map confidence as time goes by. When a voxel confidence reaches a certain threshold, it is deallocated from the map.

The Octomapper subscribes to:

* `point cloud data` - The map is updated based on point cloud data.
* `tf information` - In order to determine the location of the point cloud in the world frame, the mapper needs to rotate
the point cloud data from camera frame to world frame. This is done by threads that listen to tf updates.

The Octomapper publishes ROS visualization_markers, which can be used for human visual inspection of the map using RVIZ.
It should be mentioned that these visualization_markers are only published if there is a subscriber to it. Hence, if nobody
requests to see this in RVIZ, this won't be published. The importance of this is to avoid high-bandwidth information being
sent across the network. Map visualization can be seen through the topics:

* `mapper/obstacle_markers` - Visualization of the occupied voxels in the non-inflated map.
* `mapper/free_space_markers` - Visualization of the free voxels in the non-inflated map.
* `mapper/inflated_obstacle_markers` - Visualization of the occupied voxels in the inflated map.
* `mapper/inflated_free_space_markers` - Visualization of the free voxels in the inflated map.
* `mapper/frustum_markers` - visualization of the depth sensor frustum. This is useful for visualizing the sensor's field of view.

The Octomapper takes the following paramers as inputs (defined in mapper.config):

* `map_resolution (meters)` - Size of a voxel in the Octomap.
* `max_range (meters)` - Maximum reliable range of the depth camera.
* `min_range (meters)` - Minimum reliable range of the depth camera.
* `memory_time (seconds)` - How long the octomap remembers the fading memory map. It remembers forever when this variable is <= 0.
* `inflate_radius (meters)` - Radial inflation size of the octomap.
* `cam_fov (radians)` - Camera horizontal field-of-view. The octomap will only update the map in the volume within the fov of the depth cam.
* `cam_aspect_ratio` - Depth camera's width divided by height. Used for calculating the camera's frustum.
* `occupancy_threshold` - Probability threshold above which a node is considered occupied.
* `probability_hit` - Probability that a measured obstacle is an obstacle (detection probability).
* `probability_miss` - Probability that a measured free area is an obstacle (false alarm).
* `clamping_threshold_min` - Minimum probability assigned as occupancy for a node. This should not be set to 0, because the probability of a free node being occupied might asymptotically converge to 0 as more measurements come in. If, however, this node becomes occupied sometime in the future, the Bayesian update would not trust the sensor anymore. 
* `clamping_threshold_max` - Maximum probability assigned as occupancy for a node. This should not be set to 1, due to the considerations above.

## Sentinel
