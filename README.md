\defgroup mapper Mapper \ingroup mobility

The mapper node can be divided in the following tasks:

* `Octomapper` - Maps Astrobee's surroundings using an octomap (octree-based map);
* `Sentinel` - Checks for imminent collisions and notifies the controller if an imminent collision has been detected;
* `Trajectory Validator` - Validates planned trajectories based on keep-in / keep-out zones (These volumes define areas 
that Astrobee can / cannot have access to).

## Octomapper

The Octomapper portion of this nodelet creates a 3D occupancy map of the environment that maps known areas probabilistically. 
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

The Octomap library stores data in an Octree data structure. This implies in efficient memory allocation, since only known
areas are stored in memory. Beyond that, this library enhances memory allocation efficiency by exploiting tree pruning methods.

In addition to that, Octomap constantly updates occupancy probabilities for every voxel (3D pixel) in the tree. Therefore, at every new
point cloud measurement, the occupancy probabilities are updated through a Bayesian update.

Most of the code used in this nodelet uses the API provided in the Octomap library. However, some extra functionality was added:

* `Map inflation (C-expansion)` - This is important for collision checking and path planning.
* `Fading memory` - Since the main goal of the map is to be used for collision detection (and possibly local path planning),
there is no need to store old information in the map. Hence, this nodelet has a fading memory method that reduces map confidence
as time goes by. When a voxel confidence reaches a certain threshold, it is deallocated from the map.

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

