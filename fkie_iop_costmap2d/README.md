This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_costmap2d:_ CostMap2D

This service is used to report a ROS OccupancyGrid to IOP controller. It will only reported a part of the map around the robot. The size can be adjusted by _map_max_edge_size_. No go zones of the IOP Costmap are currently not supported.

#### Parameter:

_tf_frame_odom (str_, Default: "odom")

> Defines the odometry frame id.

_tf_frame_robot (str_, Default: "base_link")

> Defines the robot frame id.

_p_map_max_edge_size (int_ Default: 255)

> Width and height of the map reported by this service. The actual size depends also on resolution of OccupancyGrid.


#### Publisher:

> None

#### Subscriber:

_map (nav_msgs::OccupancyGrid)_

> Map reported by a ROS service, e.g. hector_mapping

