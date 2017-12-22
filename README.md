See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

List of service plugins in this repository:

[iop_costmap2d_fkie: CostMap2D](#iop_costmap2d_fkie-CostMap2D)  
[iop_measurement_sensor_fkie: MeasurementSensor](#iop_measurement_sensor_fkie-MeasurementSensor)  
[iop_path_reporter_fkie: PathReporter](#iop_path_reporter_fkie-PathReporter)  


## _iop_costmap2d_fkie:_ CostMap2D

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


## _iop_measurement_sensor_fkie:_ MeasurementSensor

This service is created for translation of measurement values defined in [iop_msgs_fkie](https://github.com/fkie/iop_core/tree/master/iop_msgs_fkie).

#### Parameter:

> None


#### Publisher:

> None

#### Subscriber:

_measurement (iop_msgs_fkie::Measurement)_

> Measurement messages


## _iop_path_reporter_fkie:_ PathReporter

Reports historical and planned paths.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame id used in ROS for global coordinates.

_tf_frame_robot (str_, Default: "base_link")

> Defines the robot frame id.

_utm_zone (str_, Default: "32U")

> The UTM zone is used for translation of ROS global position coordinates into Lat/Lon coordinates.

_maximum_points (int_, Default: 15)

> The maximum points in the lists.

_min_dist (double_, Default: 0.25)

> This value is used for historical path. A new point is only added if a distance to previous point is more then _min_dist_.

_use_tf_for_historical (bool_, Default: false)

> Use tf for historical path instead of position or odometry. **Feature currently in developement!**

#### Publisher:

> None

#### Subscriber:

_historical_pose (geometry_msgs::PoseStamped)_
_historical_odom (nav_msgs::Odometry)_

> Position or Odometry to create the historical path. Only active if _use_tf_for_historical_ is *false*.

_planned_global_path (nav_msgs::Path)_

> A list of points for planned path.


