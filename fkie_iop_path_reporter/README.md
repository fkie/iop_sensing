This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_path_reporter:_ PathReporter

Reports local historical and {local, global} planned paths.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame id used in ROS for global coordinates.

_tf_frame_odom (str_, Default: "odom")

> Frame id of local coordinates published to IOP.

_utm_zone (str_, Default: "32U")

> The UTM zone is used for translation of ROS global position coordinates into Lat/Lon coordinates.

_maximum_points (int_, Default: 15)

> The maximum points in the local history list

_min_dist (double_, Default: 0.25)

> This value is used for historical path. A new point is only added if a distance to previous point is more then _min_dist_.

_use_tf_for_historical (bool_, Default: false)

> Use tf for historical path instead of position or odometry. **Feature currently in developement and not available!**

#### Publisher:

> None

#### Subscriber:

_historical_pose (geometry_msgs::PoseStamped)_
_historical_odom (nav_msgs::Odometry)_

> Position or Odometry to create the local historical path.

_planned_global_path (nav_msgs::msg::Path)_

> A list of points for planned path.

_planned_local_path (nav_msgs::msg::Path)_

> A list of points for local path.

