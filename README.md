# blind_spot_obstacle_layer

An expansion of the costmap2d obstacle layer that allows for special handling of a polygon (blind spot)

## Test configuration

In a launch file:

```xml

<node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">

    <remap from="cloud_in" to="/aristos/livox/lidar"/>
    <remap from="scan" to="/aristos/livox/lidar/scan"/>
        <rosparam>
            transform_tolerance: 0.01
            min_height: 0.1
            max_height: 2.0

            angle_min: -3.1415 #-1.5708 # -M_PI/2
            angle_max: 3.1415 #1.5708 # M_PI/2
            angle_increment: 0.00087 # M_PI/360.0
            scan_time: 0.1
            range_min: 0.05
            range_max: 30.0
            use_inf: false
            inf_epsilon: -1.0

            # Concurrency level, affects number of pointclouds queued for processing and number of threads used
            # 0 : Detect number of cores
            # 1 : Single threaded
            # 2->inf : Parallelism level
            concurrency_level: 1
        </rosparam>

    </node>

```

In the move_base configuration file for local/global costmap:

```yaml
blind_spot_obstacle_layer:
  inf_is_valid: false
  enabled: true
  blind_spot_polygon: [[0, -3], [3, 0], [0, 3], [-3, 0]]
  blind_spot_frame: "aristos_base_link"
  blind_spot_combination_method: 1
  blind_spot_polygon_marker_topic: "blind_spot_polygon_marker"
  recovery_clearance_polygons_frame: "aristos_base_link"
  recovery_clearance_polygons_topic: "recovery_clearance_polygons"
  recovery_clearance_polygons:
    - [[0, -2], [2, 0], [0, 2], [-2, 0]]
    - [[4, -3], [4, 3], [0, 1], [0, -1]]
    - [[-4, -3], [-4, 3], [0, 1], [0, -1]]
    - [[-6, -2], [6, -2], [6, 2], [-6, 2]]
    - [[-2, -6], [2, -6], [2, 6], [-2, 6]]

  observation_sources: laserscan_from_pointcloud2
  laserscan_from_pointcloud2:
    data_type: LaserScan
    topic: /aristos/livox/lidar/scan
    marking: true
    clearing: true
    track_unknown_space: true
    frame: aristos_mid360_link
    min_obstacle_height: 0.4
    max_obstacle_height: 2.0
    update_frequency: 5.0
    observation_range: 26.0
    obstacle_range: 25.0
    raytrace_range: 30.0
    footprint_clearing: true
```
