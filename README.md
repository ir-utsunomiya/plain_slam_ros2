# plain_slam_ros2

**`plain_slam_ros2`** is a ROS2 package for LIO and LiDAR SLAM and localization, built on a minimal and modular SLAM system. In total, the implementation is concise - the C++ source files contain fewer than **3,000 lines of code**.

The core SLAM logic is independent of ROS2 and relies only on Sophus (which depends on Eigen) and nanoflann.



## How to use

### LIO

```
$ ros2 launch plain_slam_ros2 lio_3d.launch
```



### SLAM

```
$ ros2 launch plain_slam_ros2 slam_3d.launch
```



## Localization

The LIO node also has a function as localizer and it can accept our original binary point cloud data and PCD files (plain_slam_ros2 does not depend on PCL but it can receive PCD files).

To use the LIO node as localizer, please first edit config/lio_3d_config.yaml and set the use_as_localizer flag to true. Also, please specify the map_cloud_dir, which includes your PCD files.



## License

plain_slam_ros2 is publicly available software, provided free of charge for academic and personal use only. Commercial use is not permitted without prior written permission from the author.  
See [LICENSE.txt](./LICENSE.txt) for full terms.
