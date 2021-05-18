手动调参 旋转平移矩阵标定雷达
# sync_lidar
```
mkdir catkin_sync_lidar && cd catkin_sync_lidar
mkdir src && cd src
gitclone https://github.com/Lachiven/sync_lidar.git
cd .. && catkin_make
source devel setup.bash
roslaunch sync_lidar sync_lidar.launch
```
