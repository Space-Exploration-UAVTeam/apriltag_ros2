

# apriltag_ros2

`apriltag_ros2` is a ROS2 version of our fixed_camera_params [apriltag_ros](https://github.com/Space-Exploration-UAVTeam/apriltag_ros1/tree/fixed_camera_params). 
ofcourse `apriltag_ros2` depends on the release of the [AprilTag library](https://github.com/AprilRobotics/apriltag). 

## No configuration files
the "settings.yaml" is repalced by written-in parameters because we think the parameters do not often change...
the "tags.yaml" is repalced by written-in parameters because we can not find a way to read it in ROS2, which dose not support XmlRpcValue.

## Fixed camera params
write global variables for camera intrinsics, distortion coefficients, image width, image height and OpenCV distortion maps. rewrite fuction detectTags and getRelativeTransform with no camera parameters passing in.   
so that continuous_detector could spare the trouble of subscribeCamera, and subscribe for the image ONLY.  

## Building
Clone AprilTag library into your catkin workspace 
```
cd apriltag-master
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
sudo cmake --build build --target install
```

Starting with a working ROS installation (Galactic tested):
```
mkdir -p ~/apriltag_ws/src             
cd ~/apriltag_ws/src                  
git clone https://github.com/Space-Exploration-UAVTeam/apriltag_ros2.git 
cd ~/apriltag_ws                       
rosdep install --from-paths src --ignore-src -r -y  
colcon build   
source /opt/ros/Galactic/setup.bash 
source /[your home directory]/apriltag_ws/install/setup.bash 
```

## Running
```
ros2 launch apriltag_ros2 continuous_detection_launch.py
```

## Coordinates and Output
- camera frame: looking from behind the camera (like a photographer), x is right, y is down and z is straight ahead.  
- tag frame: looking straight at the tag (oriented correctly), x is right, y is up and z is towards you (out of the tag).
- the default output of the programe is: Transform_tag2cam: Cam rotation & translation relative to Tag.
