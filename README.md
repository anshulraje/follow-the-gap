# Follow the Gap
The Follow the Gap method is based on the construction of a gap array around the vehicle and the calculation of the best heading angle for heading the robot into the center of the maximum gap while simultaneously considering the goal point. You can see my implementation of the algorithm in the video demonstration below.

## Video Demonstration
https://github.com/anshulraje/follow-the-gap/assets/88229267/21a1d09c-023a-45ac-9af9-a33199606d4c

## How to use this repository
Prerequisites:
1. ROS2: https://docs.ros.org/en/foxy/Installation.html
2. F1Tenth Simulator: https://github.com/f1tenth/f1tenth_gym_ros

After installing and setting up the prerequisites, you can clone and build this package in your workspace. Copy the ```Spielberg_obs.png``` and ```Spielberg_obs.yaml``` files from the *maps* folder into the correct *maps* folder of the simulator.

Then you can launch the simulator:
```
ros2 launch f1tenth_gym_ros gym_bridge.py
```
And run the program:
```
ros2 run follow-the-gap ftg_node
```

## Limitations of this repository
The logic used in implementing this algorithm traditionally requires knowledge of obstacle locations to reactively control the car to pass obstacles without crashing into them. To achieve this, we need to filter the LiDAR data to identify the obstacles which has not been implemented in this repository yet. So you will have to fine-tune some parameters, such as the gap size or the turning radius, to your specific use case.
