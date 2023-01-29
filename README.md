# ROS2 Uwb Simulation
This package is a migration of [ROS Pozyx Simulation](https://github.com/bekirbostanci/pozyx_simulation) to ROS2 and can be used to simulate Ultra-Wideband hardware.


![](https://raw.githubusercontent.com/bekirbostanci/ros_pozyx_simulation/master/docs/1.png)

## Description
This package offers 3 node types that can be used simultaneously. When moving to real hardware the 3 nodes need to be swapped with the corresponding hardwares implementations. Although the package includes a position estimation node, it is only intended to represent built-in hardware capabilities. Improved results can be achieved, for example, by using a Kalman filter with an IMU sensor fusion.

## Broadcaster node
The broadcaster's task is to create a static landmark anchors with predefined positions in the simulation.

> [**Script**](src/uwb_broadcaster.py)   

> [**Instantiation**](launch/uwb_anchors_set.launch#L2)

> **Service:** <[Message](srv/RealPosition.srv)> `{name}/set_real_position`  
> *For changing the real position during runtime*  

> **Service:** <[Message](srv/UwbPosition.srv)> `{name}/get_position`  

## Distance node
The distance estimation node represents a simple UWB receiver that can estimate distances to other receivers in range. The distance measurements are published with added noise of 0.02 standard deviation.

> [**Script**](src/uwb_position.py)   

> [**Instantiation**](launch/uwb_distance_example.launch#L11)

> **Service:** <[Message](srv/RealPosition.srv)> `{name}/set_real_position`  
> *For changing the real position during runtime. Not needed with real hardware.*  

> **Service:** <[Message](srv/UwbPosition.srv)> `{name}/get_position`  
> *This node returns no position as it only knows distances, not its own position.*  

> **Topic:** <[Message](msg/UwbData.msg)> `{name}/distance`  
> *The distances to all other UWB clients in range*  

## Position node
The position node represents a simple localization engine that sometimes hardware provides. It calculates the position using the least squares method.

> [**Script**](src/uwb_position.py) 

> [**Instantiation**](launch/uwb_position_example.launch#L5)  

> **Service:** <[Message](srv/RealPosition.srv)> `{name}/set_real_position`  
> *For changing the real position during runtime. Not needed with real hardware.*  

> **Service:** <[Message](srv/UwbPosition.srv)> `{name}/get_position`  
> *Returns the estimated position in the simulation*  

> **Topic:** <[Message](msg/UwbData.msg)> `{name}/distance`  
> *The distances to all other UWB clients in range*  

## Example scripts

### Distance publish
Main script: `ros2 launch launch/uwb_distance_example.launch`  
Listener:    `ros2 topic echo /device_simulation/distance`  

### Position Estimation
Main script: `ros2 launch launch/uwb_position_example.launch`  

