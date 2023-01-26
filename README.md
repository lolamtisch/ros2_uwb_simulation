# ROS2 Uwb Simulation
This package is a migration of [ROS Pozyx Simulation](https://github.com/bekirbostanci/pozyx_simulation) to ROS2 and can be used to simulate Ultra-Wideband range measurements.


![](https://raw.githubusercontent.com/bekirbostanci/ros_pozyx_simulation/master/docs/1.png)

## Structure
This package provides 3 different node types. First a broadcaster which contain the important parts to stationary anchors. The second one is the distance estiamtion node, it publishes everything the distance all other UWB nodes. The 3rd one is a small showcase for a postion estimation with the help an Kalman filter. All 3 nodes types can be used in the same simulation.

## Broadcaster node
The task of the broadcaster is to create static landmark anchors with predifined positions in the simulation.

> [**Script**](src/uwb_broadcaster.py)   

> [**Instantiation**](launch/uwb_anchors_set.launch#L2)

> **Service:** <[Message](srv/RealPosition.srv)> `{name}/set_real_position`  
> *Can be used to change the position during runtime*  

> **Service:** <[Message](srv/UwbPosition.srv)> `{name}/get_position`  

## Distance node
Represents an UWB radio which can retrive the distance to other UWB recivers. This node should then be replaced when using real Hadware.

> [**Script**](src/uwb_position.py)   

> [**Instantiation**](launch/uwb_distance_example.launch#L11)

> **Service:** <[Message](srv/RealPosition.srv)> `{name}/set_real_position`  
> *Endpoint to update the real position in the simulation world. Because it is only used for the simulation, it can be dropped when moving to real hardware.*  

> **Service:** <[Message](srv/UwbPosition.srv)> `{name}/get_position`  
> *Returns no position, Because this node does not know its position only distances*  

> **Topic:** <[Message](msg/UwbData.msg)> `{name}/distance`  
> *The distances to all other UWB clients in range*  

## Position node
Small example script for a distributed UWB position estimation with the use of a Kalman filter. Based on [bekirbostanci/ieuagv_localization](https://github.com/bekirbostanci/ieuagv_localization/blob/master/src/kalman_filter_localization.py#L84).

> [**Script**](src/uwb_position.py) 

> [**Instantiation**](launch/uwb_position_example.launch#L5)  

> **Service:** <[Message](srv/RealPosition.srv)> `{name}/set_real_position`  
> *Endpoint to update the real position in the simulation world*  

> **Service:** <[Message](srv/UwbPosition.srv)> `{name}/get_position`  
> *Returns the estimated position of the simulation*  

> **Topic:** <[Message](msg/UwbData.msg)> `{name}/distance`  
> *The distances to all other UWB clients in range*  

## Example scripts

### Distance publish
Main script: `ros2 launch launch/uwb_distance_example.launch`  
Listener:    `ros2 topic echo /device_simulation/distance`  

### Position Estimation
Main script: `ros2 launch launch/uwb_position_example.launch`  

