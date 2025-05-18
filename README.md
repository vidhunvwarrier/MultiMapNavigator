# MultiMapNavigator
## Project Structure
The project contains 2 packages
1. **Multi_map_action**: The custom action has been defined here
2. **Multi_map_navigation**: The multi map navigation using the custom action is performed here, The action server is defined here.
A map folder is provided in the navigation package, which contains the map of the rooms and the python code used to find the wormhole between the maps.


To start the node
```bash
ros2 run multi_map_navigator multi_map_navigator_node
```
To send a goal
```bash
ros2 action send_goal /multi_map_goal multi_map_action/action/MultiMapGoal  "{map_name: 'room2', target_pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```
