# Create a package
```
cd ~/ros2_ws/src

# ros2 project
ros2 pkg create --build-type ament_cmake <project_name> --dependencies <list pkg name>

or

# urdf
ros2 pkg create --build-type ament_cmake urdfbot_description --dependencies urdf xacro

mkdir launch rviz urdf inside urdfbot_description

```
