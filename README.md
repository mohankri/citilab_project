# Create a package
```
# ros2 project
ros2 pkg create --build-type ament_cmake <project_name> --dependencies <list pkg name>

# urdf
ros2 pkg create --build-type ament_cmake urdfbot_description --dependencies urdf xacro

```
