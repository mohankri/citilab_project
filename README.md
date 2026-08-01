# Create a package
```
cd ~/ros2_ws/src

# ros2 project
ros2 pkg create --build-type ament_cmake <project_name> --dependencies <list pkg name>

or

# urdf
ros2 pkg create --build-type ament_cmake urdfbot_description --dependencies urdf xacro

mkdir launch rviz urdf inside urdfbot_description

# meshes

cd ~/ros2_ws/src/
git clone https://bitbucket.org/theconstructcore/urdf_meshes.git
cp -r ~/ros2_ws/src/urdf_meshes/meshes ~/ros2_ws/src/<project_name>/

```

# Run 
```
cd ~/ros2_ws
colcon build --packages-select <project_name>
source install/setup.bash

# project_name is my_box_bot_description and launch file is urdf_visualize_geometric.launch.py

ros2 launch my_box_bot_description urdf_visualize_geometric.launch.py
```

# Debug
```
# validate urdf xml file

check_urdf src/my_box_bot_description/urdf/box_bot_geometric.urdf

# run tf_tree

cd ~/ros2_ws
source install/setup.bash
ros2 run rqt_tf_tree rqt_tf_tree

```

# rviz

```
# visualize rviz file

rviz2 -d ~/ros2_ws/src/urdfbot_description/rviz/urdf_vis.rviz

```
