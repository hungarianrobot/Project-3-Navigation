# HURBA: Navigation tutorial III.

[//]: # (Image References)

[image1]: ./documentation/gazebo.png "Gazebo"
[image2]: ./documentation/map.png "Map"
[image5]: ./documentation/rtabmap.png "RTAB-Map"
[image7]: ./documentation/database.png "Database"

### Dependencies:
- [ROS Melodic](http://wiki.ros.org/melodic "ROS Melodic")
- [Gazebo 9](http://wiki.ros.org/gazebo_ros_pkgs "Gazebo ROS package")
- [RViz](http://wiki.ros.org/rviz "RViz")
- [Teleop twist keyboard](http://wiki.ros.org/teleop_twist_keyboard "Teleop twist keyboard")
- [GMapping](http://wiki.ros.org/gmapping "GMapping")
- [RTAB-Map](http://wiki.ros.org/rtabmap_ros "RTAB-Map")
- [AMCL](http://wiki.ros.org/amcl "AMCL")
- [Map server](http://wiki.ros.org/map_server "Map server")
- [move_base](http://wiki.ros.org/move_base "move_base")

### Project build instrctions:
1. Clone this repo inside the `src` folder of a catkin workspace:
`git clone https://github.com/hungarianrobot/Project-3-Navigation`
2. Build workspace: `catkin_make`
3. Source environment: `source devel/setup.bash` 

### Test the simulation
1. Start the Gazebo simulation: `roslaunch hurba_navigation world.launch`
2. Start the teleop package: `roslaunch hurba_navigation teleop.launch`
3. Start RViz and open the `basic_view.rviz` configuration.
4. Drive the omnidirectional robot inside the simulated environment.

![alt text][image1]


### [Optional] Save the map for localization
The localization uses the known map of the environment, so in the first step we have to prepare these maps. This repository already contains the saved map for AMCL and RTAB-Map in the `hurba_navigation/maps/` folder.
Note: the RTAB-Map database is stored as [Git LFS file](https://git-lfs.github.com/ "Git LFS file").

#### Saving map from GMapping:
1. Start the GMapping package: `roslaunch hurba_navigation gmapping_slam.launch`
2. Start the teleop package: `roslaunch hurba_navigation teleop.launch`
3. Drive around the simulated world until you're satisfied with the resulted map.
4. Save the map with the following ROS node: `rosrun map_server map_saver -f map`
5. This will save the map.pmg and map.yaml files to folder from which the command was executed.

![alt text][image2]

#### Saving map from RTAB-Map:
1. Start the RTAB-Map package: `roslaunch hurba_navigation rtab_map_slam.launch`
2. Start the teleop package: `roslaunch hurba_navigation teleop.launch`
3. Drive around the simulated world until you're satisfied with the resulted map.
4. Exit the node with Ctrl+C and the database file will be saved to the location specified in the launchfile.

![alt text][image5]

After the map was saved we can examine the database with RTAB-Map's database viewer, that can be started with the following command:
`rtabmap-databaseViewer ~/catkin_ws/src/Project-3-Navigation/hurba_navigation/maps/rtabmap.db`

![alt text][image7]

Once open, we will need to add some windows to get a better view of the relevant information, so:

* Say yes to using the database parameters
* View -> Constraint View
* View -> Graph View

We can check the number of loop closures during the mapping using the bottom left information:

`(311, 0, 9, 0, 0, 0, 0, 0, 0) Links(N, NM, G, LS, LT, U, P, LM, GR)`

Where G means the number of Global Loop Closures. The codes stand for the following: `Neighbor`, `Neighbor Merged`, `Global Loop closure`, `Local loop closure by space`, `Local loop closure by time`, `User loop closure`, and `Prior link`.

### Navigation with AMCL and move_base
1. Navigate to the `hurba_navigation` folder in the `src` folder of the catkin_workspace: `cd ~/catkin_ws/src/Project-3-Navigation/hurba_navigation/`
2. Execute the following script: `./scripts/launch_amcl_navigation.sh`
3. This will open 3 XTerm windows with specific timing.
    - RViz, Gazebo world and AMCL localization
    - move_base navigation
    - keyboard teleoperation
4. Localization will determine the robot's pose on the loaded map.
5. Send a navigation goal and move_base will drive the robot to the desired location.

### Navigation with RTAB-Map and move_base
1. Navigate to the `hurba_navigation` folder in the `src` folder of the catkin_workspace: `cd ~/catkin_ws/src/Project-3-Navigation/hurba_navigation/`
2. Execute the following script: `./scripts/launch_rtab_navigation.sh`
3. This will open 3 XTerm windows with specific timing.
    - RViz, Gazebo world and RTAB-Map localization
    - move_base navigation
    - keyboard teleoperation
4. Localization will determine the robot's pose on the loaded map.
5. Send a navigation goal and move_base will drive the robot to the desired location.

To put RTAB-Map into localization mode we have to specify the following parameters in the launch file:
```xml
<!-- Put the robot into localization mode -->
<param name="Mem/IncrementalMemory" type="string" value="false"/>
<param name="Mem/InitWMWithAllNodes" type="string" value="true"/>
```
And we have to delete the following SLAM related parameter:
```xml
<!-- Set to false to avoid saving data when robot is not moving -->
<param name="Mem/NotLinkedNodesKept" type="string" value="false"/>
```

### Project structure:
```bash
 tree -L 3
.
├── README.md
├── documentation
│   ├── database.png
│   ├── gazebo.png
│   ├── map.png
│   └── rtabmap.png
└── hurba_navigation
    ├── CMakeLists.txt
    ├── config
    │   ├── base_local_planner_params.yaml
    │   ├── costmap_common_params.yaml
    │   ├── global_costmap_params.yaml
    │   └── local_costmap_params.yaml
    ├── launch
    │   ├── amcl_localization.launch
    │   ├── gmapping_slam.launch
    │   ├── movebase_navigation.launch
    │   ├── robot_description.launch
    │   ├── rtab_map_localization.launch
    │   ├── rtab_map_slam.launch
    │   ├── teleop.launch
    │   └── world.launch
    ├── maps
    │   ├── map.pgm
    │   ├── map.yaml
    │   └── rtabmap.db
    ├── meshes
    │   ├── chassis.dae
    │   ├── chassis.SLDPRT
    │   ├── chassis.STEP
    │   ├── hokuyo.dae
    │   ├── wheel.dae
    │   ├── wheel.SLDPRT
    │   └── wheel.STEP
    ├── package.xml
    ├── rviz
    │   ├── amcl_navigation.rviz
    │   ├── basic_view.rviz
    │   ├── gmapping_slam.rviz
    │   ├── rtab_navigation.rviz
    │   └── rtab_slam.rviz
    ├── scripts
    │   ├── launch_amcl_navigation.sh
    │   └── launch_rtab_navigation.sh
    ├── urdf
    │   ├── hurba_mecanum.gazebo
    │   └── hurba_mecanum.xacro
    └── worlds
        ├── basic_world.world
        ├── building.model
        ├── building.world
        └── empty.world
```

