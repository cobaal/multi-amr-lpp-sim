# multi-amr-lpp-sim
based on turtlesim, local path planning, AMR, 2d

## Install
1. Get package (insert your `workspace`)
~~~
cd {workspace}/src
git clone https://github.com/cobaal/multi-amr-lpp-sim.git
~~~
2. Build (insert your `workspace`)
~~~
apt install nlohmann-json3-dev
cd {workspace}
colcon build
colcon build --allow-overriding turtlesim 
source install/setup.bash
~~~

  
## Settings
1. In `.bashrc` file (insert your `workspace`)
~~~
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE={workspace}/src/profiles.xml
export WAYPOINTS_PATH={workspace}/src/multi-amr-lpp-sim/turtlesim/data/waypoints.json
~~~
2. Make your global path (`workspace`/src/multi-amr-lpp-sim/turtlesim/data/waypoints.json)
~~~
{
    "turtle1": [
      { "x": 1.0, "y": 1.0, "theta": 0.0 },
      { "x": 1.0, "y": 2.0, "theta": 0.0 },
      { "x": 2.0, "y": 2.0, "theta": 0.0 },
      { "x": 2.0, "y": 1.0, "theta": 0.0 },
      { "x": 3.0, "y": 1.0, "theta": 0.0 },
      { "x": 3.0, "y": 3.0, "theta": 0.0 },
      { "x": 4.0, "y": 2.0, "theta": 0.0 },
      { "x": 4.0, "y": 4.0, "theta": 0.0 },
      { "x": 5.0, "y": 4.0, "theta": 0.0 },
      { "x": 5.0, "y": 5.0, "theta": 0.0 },
      { "x": 3.0, "y": 5.0, "theta": 0.0 },
      { "x": 3.0, "y": 7.0, "theta": 0.0 },
      { "x": 4.0, "y": 7.0, "theta": 0.0 },
      { "x": 5.0, "y": 8.0, "theta": 0.0 },
      { "x": 6.0, "y": 8.0, "theta": 0.0 },
      { "x": 6.0, "y": 5.0, "theta": 0.0 },
      { "x": 7.0, "y": 5.0, "theta": 0.0 },
      { "x": 8.0, "y": 4.0, "theta": 0.0 },
      { "x": 9.0, "y": 4.0, "theta": 0.0 },
      { "x": 9.0, "y": 5.0, "theta": 0.0 },
      { "x": 10.0, "y": 5.0, "theta": 0.0 },
      { "x": 10.0, "y": 7.0, "theta": 0.0 },
      { "x": 11.0, "y": 7.0, "theta": 0.0 },
      { "x": 11.0, "y": 8.0, "theta": 0.0 },
      { "x": 12.0, "y": 8.0, "theta": 0.0 },
      { "x": 12.0, "y": 10.0, "theta": 0.0 },
      { "x": 10.0, "y": 10.0, "theta": 0.0 },
      { "x": 9.0, "y": 9.0, "theta": 0.0 },
      { "x": 9.0, "y": 8.0, "theta": 0.0 },
      { "x": 8.0, "y": 8.0, "theta": 0.0 },
      { "x": 8.0, "y": 9.0, "theta": 0.0 }
    ],
    "turtle2": [
      { "x": 1.0, "y": 1.0, "theta": 0.0 },
      { "x": 1.0, "y": 2.0, "theta": 0.0 },
      { "x": 2.0, "y": 2.0, "theta": 0.0 },
      { "x": 2.0, "y": 1.0, "theta": 0.0 },
      { "x": 3.0, "y": 1.0, "theta": 0.0 },
      { "x": 3.0, "y": 3.0, "theta": 0.0 },
      { "x": 4.0, "y": 2.0, "theta": 0.0 },
      { "x": 4.0, "y": 4.0, "theta": 0.0 },
      { "x": 5.0, "y": 4.0, "theta": 0.0 },
      { "x": 5.0, "y": 5.0, "theta": 0.0 },
      { "x": 3.0, "y": 5.0, "theta": 0.0 },
      { "x": 3.0, "y": 7.0, "theta": 0.0 },
      { "x": 4.0, "y": 7.0, "theta": 0.0 },
      { "x": 5.0, "y": 8.0, "theta": 0.0 },
      { "x": 6.0, "y": 8.0, "theta": 0.0 },
      { "x": 6.0, "y": 5.0, "theta": 0.0 },
      { "x": 7.0, "y": 5.0, "theta": 0.0 },
      { "x": 8.0, "y": 4.0, "theta": 0.0 },
      { "x": 9.0, "y": 4.0, "theta": 0.0 },
      { "x": 9.0, "y": 5.0, "theta": 0.0 },
      { "x": 10.0, "y": 5.0, "theta": 0.0 },
      { "x": 10.0, "y": 7.0, "theta": 0.0 },
      { "x": 11.0, "y": 7.0, "theta": 0.0 },
      { "x": 11.0, "y": 8.0, "theta": 0.0 },
      { "x": 12.0, "y": 8.0, "theta": 0.0 },
      { "x": 12.0, "y": 10.0, "theta": 0.0 },
      { "x": 10.0, "y": 10.0, "theta": 0.0 },
      { "x": 9.0, "y": 9.0, "theta": 0.0 },
      { "x": 9.0, "y": 8.0, "theta": 0.0 },
      { "x": 8.0, "y": 8.0, "theta": 0.0 },
      { "x": 8.0, "y": 9.0, "theta": 0.0 }
    ],
    ...
}
~~~

## Commands
1. Simulator
~~~
ros2 run turtlesim turtlesim_node
~~~
2. Controller
~~~
% SUBSCRIBER
ros2 run turtlesim local_path_planner {number: 1, 2, ...}
~~~
