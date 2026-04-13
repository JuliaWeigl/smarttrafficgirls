events

1. hard braking acceleration less -2

2. near collision rear end of object 1 less than 2 meters of front of object 2

discussions:
- different datasets have different formatting -> hard to make general event detection

**After downloading the src folder**
# Smart Traffic ROS 2 Workspace

## How to run it on your laptop：
# 1. Create and enter the workspace

mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src



# 2. Clone the code into the src directory:

git clone git@github.com:JuliaWeigl/smarttrafficgirls.git .



# 3. Go back to the workspace root:

cd ~/ros2_ws



# 4. Build the project (this generates the build and install folders tailored to your environment):

colcon build --symlink-install



# 5. Source the environment and run:

source install/setup.bash

ros2 run smart_traffic csv_player


**HOW TO RUN NODE IN RVIZ2**
- Terminal1 (solve Global Frame error of Rviz2):

ros2 run tf2_rosstatic_transform_publisher --x 0 --y 0 --z 0 --frame-id map --child-frame-id base_link

- Terminal2

rviz2 

(Check: Fixed Franne ->map, Add -> By Topic ->/traffic_objects -> MarkerArray)

- Terminal3

source ~/ros2_ws/install/setup.bash

ros2 run smart_traffic cs_player
