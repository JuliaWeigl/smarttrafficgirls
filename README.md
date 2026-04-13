events

1. hard braking acceleration less -2

2. near collision rear end of object 1 less than 2 meters of front of object 2

discussions:
- different datasets have different formatting -> hard to make general event detection


## Smart Traffic ROS 2 Workspace

### How to use ```/src``` on your laptop：

#### 0. Download all the .csv files

https://www.mos.ed.tum.de/en/vt/research/data-sets/tumdot-muc/

#### 1. Create the ROS2 workspace on your own PC

```mkdir -p ~/ros2_ws/src```

```cd ~/ros2_ws/src```


#### 2. Clone the code into the src directory:

```git clone git@github.com:JuliaWeigl/smarttrafficgirls.git .```

Note: copy all the .csv files you downloaded in step0 into ```~/ros2_ws/src/data```

#### 3. Go back to the workspace root:

```cd ~/ros2_ws```



#### 4. Build the project (this generates the build and install folders tailored to your environment):

```colcon build --symlink-install```



#### 5. Source the environment and run:

```source install/setup.bash```

```ros2 run smart_traffic csv_player```


## How to Run Publisher Node in Rviz2
#### Terminal1 (solve Global Frame error of Rviz2):

```ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --frame-id map --child-frame-id base_link```

#### Terminal2

```rviz2 ```

(Check: Fixed Franne ->map, Add -> By Topic ->/traffic_objects -> MarkerArray)

#### Terminal3

```source ~/ros2_ws/install/setup.bash```

```ros2 run smart_traffic csv_player```

IMPORTANT NOTES:

in csv_player_node.py change the path to your specific path. We should probably change it to a variable that chooses teh local directory.
Add the csv file in a data folder under smart_traffic folder.
