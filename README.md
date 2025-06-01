# unitree mujoco sim

```bash
cd ~/unitree_ws/unitree_mujoco/simulate/build
./unitree_mujoco
```

```bash
cd ~/unitree_ws/unitree_mujoco/basic_nav
source ~/unitree_ws/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=1
./install/drive_go2/bin/drive_go2
```

code in ~/unitree_ws/unitree_ros2/setup_local.sh
```bash
#!/bin/bash
echo "Setup unitree ros2 simulation environment"
source /opt/ros/humble/setup.bash
source $HOME/unitree_ws/unitree_ros2/cyclonedds_ws/install/setup.bash
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```