# Nodes
## status\_pac
Execute using `ros2 run gcs status_pac` after sourcing the workspace.  
Publishes a topic `/pac_gcs/status_pac`.  

The status can be changed by entering the value in the terminal.


| value | function | description                                      |
| ----- | -------- | ------------------------------------------------ |
| 0     | ready    | lpac publishes velocity commands                 |
| 1     | pause    | lpac publishes zero velocity commands            |
| 2     | stop     | lpac stops publishing and tries to shutdown node |

## mission\_origin\_gps
Execution command:  
```bash
ros2 run gcs mission_origin_gps --ros-args  -p mission_origin_lon:=1.21113 -p mission_origin_lat:=4.111156 -p heading:=7.89
```

Publishes a topic `/pac_gcs/mission_origin_gps`.
The type is `geometry_msgs/msg/Point`

x: latitude  
y: longitude  
z: heading
