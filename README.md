# Ground Control Station (GCS) for the PAC Project
Build using `colcon build --packages-select gcs`

# Nodes
## status\_pac
Publishes a topic `status_pac`.  
The status can be changed by entering the value in the terminal.


| value | function | description                                      |
| ----- | -------- | ------------------------------------------------ |
| 0     | ready    | lpac publishes velocity commands                 |
| 1     | pause    | lpac publishes zero velocity commands            |
| 2     | stop     | lpac stops publishing and tries to shutdown node |
