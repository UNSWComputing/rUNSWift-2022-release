# State Estimation Simulator (off-board state estimation simulation)

Currently, the state estimation simulator outputs the robot pose, and timestamp.


## Saving Estimator Objects on the NAO
1. Set `save_estimator_objects_to_file` to `TRUE` in runswift.cfg
2. Add `estimator_objects_file_name=out.txt` under `[stateestimation]` in runswift.cfg, to save your file to out.txt. Default file name is `estimator_objects_record.txt`.
3. Follow instructions in [remotecontroller](../remoteControl/README.md) to make robot walk around. Alternatively you can run the robot normally.
4. Run the robot around for as long as you want to collect data.
5. Use the command `scp nao@chewie.local:/var/volatile/runswift/latest/estimator_objects_record.txt .` to copy the file to the current working directory on your laptop. 
   * Replace `chewie` with the name or ip address of your robot.
   * Replace `estimator_objects_record.txt` if you specified a name 

## State Estimation Simulator
1. Modify state estimation as you wish, and compile using `build-relwithdebinfo.sh`.
2. `cd` to the directory where you put your `estimator_objects_record.txt` you got from the robot.
3. Run  
    ```state-estimation-simulator estimator_objects_record.txt out.txt```
   * `estimator_objects_record.txt` is the input to the simulator, and can be replaced by the name of the file from your robot
   * `out.txt` is the output from the simulator, and can be replaced with any name. It stores the robot poses at every tick.
