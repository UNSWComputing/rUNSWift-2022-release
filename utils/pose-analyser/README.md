
# Pose Analyser
To visualise the robot pose over time, use the pose-analyser.
It is a simple python3 file, and can be modified for your needs.

## How to analyse
1. Run `pose-analyser data.txt`
    * Replace `data.txt` with the name of the file you want to analyse. This can be either from the state-estimation-simulator, or the ground-truth system.
    * To compare multiple files, additional arguments to the command are supported, such as  
    `pose-analyser data1.txt data2.txt data3.txt`
    * Use the --time / -t flag to plot x, y and heading against time
