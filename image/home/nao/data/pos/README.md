Welcome to pos files! This are rUNSWift specific file formats made for hardcoding motion.  

You can hardcode motion by having "key frame" Joint Value Commands, and giving a duration to achieve that position.  
The default stiffness for each keyframe is 1.0 for all joints.  
To specify joint stiffnesses for a key frame, add a Stiffness Value Command on the line above the Joint Value Command.  
The intermediate joint angles are calculated through interpolationg in the ActionGenerator, and GetupGenerator.

## $ - Stiffness Value Command
Lines that start with `$` indicate stiffness commands. Stiffness can vary from 0.0 to 1.0.  
By default, every joint angle command will have stiffness 1 for each joint by default.  
**For using non-default stiffnesses, this must be specified on EVERY line, right BEFORE the Joint Value command.**  

## ! - Joint Angle Command (& Duration)
Lines that start with `!` indicate joint angles, followed by the duration for the motion to be executed.  
The order of joints follow the V6 joint order.  
Since, this is slightly different from the V5 joint order, a joint order shifting takes place inside ActionGenerator.

## Comments
Comments can be inserted on empty lines in the pos file. Comments SHOULD NOT be made on the same lines as the stiffness of joint angles

## Formatting
The order of joints can be found in `joint names.txt`, copy this into your pos file and follow the same spacing for readability.
