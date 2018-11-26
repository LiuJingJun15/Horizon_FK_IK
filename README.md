# Horizon_FK_IK
The package and node for forward and inverse kinemarics of horizon's arm. 

compute FK and Ik: 
1. nagivate to /src/kaist-ros-pkg/arm_kinematics_tools/src/ikfastdemo
2. in terminal, run './compute fk + 6 numbers(represent 6 joint angles)' or './compute ik + 7 numbers (represent translation and rotation of end effentor)'. (e.g './compute ik 0.3 0.3 0.3 0.3 0.3 0.3')


publisher and subscriber:

rosrun fk_ik talker
rosrun fk_ik listener
        
this gives one solution of inverse kinematics, source file located at src/fk_ik/src
