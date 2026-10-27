# RB1101 Project #
Wall-Following Project for NUS RB1101 Fundamentals of Robotics I 

## Instructions ##
1. Extract compressed rb1101 folder to desired directory. The following steps assume you extracted to ~/Documents/ (where ~/ is a shortcut for home/user/)
2. Change directory to extracted folder then colcon build. In terminal, enter the following commands: \
`cd ~/Documents/rb1101` \
`colcon build --symlink-install`

3. Assign permission to run shell scripts in terminal. You only need to do this once on each computer/account. \
`chmod +x *.sh`

4. Start the gazebo sim in a terminal. The following command changes directory to the location of your workspace, then runs the Gazebo shell script. \
`cd ~/Documents/rb1101 && ./gz_heading.sh`
   
5. In **another** terminal, run your wall-following script. \
`cd ~/Documents/rb1101 && ./wall_following.sh`
* The default behaviour is left wall-following. Add your modifications to the timer_callback() function of WallFollowingNode() class, in the file `src/rb1101/wall_following.py`.

6. When running wall-following in real life, remember to set the parameter `is_simulation` to `False` in `wall_following.py`.

## Objectives ##
1. Implement wall-following with heading such that the robot will be able to turn right if there's a corner on the back-right, and allow the robot to strive towards facing heading = 0 (initial heading). To test if this works, run Gazebo with the following shell script: \
`cd ~/Documents/rb1101 && ./gz_heading.sh`

* If your wall-following with heading works, you should be able to escape the "G" and proceed forward after the G.

2. Implement wall-following with pledge such that the robot turns left or right as needed, and moves forward if the cumulative heading is equal to the initial heading. To test if this works, run Gazebo with the following shell script: \
`cd ~/Documents/rb1101 && ./gz_pledge.sh` 

* If your wall-following with pledge works, you should be able to escape the "G" and turn right after the G.

3. Ensure your implementation 

## References ##
RANSAC algorithm from wall_following.py uses an adapted version from https://github.com/creminem94/Advanced-Wall-Following/tree/main. All credits for the RANSAC implementation to them.