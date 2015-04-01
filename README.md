Khepera3_node
=============

This node retreives odometry and LaserScan (LIDAR) information from a Khepera3 on the network through [Player](http://playerstage.sourceforge.net) (using [K-Team's cfg file](http://ftp.k-team.com/KheperaIII/player_stage/korebotII/)), and publishes them on ROS.

Topics
------

 - /laser : Scans (from the urglaser driver - or any Player laser-providing driver) are published as LaserScan messages on the /laser topic at 10Hz and associated with the *laser* frame.
 - /odom : Odometry data, in addition to being published as a tf transformation, is also published as a nav_msgs::Odometry message, to account for speed information. Note that the khepera doesn't seem to actually measure speed from its wheels but assume it moves as its motors should make it do (pushing it will change reported position but speed will stay null).
 -/khepera/cmd_vel is listened to for velocity commands (turtlebot/turtlesim-like). If no command is received through the topic for 2 seconds, speed is reset to 0.

Frames
------

This node uses *khepera* as its base frame (base_link for SLAM nodes), and publishes *khepera*->*odom* information from Odometry, *odom*being ideally a fixed point at the origin of the map, but in practice has to be corrected by location algorithms to account for location error. It also publishes *khepera*->*laser*, which is a fixed translation of {0.02, 0, 0.10}, accounting for the offset between the base of the robot and the laser.

Launch sample
-------------

Included is a gmapping.launch file that launches everything needed to run [Gmapping](http://wiki.ros.org/slam_gmapping) on the robot's data. Gmapping will generate a map and publish the *odom*->*map* transform, giving the corrected absolute position of the robot on the map.

Results
-------

Here's the SLAM-generated map obtained

![The map](http://i.imgur.com/3h9wdRm.png)

Here's what the robot looks like :

![The robot](http://i.imgur.com/MjMiPSVl.jpg)

<sup><sup>Yeah I really need to find a shorter cable</sup></sup>

-----------------------

In theory, this code should work for any robot supporting Player with position2d and laser drivers, Only the relative position of the laser to the robot frame might need to be changed. To be tested.