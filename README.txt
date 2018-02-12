Stacia Near
HCR Homework part 3


NOTE: See my demo of this at  https://youtu.be/F4oXAeJQb84


Backup github link: https://github.com/nearsr/TurtlesimDrawsLogo
------------------------------------------
How to compile:
Enter the following in separate terminals

roscore
rosrun turtlesim turtlesim_node

put my code in a catkin workspace
The path should look like ~/catkin_ws/src/mines_near_stacia/src
Then go to the base directory

cd ~/catkin_ws

And compile the code

catkin_make

Now you can run my code in the same terminal

rosrun mines_near_stacia mines_near_stacia_node 

The turtle should go ahead and start moving.

------------------------------------------
Files:
src - directory - C++ executable
CMakeLists.txt + package.xml - Mandatory catkin files
minesLogoCoords.jpg - the picture I used to decide what xy points to put in the text file
coords.txt - the file read by the program to tell the turtle what desired coordinates are
snapshot.png - final result
------------------------------------------
Method:
The coords.txt are read in, and converted to the turtle's coordinate space
It figures out what the desired angle to turn to is from the next point
Then the turtle turns to face the point, and moves forward in linear motion until
it gets close to the point (not perfect)
To decide the desired angle, and to figure out when it is close to the point,
the turtle uses a subcribe method to get its own pose in the 2D space
