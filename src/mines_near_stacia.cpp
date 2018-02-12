#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include <sstream>

//File handling
#include <fstream>
//Other
#include <limits>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_sub;
turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;

//method to move robot straight
void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool cloclwise);	//this will rotate the turtle at specified angle from its current angle
double degrees2radians(double angle_in_degrees);		
double setDesiredOrientation(double desired_angle_radians); //this will rotate the turtle at an absolute angle, whatever its current angle is
void poseCallback(const turtlesim::Pose::ConstPtr & msg);	//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);	//this will move robot to goal
void moveGoalLinear(turtlesim::Pose goal_pose);	//this will move robot to goal
double getDistance(double x1, double y1, double x2, double y2);
void readGoalPosesFromFile();

//SUBSCRIBER CALL FUNCTION
//void poseCallback(const std_msgs::String::ConstPtr& msg)
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;
    //ROS_INFO("x = [%f], y = [%f]", pose_message->x, pose_message->y);
}

int main(int argc, char **argv)
{
    //New ROS node
    ros::init(argc, argv, "mines_near_stacia");
    ros::NodeHandle n;

    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;

    speed = 2.0;
    distance = 5.0;
    isForward = 1;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);
    //ros::Rate loop_rate(0.5);
    
    ROS_INFO("\n\n\n ********START TESTING*********\n");

    //move(speed, distance, isForward);

    readGoalPosesFromFile();

    /*
	setDesiredOrientation(degrees2radians(0));
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(90));
	loop_rate.sleep();
    setDesiredOrientation(degrees2radians(180));*/

    //loop_rate.sleep();

    ros::spin();
    return 0;
}

void readGoalPosesFromFile() {

    string line;
    ifstream poseFile("/home/stacia/catkin_ws/src/mines_near_stacia/coords.txt");
    if (poseFile.is_open())
    {
        while (getline(poseFile, line))
        {
            //this code block operates on one x,y coord
            //cout << line << '\n';

            if (line.find(",") != string::npos)
            {
                int splitIndex = line.find(",");
                string x_str = line.substr(0, splitIndex);//add the part up to comma to x
                string y_str = line.substr(splitIndex + 1, string::npos); //skip comma, put rest in y

                float x = strtof(x_str.c_str(),0);
                float y = strtof(y_str.c_str(),0);

                //transform coordinate frames
                //scale, and flip
                //turtle coordinate frame: bottom left is 0,0 - top right is 10,10
                //-----------------------------------------------------------------
                //text file coordinate frame: top left is 0,0 - bottom right is ~350,350
                //                      V V V
                //                     turtle:top left is 0,10 - bottom right is 10,0
                x = x*10/350;
                y = 10 - y*10/350;

                cout << x << " " << y << endl;

                turtlesim::Pose goal_pose;
                goal_pose.x = x;
                goal_pose.y = y;
                goal_pose.theta = 0;
                moveGoalLinear(goal_pose);
            }
        }
        poseFile.close();
    }

    else
        cout << "Unable to open file\n\n";
}

//Only useful if not using position data
void move(double speed, double distance, bool isForward) {
    //distance = speed * time
    geometry_msgs::Twist vel_msg;

    if (isForward) {
        vel_msg.linear.x = abs(speed);
    }
    else {
        vel_msg.linear.x = -abs(speed);
    }

    //no linear vel
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    //random ang vel in y
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    //t0 = current time
    //publish the velocity
    double t0 = ros::Time::now().toSec();
    double current_distance = 0;
    ros::Rate loop_rate(100); //10 msg per sec
    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1-t0);
        ros::spinOnce(); //allows publisher to be published
        loop_rate.sleep();

    }while (current_distance < distance); //while smaller than dist I want to move

    vel_msg.linear.x = 0; //stop immediately
    velocity_publisher.publish(vel_msg);

}

void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   if (clockwise)
	   	vel_msg.angular.z =-abs(angular_speed);
	   else
	   	vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians  
 */

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/**
 *  turns the robot to a desired absolute angle  
 */
double setDesiredOrientation(double desired_angle_radians)
{	
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	//if we want to turn at a perticular orientation, we subtract the current orientation from it
	bool clockwise = ((relative_angle_radians<0)?true:false);
	//cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
	rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

void moveGoalLinear(turtlesim::Pose goal_pose){
	//We implement a Proportional Controller. We need to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
	geometry_msgs::Twist vel_msg;
    float min = std::numeric_limits<float>::max();
    float localMin = std::numeric_limits<float>::max();

	ros::Rate loop_rate(10);

    //First, turn towards the direction you want to go
    //theta = atan(dy/dx)
    //dy = desired y - turtle y
    float desiredTheta = atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x);
    cout << desiredTheta << endl;
    //already in radians
	setDesiredOrientation(desiredTheta);
    loop_rate.sleep();

	do{
        min = localMin;

		//linear velocity 
		vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

        localMin = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

    //This part uses the SUBSCRIBER to check if the desired pose and current pose match yet
    //since we might be off a little on the angle, we just want to get as close as we can
    //as long as the distance is less that during the previous cycle, we must be getting closer
	}while(abs(localMin) < abs(min));

	cout<<"end move goal"<<endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
	//We implement a Proportional Controller. We need to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(10);
	do{
		//linear velocity 
		vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 4*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

    //This part uses the SUBSCRIBER to check if the desired pose and current pose match yet
	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}