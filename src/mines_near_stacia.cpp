#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include <sstream>

//File handling
#include <fstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_sub;
turtlesim::Pose turtlesim_pose;

//method to move robot straight
void move(double speed, double distance, bool isForward);
void poseCallback(const turtlesim::Pose::ConstPtr & msg);	//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);	//this will move robot to goal
double getDistance(double x1, double y1, double x2, double y2);
void readGoalPosesFromFile();

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
    double speed = 2.0;
    double distance = 5.0;
    bool isForward = 1;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);
    ros::Rate loop_rate(0.5);
    
    ROS_INFO("\n\n\n ********START TESTING*********\n");

    //move(speed, distance, isForward);

    readGoalPosesFromFile();


    loop_rate.sleep();

    ros::spin();
    return 0;
}

void readGoalPosesFromFile() {
    /*
    ifstream inFile;
    inFile.open("~/catkin_ws/src/mines_near_stacia/coords.txt");

    if (!inFile)
    {
        cerr << "Unable to open file datafile.txt";
        exit(1); // call system to stop
    }

    while (inFile >> x)
    {
        sum = sum + x;
    }

    inFile.close();*/

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
                moveGoal(goal_pose, 0.01);
            }
        }
        poseFile.close();
    }

    else
        cout << "Unable to open file\n\n";
}

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