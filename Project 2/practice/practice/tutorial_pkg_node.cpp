#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include <fstream>
#include <time.h>
#include <iomanip>

using namespace std;
struct EulerAngles{
double roll, pitch, yaw;};
// yaw is what you want, i.e. Th
struct Quaternion{double w, x, y, z;};
EulerAngles ToEulerAngles(Quaternion q)
{
	EulerAngles angles;
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	angles.roll = atan2(sinr_cosp, cosr_cosp);
	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
	angles.pitch = copysign(M_PI/2, sinp); //use 90 degrees if out of range
	else
	angles.pitch = asin(sinp);
	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	angles.yaw = atan2(siny_cosp, cosy_cosp);
	return angles;
}

	ofstream odomVelFile;


class Stopper {
public:
	// Tunable parameters
	// double landmark1 = 1.1, landmark2 =0.5, landmark3 =1.09,landmark4 =1.4,landmark5 =0.25;
	constexpr const static double FORWARD_SPEED_LOW = 0.1;
	constexpr const static double FORWARD_SPEED_HIGH = 0.2;
	constexpr const static double FORWARD_SPEED_SHIGH = 0.4;
	constexpr const static double FORWARD_SPEED_STOP = 0;
	constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
	constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
	constexpr const static double TURN_RIGHT_SPEED_HIGH = -2.4;
	constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
	constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
	Stopper();
	void startMoving();
	void moveForward(double forwardSpeed);
	void moveStop();
	void moveRight(double turn_right_speed = TURN_RIGHT_SPEED_HIGH);
	void moveForwardRight(double forwardSpeed, double turn_right_speed);
	// add the following code
	Quaternion robotQuat;
	EulerAngles robotAngles;
	double robotHeadAngle;
	void transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg); // callback function for odomerty data
	int stage;
	double PositionX, PositionY;
	double robVelocity;
	double startTime; // start time
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan); // callback function for laser data
	double frontRange, mleftRange, leftRange, rightRange, mrightRange;
private:
	ros::NodeHandle node;
	ros::Publisher commandPub; // Publisher to the robot's velocity command topic
	ros::Subscriber odomSub; //Subscriber to robot’s odometry topic
	ros::Subscriber laserSub; // Subscriber to robot’s laser topic
};
Stopper::Stopper(){
	//Advertise a new publisher for the simulated robot's velocity command topic at 10Hz
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	// subscribe to the odom topic
	odomSub = node.subscribe("odom", 20, &Stopper::odomCallback, this);
	laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
}
//send a velocity command
void Stopper::moveForward(double forwardSpeed){
	geometry_msgs::Twist msg;//The default constructor to set all commands to 0
	msg.linear.x = forwardSpeed; //Drive forward at a given speed along the x-axis.
	commandPub.publish(msg);
}
void Stopper::moveStop(){
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_STOP;
	commandPub.publish(msg);
}
void Stopper::moveRight(double turn_right_speed){
	geometry_msgs::Twist msg;
	msg.angular.z = turn_right_speed;
	commandPub.publish(msg);
}


void Stopper::moveForwardRight(double forwardSpeed, double turn_right_speed){
//move forward and right at the same time
geometry_msgs::Twist msg;
msg.linear.x = forwardSpeed;
msg.angular.z = turn_right_speed;
commandPub.publish(msg);
}

// add the callback function to determine the robot position.
void Stopper::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
	PositionX = odomMsg->pose.pose.position.x;
	PositionY = odomMsg->pose.pose.position.y;
	// cout << PositionX <<","<<PositionY<< endl;
	robVelocity = odomMsg->twist.twist.linear.x;
	double currentSeconds = ros::Time::now().toSec();
	double elapsedTime = currentSeconds - startTime;
	odomVelFile << elapsedTime << "," << robVelocity << endl; // collecting odomertry velocity data
	robotQuat.x = odomMsg->pose.pose.orientation.x;
	robotQuat.y = odomMsg->pose.pose.orientation.y;
	robotQuat.z = odomMsg->pose.pose.orientation.z;
	robotQuat.w = odomMsg->pose.pose.orientation.w;
	robotAngles = ToEulerAngles(robotQuat);
	robotHeadAngle = robotAngles.yaw;
	// code to reach different waypoints using odomerty data
	// if (( PositionY < 1.1 ) & (PositionX < 0.5))
	// stage = 1;
	// // the robot moves forward fast
	// else if (PositionX < 0.58)
	// stage = 2;
	// // the robot turns right to the 1 st gap
	// else if (PositionX < 1.25)
	// stage = 3;
	// // the robot moves forward fast
	// else if (PositionX < 1.38)
	// stage = 4;
	// // the robot moves and turns right slowly
	// else if (PositionX < 1.95)
	// stage = 5;
	// // the robot moves forward to the charger
	// else stage = 6;
}

void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	frontRange = scan->ranges[0]; // get the range reading at 0 radians
	mleftRange = scan->ranges[89]; // get the range reading at -π/4 radians
	leftRange = scan->ranges[179]; // get the range reading at -π/2 radians
	rightRange = scan->ranges[539]; // get the range reading at π/2 radians
	mrightRange = scan->ranges[629]; // get the range reading at π/4 radians
}

void Stopper::transformMapPoint(ofstream& fp, double laserRange, double laserTh,
double robotTh, double robotX, double robotY){
	double transX, transY, homeX=0.3, homeY=0.3;
	transX = laserRange * cos(robotTh + laserTh) + robotX;
	transY = laserRange * sin(robotTh + laserTh) + robotY;
	if (transX < 0) transX = homeX; else transX += homeX;
	if (transY < 0) transY = homeX; else transY += homeY;
	fp << transX << "," << transY << endl;
}

// add the following lines of code
void Stopper::startMoving(){
	
	odomVelFile.open("/home/local/CAMPUS/nk18813/M-Drive/ros_workspace/src/tutorial_pkg/odomVelData.csv", ios::trunc); // open/create file at given location and truncate it
	startTime = ros::Time::now().toSec(); // obtain the start time
	
	ofstream odomTrajFile;
	odomTrajFile.open("/home/local/CAMPUS/nk18813/M-Drive/ros_workspace/src/tutorial_pkg/odomTrajData.csv", ios::trunc); // open/create file at given location and truncate it
	
	ofstream laserFile;
	laserFile.open("/home/local/CAMPUS/nk18813/M-Drive/ros_workspace/src/tutorial_pkg/laserData.csv", ios::trunc); // open/create file at given location and truncate it
	int i = 0; // the index to record laser scan data
	
	// setting initial values of angles to find ranges from laser data
	double frontAngle=0, mleftAngle=0.785, leftAngle=1.57;
	double rightAngle=-1.57, mrightAngle=-0.785;
	
	ofstream laserMapFile;
	laserMapFile.open("/home/local/CAMPUS/nk18813/M-Drive/ros_workspace/src/tutorial_pkg/laserMapData.csv",ios::trunc); // open/create file at given location and truncate it
	
	ros::Rate rate(60); //Define rate for repeatable operations.
	ROS_INFO("Start moving");
	stage = 1;
	while (ros::ok()){ // keep spinning loop until user presses Ctrl+C
	 	// code to see different values of laser data at each state for testing
		cout<< frontRange <<" " <<rightRange << " " <<leftRange<<" "<<mrightRange<<" "<<mleftRange<<" "<< stage<< endl;
		// logic to reach different waypoints using laser data
		switch(stage){
			case 1:
			if(frontRange > 0 ){ // condition to start rosbot movement in forward direction
				moveForward(FORWARD_SPEED_SHIGH);
				if(frontRange<1.1 and frontRange>0.1){ // condition to reach first waypoint
					stage =2;
				}
			}
			break;
			case 2:
			if(rightRange<0.5 and rightRange>0.26 ) // condition to reach second waypoint
				moveForwardRight( FORWARD_SPEED_HIGH,TURN_RIGHT_SPEED_MIDDLE);
			else stage=3;
			break;
			case 3:
			if(frontRange>1.15) // condition to reach third waypoint
			moveForward(FORWARD_SPEED_SHIGH);
			else stage=4;
			break;
			case 4:
			if(frontRange>0.35 and frontRange<1.45 ) // condition to reach fourth waypoint
			moveForwardRight( FORWARD_SPEED_HIGH,TURN_RIGHT_SPEED_MIDDLE);
			else stage = 5;
			break;
			case 5:
		//	if(frontRange>0.5 and leftRange>0.15) // condition to reach fifth waypoint
		//	moveForward(FORWARD_SPEED_SHIGH);
		//	else stage=6;
		//	break;
		//	case 6:
		//	if (leftRange>0.3 and rightRange<1.5) // condition to reach sixth waypoint
		//	moveForwardRight( FORWARD_SPEED_HIGH,TURN_RIGHT_SPEED_HIGH);
		//	else stage =7;
		//	break;
		//	case 7:
			if (frontRange>0.35) // condition to reach final waypoint(charger)
			moveForward(FORWARD_SPEED_SHIGH);
			else stage=8;
			break;
			case 8:
			moveStop(); // stop rosbot at charger
			break;
		}
		odomTrajFile << PositionX << "," << PositionY << endl; // collecting trajectory data and storing to file
		laserFile<<i++ << frontRange << "," << mleftRange << "," << leftRange << "," << rightRange << "," << mrightRange << endl; // collecting laser data and writing to file

		transformMapPoint(laserMapFile, frontRange, frontAngle, robotHeadAngle,PositionX,PositionY);
		transformMapPoint(laserMapFile, mleftRange, mleftAngle, robotHeadAngle,PositionX,PositionY);
		transformMapPoint(laserMapFile, leftRange, leftAngle, robotHeadAngle, PositionX,PositionY);
		transformMapPoint(laserMapFile, rightRange, rightAngle, robotHeadAngle,PositionX, PositionY);
		transformMapPoint(laserMapFile, mrightRange, mrightAngle, robotHeadAngle,PositionX, PositionY);

		ros::spinOnce(); // Allow ROS to process incoming messages
		rate.sleep(); // Wait until defined time passes.
	}
	//close all opened files
	odomTrajFile.close();
	odomVelFile.close();
	laserFile.close();
	laserMapFile.close();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "stopper"); // Initiate new ROS node named "stopper"
	Stopper stopper; // Create new stopper object
	stopper.startMoving(); // Start the movement
	return 0;
}
