#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include <time.h>
#include <iomanip>
#include "sensor_msgs/LaserScan.h"

using namespace std;

struct EulerAngles{
double roll, pitch, yaw;}; // yaw is what you want, i.e. Th
 
struct Quaternion{ double w, x, y, z;};
 
EulerAngles ToEulerAngles(Quaternion q)
{
EulerAngles angles;
// roll (x-axis rotation)
double sinr_cosp = +2.0 * (q.w* q.x + q.y * q.z);
double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
angles.roll = atan2(sinr_cosp, cosr_cosp); 
// pitch (y-axis rotation)
double sinp = + 2.0 * (q.w * q.y - q.z * q.x);
if(fabs(sinp) >= 1)
angles.pitch = copysign(M_PI/2,sinp); //use 90 degrees if out of range
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
// Tunable parameters and speedd
constexpr const static double FORWARD_SPEED_LOW = 0.1;
constexpr const static double FORWARD_SPEED_HIGH = 0.2;
constexpr const static double FORWARD_SPEED_SHIGH= 0.4;
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
void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
int stage;
double PositionX, PositionY;
double robVelocity;
double startTime;  // start time
Quaternion robotQuat;
EulerAngles robotAngles; 
double robotHeadAngle;
void transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
double frontRange, mleftRange, leftRange, rightRange, mrightRange;
double backRange, leftbackRange, rightbackRange;
double landmark1 = 1.1, landmark2 = 1.5, landmark3 = 1.6, landmark4 = 0.8, landmark5 = 2;
int vel_timer = 0;

private:
ros::NodeHandle node;
ros::Publisher commandPub; // Publisher to the robot's velocity command topic
ros::Subscriber odomSub; //Subscriber to robot's Odometry topic
ros::Subscriber laserSub;  //Subscriber to robot’s lasertopic
};

Stopper::Stopper(){
//Advertise a new publisher for the simulated robot's velocity command topic at 10Hz
commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
odomSub = node.subscribe("odom", 20,&Stopper::odomCallback, this);
laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
}


//send a velocity command
void Stopper::moveForward(double forwardSpeed){
geometry_msgs::Twist msg;//The default constructor toset all commands to 0
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

void Stopper::moveForwardRight(double forwardSpeed,double turn_right_speed){
//move forward and right at the same time
geometry_msgs::Twist msg;
msg.linear.x = forwardSpeed;
msg.angular.z = turn_right_speed;
commandPub.publish(msg);
}

void Stopper::transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY)
{
double transX, transY, homeX=0.3, homeY=0.3;
transX = laserRange * cos(robotTh + laserTh) + robotX;
transY = laserRange * sin(robotTh + laserTh) + robotY;
if (transX < 0) transX = homeX; else transX += homeX;
if (transY < 0) transY = homeX; else transY += homeY;
fp << transX << "  " << transY << endl;
}

void Stopper::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
PositionX = odomMsg->pose.pose.position.x;
PositionY = odomMsg->pose.pose.position.y;
robVelocity = odomMsg->twist.twist.linear.x;
double currentSeconds = ros::Time::now().toSec(); 
double elapsedTime = currentSeconds - startTime; 
odomVelFile << vel_timer++ << " " << robVelocity << endl;
robotQuat.x = odomMsg->pose.pose.orientation.x;
robotQuat.y = odomMsg->pose.pose.orientation.y;
robotQuat.z = odomMsg->pose.pose.orientation.z;
robotQuat.w = odomMsg->pose.pose.orientation.w;
robotAngles = ToEulerAngles(robotQuat);
robotHeadAngle = robotAngles.yaw;
}

void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
frontRange = scan->ranges[0];// get the range readingat 0 radians
mleftRange = scan->ranges[89];// get therange readingat -π/4 radians
leftRange = scan->ranges[179];   // get the range readingat -π/2radians
rightRange = scan->ranges[539]; // get the range readingat π/2radians
mrightRange = scan->ranges[629]; // get the range readingat π/4 radians
backRange = scan->ranges[359]; // get the range reading at π radians
leftbackRange = scan->ranges[269]; // get the range reading at 3π/4radians
rightbackRange = scan->ranges[449]; // get the range reading at -3π/4 radians}
}

void Stopper::startMoving(){
ofstream odomTrajFile;
odomTrajFile.open("/home/jd16264/ros_workspace/src/tutorial_pkg/odomTrajData.csv",ios::trunc);

odomVelFile.open("/home/jd16264/ros_workspace/src/tutorial_pkg/odomVelData.csv",ios::trunc);
// data files 
ofstream laserFile;
laserFile.open("/home/jd16264/ros_workspace/src/tutorial_pkg/laserData.csv",ios::trunc);

int i = 0;// the index to record laser scan data
double frontAngle=0, mleftAngle=0.785, leftAngle=1.57;
double rightAngle=-1.57, mrightAngle=-0.785;
ofstream laserMapFile;
laserMapFile.open("/home/jd16264/ros_workspace/src/tutorial_pkg/laserMapData.csv",ios::trunc);

startTime = ros::Time::now().toSec();   // obtain the start time
//double frontRange = 2.0;

ros::Rate rate(50);   // Defined rate for repeatable operations.
ROS_INFO("Start moving");
ROS_INFO_STREAM("frontRange: " << frontRange);
ROS_INFO_STREAM("leftRange: " << leftRange);
ROS_INFO_STREAM("rightRange: " << rightRange); // Info stream
ROS_INFO_STREAM("mleftRange: " << mleftRange);
ROS_INFO_STREAM("mrightRange: " << mrightRange);
ROS_INFO_STREAM("stage: " << stage);
stage = 1;
while (ros::ok()){// keep spinning loop until user presses Ctrl+C
switch (stage){
case 1:
if (frontRange > 0){ // checks if the rosbot is reaching the 1st landmark
moveForward(FORWARD_SPEED_SHIGH);
if(frontRange < 1.1 and frontRange > 0.5){
stage = 2;
}
}
break;
case 2:
if (rightRange < 0.55 and rightRange > 0.32){ // 2nd landmark (turning)
moveForwardRight(FORWARD_SPEED_HIGH,TURN_RIGHT_SPEED_MIDDLE);
stage = 3;
}
break;
case 3:
if (frontRange > 1.15 and rightRange < 0.25){ // 3rd landmark through the object
moveForwardRight(FORWARD_SPEED_HIGH, TURN_RIGHT_SPEED_MIDDLE);
stage = 4;
}
break;
case 4:
if (frontRange > 0.6 and frontRange < 1.5){ // 4th landmark straight
moveForward(FORWARD_SPEED_SHIGH);
stage = 5;
} 
break;
case 5:
if (frontRange > 0.7 and rightRange > 0.26){
moveForwardRight(FORWARD_SPEED_HIGH, TURN_RIGHT_SPEED_LOW); // 5th landmark
stage = 6;
}
break;
// Dont touch above at all
case 6:
if (frontRange > 0.3 and leftRange < 0.3){
moveForward(FORWARD_SPEED_HIGH); // 0.55
stage = 7;
}
break;
case 7:
if (frontRange < 0.2 and rightRange < 0.45){
moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_MIDDLE); // 0.55
stage = 8;
}
break;
case 8:
if (frontRange < 0.35){
moveForward(FORWARD_SPEED_SHIGH);
stage = 9;
}
break;
case 9:
moveStop();  
break;
}

odomTrajFile << PositionX << " " << PositionY << endl;
laserFile << i++ << frontRange<< " " << mleftRange << " " << leftRange << " " << rightRange << " " << mrightRange << endl;
transformMapPoint(laserMapFile, frontRange, frontAngle, robotHeadAngle, PositionX, PositionY);
transformMapPoint(laserMapFile, mleftRange, mleftAngle, robotHeadAngle, PositionX, PositionY);
transformMapPoint(laserMapFile, leftRange, leftAngle, robotHeadAngle, PositionX, PositionY);
transformMapPoint(laserMapFile, rightRange, rightAngle, robotHeadAngle, PositionX, PositionY);
transformMapPoint(laserMapFile, mrightRange, mrightAngle, robotHeadAngle, PositionX, PositionY);
ros::spinOnce(); // Allow ROS to process incoming messages
rate.sleep();   // Wait until defined time passes.
}
odomTrajFile.close();
odomVelFile.close(); //closes data files
laserFile.close();
laserMapFile.close();
}

int main(int argc, char **argv) {
ros::init(argc, argv, "stopper"); // Initiate new ROS node named "stopper"
Stopper stopper; // Create new stopper object  
stopper.startMoving();  // Start the movement
return 0;
}



