//Task 2

#include<math.h>
#include<iostream>
#include<fstream>

#define PI   3.14159265
int wheelbase=30, delta_t=1; 
const int SIZE=200; //k = 1,2,3,4,5,6,7....200 (this means 200 results)
double vl=5,vr=7; //paremeters given
double rob_x[SIZE]={30}, rob_y[SIZE]={30}, rob_theta[SIZE]={PI/4};


int robot_kinematics(double left_vel, double right_vel) //left wheel & right wheel
{
int i;
for(i=0; i<SIZE; i++)
{
rob_x[i+1]=rob_x[i]+(left_vel + right_vel)/ 2*cos(rob_theta[i])*delta_t;
//two codes for the equationshere
rob_y[i+1]=rob_y[i]+(left_vel + right_vel)/2*sin(rob_theta[i])*delta_t;
rob_theta[i+1]=rob_theta[i]+(right_vel - left_vel/wheelbase)*delta_t;
}
return 0;
}

//T2
FILE *fp;
int main(int argc, char **argv) 
{
int i;
fp = fopen("odometry_data","w"); //opens odometry data as writable file
robot_kinematics(vl, vr);
// open a file for recording robot_kinematics(vl, vr);

fprintf(fp, "%d, %d  \n",30,30);  //records the trajectory data
for(i=1; i<SIZE;i++){
fprintf(fp, "%f, %f\n", rob_x[i], rob_y[i]);
} // record initial positions
fclose(fp);
return 0;
}
