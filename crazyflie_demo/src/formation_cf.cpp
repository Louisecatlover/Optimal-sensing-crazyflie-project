/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "eigen3/Eigen/Dense"
#include "crazyflie_driver/VelocityWorld.h"
#define pi 3.1415926
float KPx = 0.4;
float KPy = 0.4;
float KPz = 0.4;
Eigen::Matrix3d R_N2E;
using namespace std;
struct vir {
	float roll;
	float x;
	float y;
	float z;
};
struct displacement {
	float x;
	float y;
};


geometry_msgs::PoseStamped host_mocap, host_mocap2, host_mocap3, host_mocap4;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	host_mocap = *msg;
}
void host_pos2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	host_mocap2 = *msg;
}
void host_pos3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	host_mocap3 = *msg;
}


void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, crazyflie_driver::VelocityWorld* vs, displacement &dis_host, geometry_msgs::PoseStamped& nbr_mocap, displacement &dis_nbr, geometry_msgs::PoseStamped& nbr2_mocap, displacement &dis_nbr2)
{
	float errx, erry, errz, err_roll;
	float ux, uy, uz;
	float local_x, local_y;
	float local_x1, local_y1;
	float local_x2, local_y2;
	float dis_x1, dis_y1;
	float dis_x2, dis_y2;
	Eigen::Vector3d controller_U,controller_D;

	local_x = cos(vir.roll)*dis_host.x+sin(vir.roll)*dis_host.y;
	local_y = -sin(vir.roll)*dis_host.x+cos(vir.roll)*dis_host.y;

	dis_x1 = dis_host.x - dis_nbr.x;
	dis_y1 = dis_host.y - dis_nbr.y;

	local_x1 = cos(vir.roll)*dis_x1+sin(vir.roll)*dis_y1;
	local_y1 = -sin(vir.roll)*dis_x1+cos(vir.roll)*dis_y1;

	dis_x2 = dis_host.x - dis_nbr2.x;
	dis_y2 = dis_host.y - dis_nbr2.y;

	local_x2 = cos(vir.roll)*dis_x2+sin(vir.roll)*dis_y2;
	local_y2 = -sin(vir.roll)*dis_x2+cos(vir.roll)*dis_y2;

	errx = 1.3*(vir.x - host_mocap.pose.position.x + local_x) + (nbr_mocap.pose.position.x - host_mocap.pose.position.x + local_x1) + (nbr2_mocap.pose.position.x - host_mocap.pose.position.x + local_x2);
	erry = 1.3*(vir.y - host_mocap.pose.position.y + local_y) + (nbr_mocap.pose.position.y - host_mocap.pose.position.y + local_y1) + (nbr2_mocap.pose.position.y - host_mocap.pose.position.y + local_y2);
	errz = 1.3*(vir.z - host_mocap.pose.position.z );

	ux = KPx*errx;
	uy = KPy*erry;
	uz = KPz*errz;
	/*controller_U << ux,uy,uz;
	controller_D = R_N2E*controller_U;
	vs->twist.linear.x = controller_D(0);
	vs->twist.linear.y = controller_D(1);
	vs->twist.linear.z = controller_D(2);*/
	vs->vel.x = ux;
	vs->vel.y = uy;
	vs->vel.z = uz;
}
/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
	int flags = fcntl(0, F_GETFL, 0);
	fcntl(0, F_SETFL, flags | O_NONBLOCK);

	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0) {
		perror("tcsetattr()");
	}
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0) {
		perror("tcsetattr ICANON");
	}
	if (read(0, &buf, 1) < 0) {
		//perror ("read()");
	}
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0) {
		perror ("tcsetattr ~ICANON");
	}
	return (buf);
}


/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "formation_three");
	ros::NodeHandle nh;

	ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie1/pose", 10, host_pos);
	ros::Subscriber host_sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie2/pose", 10, host_pos2);
	ros::Subscriber host_sub3 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie3/pose", 10, host_pos3);

	ros::Publisher local_vel_pub = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie1/cmd_velocity_world", 2);
	ros::Publisher local_vel_pub2 = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie2/cmd_velocity_world", 2);
	ros::Publisher local_vel_pub3 = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie3/cmd_velocity_world", 2);
	

	// The setpoint publishing rate MUST be faster than 2Hz.
	// Using multi thread
	ros::Rate rate(30);

   
	crazyflie_driver::VelocityWorld vs, vs2, vs3;
	vir vir1;
	displacement dis1, dis2, dis3;

	dis1.x = 0;
	dis1.y = -0.6;
	dis2.x= -0.6;
	dis2.y = 0;
	dis3.x= 0;
	dis3.y = 0.6;

	vir1.x = 0.6;
	vir1.y = 1.2;
	vir1.z = 0.5;
	vir1.roll = 0;

	vs.header.seq = 0;
    vs.header.stamp = ros::Time::now();
    vs.header.frame_id ="/world";
	vs.vel.x = 0;
	vs.vel.y = 0;
	vs.vel.z = 0;
	vs.yawRate= 0;

	vs2.header.seq = 0;
    vs2.header.stamp = ros::Time::now();
    vs2.header.frame_id ="/world";
	vs2.vel.x = 0;
	vs2.vel.y = 0;
	vs2.vel.z = 0;
	vs2.yawRate= 0;

	vs3.header.seq = 0;
    vs3.header.stamp = ros::Time::now();
    vs3.header.frame_id ="/world";
	vs3.vel.x = 0;
	vs3.vel.y = 0;
	vs3.vel.z = 0;
	vs3.yawRate= 0;


	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i) {
		local_vel_pub.publish(vs);
		local_vel_pub2.publish(vs2);
		local_vel_pub3.publish(vs3);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
    
	while (ros::ok()) {

		int c = getch();
		//ROS_INFO("C: %d",c);
		if (c != EOF) {
			switch (c) {
			case 65:    // key up
				vir1.z += 0.1;
				break;
			case 66:    // key down
				vir1.z += -0.1;
				break;
			case 67:    // key CW
				vir1.roll += -0.1;
				break;
			case 68:    // key CCW
				vir1.roll += 0.1;
				break;
			case 119:    // key foward
				vir1.x += 0.3;
				break;
			case 120:    // key back
				vir1.x += -0.3;
				break;
			case 97:    // key left
				vir1.y += 0.3;
				break;
			case 100:    // key right
				vir1.y += -0.3;
				break;
			case 115: {  // key right
				vir1.x = 0;
				vir1.y = 0;
				vir1.z = 0;
				vir1.roll = 0;
				break;
			}
			case 108: {  // close arming

				break;
			}
			case 63:
				return 0;
				break;
			}
		}

		ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z,vir1.roll/pi*180);
		follow(vir1,host_mocap,&vs,dis1,host_mocap2,dis2,host_mocap3,dis3);        //be careful of the sign
		vs.header.seq += 1;
        vs.header.stamp = ros::Time::now();
		local_vel_pub.publish(vs);
		follow(vir1,host_mocap2,&vs2,dis2,host_mocap,dis1,host_mocap3,dis3);     //be careful of the sign
		vs2.header.seq += 1;
        vs2.header.stamp = ros::Time::now();
		local_vel_pub2.publish(vs2);
		follow(vir1,host_mocap3,&vs3,dis3,host_mocap,dis1,host_mocap2,dis2);     //be careful of the sign
		vs3.header.seq += 1;
        vs3.header.stamp = ros::Time::now();
		local_vel_pub3.publish(vs3);
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


