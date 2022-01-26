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


geometry_msgs::PoseStamped host_mocap, host_mocap2, host_mocap3, host_mocap_target;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	host_mocap = *msg;
}
void host_pos2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	host_mocap2 = *msg;
}
void host_pos_target(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	host_mocap_target = *msg;
}


void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, crazyflie_driver::VelocityWorld* vs, displacement &dis_host, geometry_msgs::PoseStamped& nbr_mocap, displacement &dis_nbr, float vx, float vy)
{
	float errx, erry, errz, err_roll;
	float ux, uy, uz;
	float local_x, local_y;
	float local_x1, local_y1;
	float dis_x1, dis_y1;

	local_x = cos(vir.roll)*dis_host.x+sin(vir.roll)*dis_host.y;
	local_y = -sin(vir.roll)*dis_host.x+cos(vir.roll)*dis_host.y;

	dis_x1 = dis_host.x - dis_nbr.x;
	dis_y1 = dis_host.y - dis_nbr.y;

	local_x1 = cos(vir.roll)*dis_x1+sin(vir.roll)*dis_y1;
	local_y1 = -sin(vir.roll)*dis_x1+cos(vir.roll)*dis_y1;

	errx = 1.3*(vir.x - host_mocap.pose.position.x + local_x) + (nbr_mocap.pose.position.x - host_mocap.pose.position.x + local_x1) ;
	erry = 1.3*(vir.y - host_mocap.pose.position.y + local_y) + (nbr_mocap.pose.position.y - host_mocap.pose.position.y + local_y1) ;
	errz = 1.3*(vir.z - host_mocap.pose.position.z );

	ux = KPx*errx+vx;
	uy = KPy*erry+vy;
	uz = KPz*errz;

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

	ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie1/pose", 2, host_pos);
	ros::Subscriber host_sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie2/pose", 2, host_pos2);
	ros::Subscriber host_sub_target = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/target/pose", 2, host_pos_target);
	ros::Publisher local_vel_pub = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie1/cmd_velocity_world", 2);
	ros::Publisher local_vel_pub2 = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie2/cmd_velocity_world", 2);
	
	// The setpoint publishing rate MUST be faster than 2Hz.
	// Using multi thread
	ros::Rate rate(30);

	crazyflie_driver::VelocityWorld vs, vs2;
	vir vir1;
	displacement dis1, dis2;
    float vx, vy;

	dis1.x = -0.6;
	dis1.y = 0;
	dis2.x= 0.6;
	dis2.y = 0;

	vir1.x = host_mocap_target.pose.position.x;
	vir1.y = host_mocap_target.pose.position.y;
	vir1.z = 0.5;
	vir1.roll = 0;
    vy = 0.1;

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

	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i) {
		vir1.x = host_mocap_target.pose.position.x;
		vir1.y = host_mocap_target.pose.position.y;
		local_vel_pub.publish(vs);
		local_vel_pub2.publish(vs2);
		ros::spinOnce();
		rate.sleep();
	}
    
	while (ros::ok()) {
		vir1.x = host_mocap_target.pose.position.x;
		vir1.y = host_mocap_target.pose.position.y;
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
            case 115: // key origin(s)
                vx = 0;
                vy = 0;
				break;
            case 116:// key start(t)
                vy = 0.1;
                break;
			}
		}

		ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z,vir1.roll);
		follow(vir1,host_mocap,&vs,dis1,host_mocap2,dis2,vx,vy);        //be careful of the sign
		vs.header.seq += 1;
        vs.header.stamp = ros::Time::now();
		local_vel_pub.publish(vs);
		follow(vir1,host_mocap2,&vs2,dis2,host_mocap,dis1,vx,vy);     //be careful of the sign
		vs2.header.seq += 1;
        vs2.header.stamp = ros::Time::now();
		local_vel_pub2.publish(vs2);
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}



