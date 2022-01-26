#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include "crazyflie_driver/VelocityWorld.h"
#define gravity 9.806
using namespace std;

bool init = false;
bool start = false;
//set control P-gain
float KPx=1, KPy=1, KPz=1.2;
//float KPx=5, KPy=5, KPz=1;
float KPyaw = 1;
double roll, pitch, yaw;

typedef struct {
	float yaw;
	float x;
	float y;
	float z;
} vir;

geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped initial_pose;
float initial_yaw;

void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//store odometry into global variable
	host_mocap.header = msg->header;
	host_mocap.pose.position = msg->pose.position;
	host_mocap.pose.orientation = msg->pose.orientation;
	//transfer quartenion to roll, pitch, yaw
	tf::Quaternion Q(
	        host_mocap.pose.orientation.x,
	        host_mocap.pose.orientation.y,
	        host_mocap.pose.orientation.z,
	        host_mocap.pose.orientation.w);
	tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
	if(yaw>2*3.14)
		yaw = yaw - 2*3.14;
	else if(yaw<0)
		yaw = yaw + 2*3.14;
	ROS_INFO("yaw: %3f", yaw);
	//store intial pose for first callback
	if(init == false) {
		initial_pose = host_mocap;
		initial_yaw = yaw;
		ROS_INFO("initial_pose: %3f, %3f, %3f", initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z);
		ROS_INFO("initial_yaw: %3f", initial_yaw);
		init = true;
	}
}

void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, crazyflie_driver::VelocityWorld* vs,float vx,float vy,float ax,float ay)
{
	float errx, erry, errz, err_yaw;
	float ux, uy, uz, uyaw;
	//compute error: desired - measurement
	errx = vir.x - host_mocap.pose.position.x;
	erry = vir.y - host_mocap.pose.position.y;
	errz = vir.z - host_mocap.pose.position.z;
	err_yaw = vir.yaw - yaw;
	if(err_yaw>3.14)
			err_yaw = err_yaw - 2*3.14;
	else if(err_yaw<-3.14)
			err_yaw = err_yaw + 2*3.14;
	ROS_INFO("err: %.3f,%.3f,%.3f,%.3f", errx, erry, errz, err_yaw);

	if(start == false) {
		ux = KPx*errx;
		uy = KPy*erry;
		uz = KPz*errz;
		uyaw = (KPyaw*180*err_yaw)/3.14;
	} else {
		//feedback + feedforward control
		ux = KPx*errx + vx;
		uy = KPy*erry + vy;
		uz = KPz*errz;
		uyaw = (KPyaw*180*err_yaw)/3.14;
	}

	//set max&min for control input
	if(ux<=-1.5 ||ux>=1.5) {
		ux = 1.5*ux/abs(ux);
	}
	if(uy<=-1.5 ||uy>=1.5) {
		uy = 1.5*uy/abs(uy);
	}
	if(uz<=-1.0 ||uz>=1.0) {
		uz = 1.0*uz/abs(uz);
	}
	if(uyaw<=-5.0 ||uyaw>=5.0) {
		uyaw = 5.0*uyaw/abs(uyaw);
	}
	//output control input
	vs->vel.x = ux;
	vs->vel.y = uy;
	vs->vel.z = uz;
	vs->yawRate = -uyaw;
	std::cout << "uyaw:" << uyaw << std::endl; 
}

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motive");
	ros::NodeHandle nh;

	ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie1/pose", 10, host_pos);
	//output final command to flight controller
	ros::Publisher local_vel_pub = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie1/cmd_velocity_world", 2);
	// The setpoint publishing rate MUST be faster than 2Hz.
	ros::Rate rate(30);

	//initialize desired state
	vir vir1;
	vir1.x = initial_pose.pose.position.x;
	vir1.y = initial_pose.pose.position.y;
	vir1.z = initial_pose.pose.position.z+0.5;
	vir1.yaw = initial_yaw;
	//initialize control input
	crazyflie_driver::VelocityWorld vs;
	vs.header.seq = 0;
    vs.header.stamp = ros::Time::now();
    vs.header.frame_id ="/world";
	vs.vel.x = 0;
	vs.vel.y = 0;
	vs.vel.z = 0;
	vs.yawRate= 0;
	float T = 2*M_PI;
	float r = 0.5, a = 1;
	float t = 0, dt = 0.02;
	float m_thrust=20000;
	float duration = 0;

	for(int i = 100; ros::ok() && i > 0; --i) {
		vs.header.seq += 1;
        vs.header.stamp = ros::Time::now();
		local_vel_pub.publish(vs);
		vir1.x = initial_pose.pose.position.x;
		vir1.y = initial_pose.pose.position.y;
		vir1.z = initial_pose.pose.position.z+0.5;
		vir1.yaw = vir1.yaw = initial_yaw;

		ros::spinOnce();
		rate.sleep();
	}




	while (ros::ok()) {
		//keyboard control
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
			case 67:    // key CW(->)
				vir1.yaw -= 0.05;
				break;
			case 68:    // key CCW(<-)
				vir1.yaw += 0.05;
				break;
			case 119:    // key foward(w)
				vir1.x += 0.3;
				break;
			case 120:    // key back(x)
				vir1.x += -0.3;
				break;
			case 97:    // key left(a)
				vir1.y += 0.3;
				break;
			case 100:    // key right(d)
				vir1.y -= 0.3;
				break;
			case 115: {  // key origin(s)
			}
			case 108: {  // closed arming
			}
			case 116: {  // (t)
				if(start == true) {
					start = false;
				} else if(start == false) {
					start = true;
				}
				break;
			}
			case 105: {  // i


				break;
			}
			case 63:
				return 0;
				break;
			}
		}

		if(vir1.yaw>2*3.14)
			vir1.yaw = vir1.yaw - 2*3.14;
		else if(vir1.yaw<0)
			vir1.yaw = vir1.yaw + 2*3.14;

		geometry_msgs::Point vel;
		vel.x = 0;
		vel.y = 0;
		vel.z = 0;
		
		ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z, vir1.yaw);
		//input desired position and measurement, plus feedforward velocity
		//output control input vs
		follow(vir1, host_mocap, &vs, vel.x, vel.y, 0, 0);

		//mocap_pos_pub.publish(host_mocap);
		vs.header.seq += 1;
        vs.header.stamp = ros::Time::now();
		local_vel_pub.publish(vs);
		
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}



