#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include "eigen3/Eigen/Dense"
#include "crazyflie_driver/VelocityWorld.h"
#include <optimal_sensing/output_ukf.h>
#include <optimal_sensing/output_inter_uavs_distances.h>
#include <optimal_sensing/output_formation.h>
#include <optimal_sensing/output_uavs_controller.h>
#include <optimal_sensing/output_uavs_pose.h>
#include <string>
#include <iostream>
#include <deque>
#include <numeric>
#include <random>
#include <vector>
#include <eigen_conversions/eigen_msg.h>

#define pi 3.14159265359

using namespace std;
////////////////////Self-defined struct//////////////////
//orientation
typedef struct{
    double roll,pitch,yaw;
}rpy;

// virtual leader
typedef struct{
    float px,py,pz;
    float yaw;
}vir;

// dynamic  virtual leader
typedef struct{
    float px,py,pz;
    float vx,vy,vz;
    float yaw;
}dynamic_vir;

// desired distance to virtual leader
typedef struct{
    float x,y,z;
    float yaw;
}dis;
///////////////////////////////////////////////////////

////////////////////Global variable//////////////////
//Setting
int loop_rate = 10;
float model_height = 0.16;
//Trigger flag
bool formation_mode = false;

//UAV ground truth
Eigen::VectorXd r_target,rs_1, rs_2, rs_3;
rpy rpy_crazyflie_1, rpy_crazyflie_2, rpy_crazyflie_3;
Eigen::VectorXd rs_1_last, rs_2_last, rs_3_last;
Eigen::VectorXd cf_1_vel,cf_2_vel,cf_3_vel;

//UAV motive controller
vir vir_1,vir_2,vir_3;
float KPx ,KPy, KPz, KP_yaw;
//UAV formation controller
dynamic_vir vir_target;
float K_Formation_x ,K_Formation_y, weight_target;
dis dis_1,dis_2,dis_3,dis_hover;
float desired_heading_1, desired_heading_2, desired_heading_3;

//Publisher
crazyflie_driver::VelocityWorld vs_1,vs_2,vs_3;
optimal_sensing::output_uavs_pose uavs_pose;
optimal_sensing::output_uavs_controller formation_controller,uavs_vel;
optimal_sensing::output_formation formation;
/////////////////////////////////////////////////////


////////////////////Callback function//////////////////
//optitrack pose information
geometry_msgs::PoseStamped crazyflie_1_mocap,crazyflie_2_mocap,crazyflie_3_mocap,target_pose;
void host_pos1(const geometry_msgs::PoseStamped::ConstPtr& msg){
	crazyflie_1_mocap = *msg;
}
void host_pos2(const geometry_msgs::PoseStamped::ConstPtr& msg){
	crazyflie_2_mocap = *msg;
}
void host_pos3(const geometry_msgs::PoseStamped::ConstPtr& msg){
	crazyflie_3_mocap = *msg;
}
void host_pos_target(const geometry_msgs::PoseStamped::ConstPtr& msg){
	target_pose = *msg;
}
//UKF estimation
optimal_sensing::output_ukf estimated_target_state;
void estimated_data_cb(const optimal_sensing::output_ukf::ConstPtr& msg){
    estimated_target_state = *msg;
}
////////////////////////////////////////////////////////////

////////////////////UAV controller//////////////////
void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, rpy host_rpy,crazyflie_driver::VelocityWorld* vs, dis& dis_host)
{
    float ex, ey, ez, e_yaw;
    float ux, uy, uz, u_yaw;
    float local_x, local_y;

    local_x = cos(vir.yaw)*dis_host.x+sin(vir.yaw)*dis_host.y;
    local_y = -sin(vir.yaw)*dis_host.x+cos(vir.yaw)*dis_host.y;

    ex = vir.px - host_mocap.pose.position.x - local_x;
    ey = vir.py - host_mocap.pose.position.y - local_y;
    ez = vir.pz - host_mocap.pose.position.z - 0;
    e_yaw = dis_host.yaw - host_rpy.yaw;
    if(e_yaw>pi) e_yaw = e_yaw - 2*pi;
    else if(e_yaw<-pi) e_yaw = e_yaw + 2*pi;
    //ROS_INFO("e_yaw: %.3f",e_yaw);

    ux = KPx*ex;
    uy = KPy*ey;
    uz = KPz*ez;
    u_yaw = KP_yaw*(180*e_yaw/pi);
    if(u_yaw<=-3.0 ||u_yaw>=3.0) u_yaw = 3.0*u_yaw/abs(u_yaw);

    vs->vel.x = ux;
	vs->vel.y = uy;
	vs->vel.z = uz;
	vs->yawRate = -u_yaw;
}
void formation_control(dynamic_vir& dynamic_vir, geometry_msgs::PoseStamped& host_mocap, rpy host_rpy,crazyflie_driver::VelocityWorld*  vs, dis& dis_host, geometry_msgs::PoseStamped& nbr_1_mocap, dis& dis_nbr_1, geometry_msgs::PoseStamped& nbr_2_mocap, dis& dis_nbr_2)
{
    float ex, ey, ez, e_yaw;
    float ux, uy, uz, u_yaw;
	float local_x, local_y;
	float local_x1, local_y1;
	float local_x2, local_y2;
	float dis_x1, dis_y1;
	float dis_x2, dis_y2;

	local_x = cos(dynamic_vir.yaw)*dis_host.x+sin(dynamic_vir.yaw)*dis_host.y;
	local_y = -sin(dynamic_vir.yaw)*dis_host.x+cos(dynamic_vir.yaw)*dis_host.y;

	dis_x1 = dis_host.x - dis_nbr_1.x;
	dis_y1 = dis_host.y - dis_nbr_1.y;

	local_x1 = cos(dynamic_vir.yaw)*dis_x1+sin(dynamic_vir.yaw)*dis_y1;
	local_y1 = -sin(dynamic_vir.yaw)*dis_x1+cos(dynamic_vir.yaw)*dis_y1;

	dis_x2 = dis_host.x - dis_nbr_2.x;
	dis_y2 = dis_host.y - dis_nbr_2.y;

	local_x2 = cos(dynamic_vir.yaw)*dis_x2+sin(dynamic_vir.yaw)*dis_y2;
	local_y2 = -sin(dynamic_vir.yaw)*dis_x2+cos(dynamic_vir.yaw)*dis_y2;

    ex = weight_target*(dynamic_vir.px - host_mocap.pose.position.x + local_x) + (nbr_1_mocap.pose.position.x - host_mocap.pose.position.x + local_x1) + (nbr_2_mocap.pose.position.x - host_mocap.pose.position.x + local_x2);
    ey = weight_target*(dynamic_vir.py - host_mocap.pose.position.y + local_y) + (nbr_1_mocap.pose.position.y - host_mocap.pose.position.y + local_y1) + (nbr_2_mocap.pose.position.y - host_mocap.pose.position.y + local_y2);
    ez =dis_host.z - host_mocap.pose.position.z;
    e_yaw = dis_host.yaw  - host_rpy.yaw;
    if(e_yaw>pi) e_yaw = e_yaw - 2*pi;
    else if(e_yaw<-pi) e_yaw = e_yaw + 2*pi;
    ux = K_Formation_x*ex + dynamic_vir.vx;
    uy = K_Formation_y*ey + dynamic_vir.vy;
    uz = KPz*ez ;
    u_yaw = KP_yaw*(180*e_yaw/pi);
    if(u_yaw<=-3.0 ||u_yaw>=3.0) u_yaw = 3.0*u_yaw/abs(u_yaw);
	
    vs->vel.x = ux;
	vs->vel.y = uy;
	vs->vel.z = uz;
    vs->yawRate = -u_yaw;
}
///////////////////////////////////////////////////////

////////////////////Small function//////////////////
rpy quaternionToRPY(float quat_x, float quat_y, float quat_z, float quat_w)
{
    rpy rpy1;
    double roll, pitch, yaw;
    tf::Quaternion quat1(quat_x,quat_y,quat_z,quat_w);
    tf::Matrix3x3(quat1).getRPY(roll, pitch, yaw);
    rpy1.roll = roll;
    rpy1.pitch = pitch;
    rpy1.yaw = yaw;

    return rpy1;
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
///////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ////////////////////Initialization//////////////////
    ros::init(argc, argv, "heterogeneous_system_formation_tracking");
    ros::NodeHandle nh;
    //Subscriber
    ros::Subscriber host_sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie1/pose", 1, host_pos1);
	ros::Subscriber host_sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie2/pose", 1, host_pos2);
	ros::Subscriber host_sub3 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie3/pose", 1, host_pos3);
    ros::Subscriber host_sub_target = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/target/pose", 1, host_pos_target);
	ros::Subscriber estimated_sub = nh.subscribe<optimal_sensing::output_ukf>("/estimated_data",1,estimated_data_cb);
    //Publisher
    ros::Publisher local_vel_pub1 = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie1/cmd_velocity_world", 1);
	ros::Publisher local_vel_pub2 = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie2/cmd_velocity_world", 1);
    ros::Publisher local_vel_pub3 = nh.advertise<crazyflie_driver::VelocityWorld>("/crazyflie3/cmd_velocity_world", 1);
    ros::Publisher formation_pub = nh.advertise<optimal_sensing::output_formation>("/formation", 1);
    ros::Publisher formation_controller_pub = nh.advertise<optimal_sensing::output_uavs_controller>("/formation_controller", 1);
    ros::Publisher uavs_pose_pub = nh.advertise<optimal_sensing::output_uavs_pose>("/uavs_pose", 1);
    ros::Publisher uavs_vel_pub = nh.advertise<optimal_sensing::output_uavs_controller>("/uavs_vel", 1);
    ros::Rate rate(loop_rate);

    r_target.setZero(3);
    rs_1.setZero(4);rs_2.setZero(4);rs_3.setZero(4);
    rs_1_last.setZero(4);rs_2_last.setZero(4);rs_3_last.setZero(4);
    cf_1_vel.setZero(4);cf_2_vel.setZero(4);cf_3_vel.setZero(4);

    ros::param::get("~KPx", KPx);
    ros::param::get("~KPy", KPy);
    ros::param::get("~KPz", KPz);
    ros::param::get("~KP_yaw", KP_yaw);
    ros::param::get("~K_Formation_x", K_Formation_x);
    ros::param::get("~K_Formation_y", K_Formation_y);
    ros::param::get("~weight_target", weight_target);
    ros::param::get("~desired_heading_1", desired_heading_1);
    ros::param::get("~desired_heading_2", desired_heading_2);
    ros::param::get("~desired_heading_3", desired_heading_3);
    ros::param::get("~dis_1_x", dis_1.x);
    ros::param::get("~dis_1_y", dis_1.y);
    ros::param::get("~dis_1_z", dis_1.z);
    ros::param::get("~dis_2_x", dis_2.x);
    ros::param::get("~dis_2_y", dis_2.y);
    ros::param::get("~dis_2_z", dis_2.z);
    ros::param::get("~dis_3_x", dis_3.x);
    ros::param::get("~dis_3_y", dis_3.y);
    ros::param::get("~dis_3_z", dis_3.z);

    vs_1.header.seq = 0;
    vs_1.header.stamp = ros::Time::now();
    vs_1.header.frame_id ="/world";
	vs_1.vel.x = 0;
	vs_1.vel.y = 0;
	vs_1.vel.z = 0;
	vs_1.yawRate= 0;

    vs_2.header.seq = 0;
    vs_2.header.stamp = ros::Time::now();
    vs_2.header.frame_id ="/world";
	vs_2.vel.x = 0;
	vs_2.vel.y = 0;
	vs_2.vel.z = 0;
	vs_2.yawRate= 0;

    vs_3.header.seq = 0;
    vs_3.header.stamp = ros::Time::now();
    vs_3.header.frame_id ="/world";
	vs_3.vel.x = 0;
	vs_3.vel.y = 0;
	vs_3.vel.z = 0;
	vs_3.yawRate= 0;

    dis_1.yaw = desired_heading_1;
    dis_2.yaw = desired_heading_2;
    dis_3.yaw = desired_heading_3;

    //send a few setpoints before starting
    for(int i = 50; ros::ok() && i > 0; --i){
        local_vel_pub1.publish(vs_1);
        local_vel_pub2.publish(vs_2);
        local_vel_pub3.publish(vs_3);
        
        vir_1.px = crazyflie_1_mocap.pose.position.x;
        vir_1.py = crazyflie_1_mocap.pose.position.y;
        vir_1.pz = 0.2;
        vir_1.yaw = 0;

        vir_2.px = crazyflie_2_mocap.pose.position.x;
        vir_2.py = crazyflie_2_mocap.pose.position.y;
        vir_2.pz = 0.2;
        vir_2.yaw = 0;
        
        vir_3.px = crazyflie_3_mocap.pose.position.x;
        vir_3.py = crazyflie_3_mocap.pose.position.y;
        vir_3.pz = 0.2;
        vir_3.yaw = 0;
        ros::spinOnce();
        rate.sleep();
    }
    ///////////////////////////////////////////////////////
    while(ros::ok()){
        ////////////////////Update Ground Truth//////////////////
        rpy_crazyflie_1 = quaternionToRPY(crazyflie_1_mocap.pose.orientation.x,crazyflie_1_mocap.pose.orientation.y,crazyflie_1_mocap.pose.orientation.z,crazyflie_1_mocap.pose.orientation.w);
        rpy_crazyflie_2 = quaternionToRPY(crazyflie_2_mocap.pose.orientation.x,crazyflie_2_mocap.pose.orientation.y,crazyflie_2_mocap.pose.orientation.z,crazyflie_2_mocap.pose.orientation.w);
        rpy_crazyflie_3 = quaternionToRPY(crazyflie_3_mocap.pose.orientation.x,crazyflie_3_mocap.pose.orientation.y,crazyflie_3_mocap.pose.orientation.z,crazyflie_3_mocap.pose.orientation.w);

        if(rpy_crazyflie_1.yaw>2*pi) rpy_crazyflie_1.yaw = rpy_crazyflie_1.yaw - 2*pi;
        else if(rpy_crazyflie_1.yaw<0) rpy_crazyflie_1.yaw = rpy_crazyflie_1.yaw + 2*pi;
        if(rpy_crazyflie_2.yaw>2*pi) rpy_crazyflie_2.yaw = rpy_crazyflie_2.yaw - 2*pi;
        else if(rpy_crazyflie_2.yaw<0) rpy_crazyflie_2.yaw = rpy_crazyflie_2.yaw + 2*pi;
        if(rpy_crazyflie_3.yaw>2*pi) rpy_crazyflie_3.yaw = rpy_crazyflie_3.yaw - 2*pi;
        else if(rpy_crazyflie_3.yaw<0) rpy_crazyflie_3.yaw = rpy_crazyflie_3.yaw + 2*pi;
        
        rs_1 << crazyflie_1_mocap.pose.position.x, crazyflie_1_mocap.pose.position.y, crazyflie_1_mocap.pose.position.z, rpy_crazyflie_1.yaw;
        rs_2 << crazyflie_2_mocap.pose.position.x, crazyflie_2_mocap.pose.position.y, crazyflie_2_mocap.pose.position.z, rpy_crazyflie_2.yaw;
        rs_3 << crazyflie_3_mocap.pose.position.x, crazyflie_3_mocap.pose.position.y, crazyflie_3_mocap.pose.position.z, rpy_crazyflie_3.yaw;
        r_target <<target_pose.pose.position.x, target_pose.pose.position.y, model_height/2;
        ///////////////////////////////////////////////////////////////

         ////////////////////Publish UAV Ground Truth//////////////////
       //pose check
        uavs_pose.rx_c.data = rs_1(0);
        uavs_pose.ry_c.data = rs_1(1);
        uavs_pose.rz_c.data = rs_1(2);
        uavs_pose.theta_c.data = rs_1(3);
        uavs_pose.rx_r.data = rs_2(0);
        uavs_pose.ry_r.data = rs_2(1);
        uavs_pose.rz_r.data = rs_2(2);
        uavs_pose.theta_r.data = rs_2(3);
        uavs_pose.rx_b.data = rs_3(0);
        uavs_pose.ry_b.data = rs_3(1);
        uavs_pose.rz_b.data = rs_3(2);
        uavs_pose.theta_b.data = rs_3(3);
        uavs_pose_pub.publish(uavs_pose);
        //velocity check
        float duration = 1/(float)loop_rate;
        cf_1_vel << (rs_1(0)-rs_1_last(0))/duration, (rs_1(1)-rs_1_last(1))/duration, (rs_1(2)-rs_1_last(2))/duration, (rs_1(3)-rs_1_last(3))/duration;
        cf_2_vel << (rs_2(0)-rs_2_last(0))/duration, (rs_2(1)-rs_2_last(1))/duration, (rs_2(2)-rs_2_last(2))/duration, (rs_2(3)-rs_2_last(3))/duration;
        cf_3_vel << (rs_3(0)-rs_3_last(0))/duration, (rs_3(1)-rs_3_last(1))/duration, (rs_3(2)-rs_3_last(2))/duration, (rs_3(3)-rs_3_last(3))/duration;
        rs_1_last = rs_1; rs_2_last = rs_2; rs_3_last = rs_3;
        uavs_vel.vx_1.data = cf_1_vel(0);
        uavs_vel.vy_1.data = cf_1_vel(1);
        uavs_vel.vz_1.data = cf_1_vel(2);
        uavs_vel.wz_1.data = cf_1_vel(3);
        uavs_vel.vx_2.data = cf_2_vel(0);
        uavs_vel.vy_2.data = cf_2_vel(1);
        uavs_vel.vz_2.data = cf_2_vel(2);
        uavs_vel.wz_2.data = cf_2_vel(3);
        uavs_vel.vx_3.data = cf_3_vel(0);
        uavs_vel.vy_3.data = cf_3_vel(1);
        uavs_vel.vz_3.data = cf_3_vel(2);
        uavs_vel.wz_3.data = cf_3_vel(3);
        uavs_vel_pub.publish(uavs_vel);
        //formation check
        float rho_xy_1,rho_xy_2,rho_xy_3,beta_12,beta_13,beta_23;
        rho_xy_1 = sqrt(pow(rs_1(0)-r_target(0),2)+pow(rs_1(1)-r_target(1),2));
        rho_xy_2 = sqrt(pow(rs_2(0)-r_target(0),2)+pow(rs_2(1)-r_target(1),2));
        rho_xy_3 = sqrt(pow(rs_3(0)-r_target(0),2)+pow(rs_3(1)-r_target(1),2));
        beta_12 = abs(atan2(rs_1(1)-r_target(1),rs_1(0)-r_target(0))-atan2(rs_2(1)-r_target(1),rs_2(0)-r_target(0)));
        if (beta_12 > pi) beta_12 = 2*pi - beta_12;
        beta_13 = abs(atan2(rs_1(1)-r_target(1),rs_1(0)-r_target(0))-atan2(rs_3(1)-r_target(1),rs_3(0)-r_target(0)));
        if (beta_13 > pi) beta_13 = 2*pi - beta_13;
        beta_23 = abs(atan2(rs_2(1)-r_target(1),rs_2(0)-r_target(0))-atan2(rs_3(1)-r_target(1),rs_3(0)-r_target(0)));
        if (beta_23 > pi) beta_23 = 2*pi - beta_23;
        formation.rho_xy_1.data = rho_xy_1;
        formation.rho_xy_2.data = rho_xy_2;
        formation.rho_xy_3.data = rho_xy_3;
        formation.beta_12.data = beta_12;
        formation.beta_13.data = beta_13;
        formation.beta_23.data = beta_23;
        formation_pub.publish(formation);
        ///////////////////////////////////////////////////////////////
        
        ////////////////////UAV controller//////////////////
        //Choose controller: 
        //key: "1" motive ,"2" formation control ,"s" landding
        int c = getch();
        if (c != EOF) {
            switch (c) {
                case 49:    // motive mode
                {
                    formation_mode = false;
                    break;
                }
                case 50:    // formation_mode
                {
                    formation_mode = true;
                    break;
                }
                case 115:    // key (s), landding
                {   
                    formation_mode = false;
                    vir_1.px = crazyflie_1_mocap.pose.position.x;
                    vir_1.py = crazyflie_1_mocap.pose.position.y;
                    vir_1.pz = 0;
                    vir_1.yaw = desired_heading_1;
                    vir_2.px = crazyflie_2_mocap.pose.position.x;
                    vir_2.py = crazyflie_2_mocap.pose.position.y;
                    vir_2.pz = 0;
                    vir_2.yaw = desired_heading_2;
                    vir_3.px = crazyflie_3_mocap.pose.position.x;
                    vir_3.py = crazyflie_3_mocap.pose.position.y;
                    vir_3.pz = 0;
                    vir_3.yaw = desired_heading_3;
                    break;
                }
            }
        }
        //motive controller
        if(formation_mode == false)
        {
            dis_hover.x = 0;
            dis_hover.y = 0;
            dis_hover.z = 0;
            dis_hover.yaw = 0;
            follow(vir_1,crazyflie_1_mocap,rpy_crazyflie_1,&vs_1,dis_hover);
            local_vel_pub1.publish(vs_1);
            follow(vir_2,crazyflie_2_mocap,rpy_crazyflie_2,&vs_2,dis_hover);
            local_vel_pub2.publish(vs_2);
            follow(vir_3,crazyflie_3_mocap,rpy_crazyflie_3,&vs_3,dis_hover);
            local_vel_pub3.publish(vs_3);
        }
        //formation controller
        else
        {   
            vir_target.px = estimated_target_state.target_pose.x;
            vir_target.py = estimated_target_state.target_pose.y;
            vir_target.pz = model_height/2;
            vir_target.vx = estimated_target_state.target_vel.x;
            vir_target.vy = estimated_target_state.target_vel.y;
            vir_target.yaw =0;
            formation_control(vir_target,crazyflie_1_mocap,rpy_crazyflie_1,&vs_1,dis_1,crazyflie_2_mocap,dis_2,crazyflie_3_mocap,dis_3);
            formation_control(vir_target,crazyflie_2_mocap,rpy_crazyflie_2,&vs_2,dis_2,crazyflie_1_mocap,dis_1,crazyflie_3_mocap,dis_3);
            formation_control(vir_target,crazyflie_3_mocap,rpy_crazyflie_3,&vs_3,dis_3,crazyflie_1_mocap,dis_1,crazyflie_2_mocap,dis_2);
            formation_controller.vx_1.data = vs_1.vel.x;
            formation_controller.vy_1.data = vs_1.vel.y;
            formation_controller.vz_1.data = vs_1.vel.z;
            formation_controller.wz_1.data = vs_1.yawRate;
            formation_controller.vx_2.data = vs_2.vel.x;
            formation_controller.vy_2.data = vs_2.vel.y;
            formation_controller.vz_2.data = vs_2.vel.z;
            formation_controller.wz_2.data = vs_2.yawRate;
            formation_controller.vx_3.data = vs_3.vel.x;
            formation_controller.vy_3.data = vs_3.vel.y;
            formation_controller.vz_3.data = vs_3.vel.z;
            formation_controller.wz_3.data = vs_3.yawRate;
            formation_controller_pub.publish(formation_controller);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
