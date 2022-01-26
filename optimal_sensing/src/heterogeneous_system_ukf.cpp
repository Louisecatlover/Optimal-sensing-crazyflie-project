#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <optimal_sensing/output_ukf.h>
#include <optimal_sensing/output_measurement.h>
#include <optimal_sensing/output_uavs_controller.h>
#include <std_msgs/Float64.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <deque>
#include <numeric>
#include <random>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include <eigen_conversions/eigen_msg.h>
#define pi 3.14159265359

using namespace std;

typedef struct
{
    double roll;
    double pitch;
    double yaw;
}rpy;

////////////////////Global variable//////////////////
//Setting
int loop_rate = 10;
float model_height = 0.16;
//Trigger flag
bool ukf_flag = false;
bool initial_finish_flag = false;
bool optimal_sensing_mode = false;
//Timer
double dt;
ros::Time previous_time,current_time;

//UAV and target ground truth
Eigen::VectorXd rs_1, rs_2, rs_3;
rpy rpy_crazyflie_1;
Eigen::VectorXd r_target, r_target_last;
Eigen::VectorXd target_gvel;
//UAV rotation matrix(c : camera b:UAV body_frame g : global) 
Eigen::MatrixXd rotation_crazyflie_1_x, rotation_crazyflie_1_y, rotation_crazyflie_1_z;
Eigen::MatrixXd rotationB2C_crazyflie_x, rotationB2C_crazyflie_y, rotationB2C_crazyflie_z;
Eigen::MatrixXd rot_g2c;

//UKF parameters
Eigen::MatrixXd P_init;
Eigen::MatrixXd process_noise;
Eigen::MatrixXd measurement_noise;
//System measurement
Eigen::VectorXd measure_vector;
float center_u_1_model, center_v_1_model, rho_2, beta_3;
float fx = 381.36, fy = 381.36,cx = 320.5, cy = 240.5; 

//Publisher
optimal_sensing::output_ukf error_value, true_value, estimate_value;
optimal_sensing::output_measurement measurement_value;
std_msgs::Float64MultiArray P_final;
std_msgs::Float64 det_P_msg;
/////////////////////////////////////////////////////

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

////////////////////Callback function//////////////////
//Optitrck pose information
geometry_msgs::PoseStamped crazyflie_1_mocap,crazyflie_2_mocap,crazyflie_3_mocap,target_pose;
void host_pos1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	crazyflie_1_mocap = *msg;
}
void host_pos2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	crazyflie_2_mocap = *msg;
}
void host_pos3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	crazyflie_3_mocap = *msg;
}
void host_pos_target(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	target_pose = *msg;
}
////////////////////////////////////////////////////////////


////////////////////UKF global variable//////////////////
//define state and measurement
enum state{
    member_xq=0,
    member_yq,
    member_vqx,
    member_vqy,
    statesize
};
enum measurement{
    member_mu_1 = 0,
    member_mv_1,
    member_mrho_2,
    member_mbeta_3,
    measurementsize
};
//estimator
Eigen::VectorXd x ; //states
Eigen::MatrixXd P ; //covariance matrix
float det_P;//detminant of covariance matrix (position only)
//initialize
int x_size,y_size;
int x_sigmavector_size,y_sigmavector_size;
double L,alpha,kappa,beta,lambda;//sigma point parameters
Eigen::MatrixXd H ;    //measurement transform
Eigen::VectorXd w_c,w_m ; //weights for 2 models
//predict
Eigen::MatrixXd x_sigmapoint;
Eigen::MatrixXd x_sigmavector ;
Eigen::VectorXd x_hat; //x mean
Eigen::MatrixXd P_ ; //predict state covariance matrix
Eigen::MatrixXd Q ; //process noise matrix
Eigen::MatrixXd y_sigmavector ;
Eigen::VectorXd y_hat; //y mean
//correct
Eigen::VectorXd y ; //measurements
Eigen::MatrixXd P_yy_ ;//predict mesurement covariance matrix
Eigen::MatrixXd R ; //measurement noise matrix
Eigen::MatrixXd Kalman_gain ;
Eigen::MatrixXd P_xy ;
Eigen::MatrixXd P_yy ;
////////////////////////////////////////////////////////////////

////////////////////UKF function//////////////////
//Predict state
Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state){

    x_size = statesize;
    x_sigmavector_size = 2*x_size+1;
    Eigen::MatrixXd predict_sigma_state(x_size,x_sigmavector_size);

    for(int i=0;i<x_sigmavector_size;i++){        
        //initialize the sigma vector
        Eigen::Vector2d q_pose,vq;
        q_pose.setZero();
        vq.setZero();
        q_pose << sigma_state(member_xq,i), sigma_state(member_yq,i);
        vq << sigma_state(member_vqx,i), sigma_state(member_vqy,i);
        //Process model
        Eigen::Vector2d q_pose_,vq_;
        q_pose_.setZero();
        vq_.setZero();
        q_pose_(0) = q_pose(0) + vq(0)*dt;
        q_pose_(1) = q_pose(1) + vq(1)*dt;
        vq_ = vq;         
       //update the predict_sigma_state
        predict_sigma_state(member_xq,i) =  q_pose_(0);
        predict_sigma_state(member_yq,i) =  q_pose_(1);
        predict_sigma_state(member_vqx,i) =  vq_(0);
        predict_sigma_state(member_vqy,i) =  vq_(1);
    }
    return predict_sigma_state;
}
//Predict measurement
Eigen::MatrixXd state_to_measure(Eigen::MatrixXd sigma_state){
    y_size = measurementsize;
    x_sigmavector_size = 2*x_size+1;
    Eigen::MatrixXd predict_sigma_measure(y_size,x_sigmavector_size);
    for(int i=0;i<x_sigmavector_size;i++){
        predict_sigma_measure( member_mu_1 ,i) = (double)-fx*(cos(rpy_crazyflie_1.yaw)*(rs_1(1)-sigma_state(member_yq,i))-sin(rpy_crazyflie_1.yaw)*(rs_1(0)-sigma_state(member_xq,i)))/(cos(rpy_crazyflie_1.yaw)*(rs_1(0)-sigma_state(member_xq,i))+sin(rpy_crazyflie_1.yaw)*(rs_1(1)-sigma_state(member_yq,i))) + (double)cx;
        predict_sigma_measure( member_mv_1 ,i) = (double)-fy*(rs_1(2)-r_target(2))/(cos(rpy_crazyflie_1.yaw)*(rs_1(0)-sigma_state(member_xq,i))+sin(rpy_crazyflie_1.yaw)*(rs_1(1)-sigma_state(member_yq,i))) + (double)cy;
        predict_sigma_measure( member_mrho_2 ,i) = sqrt(pow(rs_2(0)-sigma_state(member_xq,i),2) + pow(rs_2(1)-sigma_state(member_yq,i),2));
        predict_sigma_measure( member_mbeta_3 ,i) =  atan2(rs_3(1)-sigma_state(member_yq,i),rs_3(0)-sigma_state(member_xq,i));
    }
    return predict_sigma_measure;
}
//UKF process
void initialize(){
    ROS_INFO("Initilaize");
    x_size = statesize;
    y_size = measurementsize;
    alpha = 0.001;
    kappa = 0.0;
    beta = 2.0;
    lambda = 0.0;

    L=(double)x_size;
    x_sigmavector_size=2*x_size+1;

    lambda= alpha * alpha * (L + kappa) -L;

    x.setZero(x_size);
    y.setZero(y_size);

    x_hat.setZero(x_size);
    y_hat.setZero(y_size);

    x_sigmapoint.setZero(x_size,x_sigmavector_size);
    x_sigmavector.setZero(x_size,x_sigmavector_size);
    y_sigmavector.setZero(y_size,x_sigmavector_size);

    H.setZero(y_size,x_size);  // measurement matrix
    y = H*x;

    w_c.setZero(x_sigmavector_size);
    w_m.setZero(x_sigmavector_size);

    w_c(0) = (lambda / (L+lambda))+(1.0-alpha*alpha+beta);
    w_m(0) = (lambda)/(L+lambda);
    for(int i=1 ; i<x_sigmavector_size ; i++){
        w_c(i) = 1/(2*(L+lambda));
        w_m(i) = 1/(2*(L+lambda));
    }

    Q.setZero();
    R.setZero();
    P.setZero(x_size,x_size);
    P_.setZero(x_size,x_size);
    P_yy_.setZero(y_size,y_size);
    P_yy.setZero(y_size,y_size);
    P_xy.setZero(x_size,y_size);
    Kalman_gain.setZero(x_size,y_size);
}
void predict(){
    //find sigma point
    P=(lambda+L)*P;
    Eigen::MatrixXd M;
    M.setZero(x_size,x_size);
    M = (P).llt().matrixL();
    x_sigmapoint.col(0) = x;
    for(int i=0;i<x_size;i++)
    {
        Eigen::VectorXd sigma =(M.row(i)).transpose();
        x_sigmapoint.col(i+1) = x + sigma;
        x_sigmapoint.col(i+x_size+1) = x - sigma;
    }
    //predict state
    x_sigmavector = dynamics( x_sigmapoint);
    //x_hat (mean) 
    x_hat.setZero(x_size);//initialize x_hat
    for(int i=0;i<x_sigmavector_size;i++){
        x_hat += w_m(i)* x_sigmavector.col(i);
    }
    //covariance
    P_.setZero(x_size,x_size);
    for(int i=0 ; i<x_sigmavector_size ;i++){
        P_+=   w_c(i) * (x_sigmavector.col(i)-x_hat) * ((x_sigmavector.col(i)-x_hat).transpose());
    }
    //add process noise covariance
    P = P_ + Q;
    //predict measurement
    y_sigmavector = state_to_measure(x_sigmavector);
    //y_hat (mean)
    y_hat.setZero(y_size);
    for(int i=0;i< x_sigmavector_size;i++){
        y_hat += w_m(i) * y_sigmavector.col(i);
    }
}
void correct(Eigen::VectorXd measure){
    y = measure;

    P_yy_.setZero(y_size,y_size);
    P_yy.setZero(y_size,y_size);
    P_xy.setZero(x_size,y_size);

    for(int i=0;i<x_sigmavector_size;i++){
        Eigen::VectorXd err_y;
        err_y.setZero(y_size);
        err_y = y_sigmavector.col(i) - y_hat;
        P_yy_ += w_c(i) * err_y * err_y.transpose();
    }
    //add measurement noise covarinace
    P_yy = P_yy_ + R;

    for(int i=0;i<x_sigmavector_size;i++){
        Eigen::VectorXd err_y , err_x;
        err_y.setZero(y_size);
        err_x.setZero(x_size);
        err_y = y_sigmavector.col(i) - y_hat;
        err_x = x_sigmavector.col(i) - x_hat;
        P_xy += w_c(i) * err_x * err_y.transpose();
    }
    Kalman_gain = P_xy * (P_yy.inverse());
    x = x_hat + Kalman_gain *(y-y_hat);
    P = P - Kalman_gain * P_yy * (Kalman_gain.transpose());
    det_P = (float)P.block<2,2>(0,0).determinant();
    cout << "det_P:" << det_P << endl;
}
/////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heterogeneous_system_sensing");
    ros::NodeHandle nh;
    //Subscriber
    ros::Subscriber host_sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie1/pose", 1, host_pos1);
	ros::Subscriber host_sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie2/pose", 1, host_pos2);
	ros::Subscriber host_sub3 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/crazyflie3/pose", 1, host_pos3);
    ros::Subscriber host_pos_target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/target/pose", 1, host_pos_target);
    //Publisher
    ros::Publisher error_pub = nh.advertise<optimal_sensing::output_ukf>("/error_data", 1);
    ros::Publisher measurement_pub = nh.advertise<optimal_sensing::output_measurement>("/measurement_data", 1);
    ros::Publisher det_P_pub = nh.advertise<std_msgs::Float64>("/det_P", 1);
    ros::Publisher true_pub = nh.advertise<optimal_sensing::output_ukf>("/true_data", 1);
    ros::Publisher estimate_pub = nh.advertise<optimal_sensing::output_ukf>("/estimated_data", 1);
    ros::Publisher covariance_pub = nh.advertise<std_msgs::Float64MultiArray>("/covariance_matrix", 1);
    ros::Rate rate(loop_rate);
    //initialize variables 
    rs_1.setZero(3);rs_2.setZero(3);rs_3.setZero(3);
    r_target.setZero(3);
    r_target_last.setZero(3);
    target_gvel.setZero(2);
    rotation_crazyflie_1_x.setZero(3,3);rotation_crazyflie_1_y.setZero(3,3);rotation_crazyflie_1_z.setZero(3,3);
    rotationB2C_crazyflie_x.setZero(3,3);rotationB2C_crazyflie_y.setZero(3,3);rotationB2C_crazyflie_z.setZero(3,3);
    measure_vector.setZero(measurementsize);
    current_time = ros::Time::now();
    previous_time = ros::Time::now();

    ////////////////////UKF initialization//////////////////
    initialize();
    //set initial value of state
    ros::param::get("~x_init_0", x(0));
    ros::param::get("~x_init_1", x(1));
    ros::param::get("~x_init_2", x(2));
    ros::param::get("~x_init_3", x(3));
    //set initial P matrix
    //increase the initial value of P can increase the speed of convergence
    P_init.setZero(statesize,statesize);
    ros::param::get("~P_init_0", P_init(0,0));
    ros::param::get("~P_init_1", P_init(1,1));
    ros::param::get("~P_init_2", P_init(2,2));
    ros::param::get("~P_init_3", P_init(3,3));
    P = P_init;             
    //set measurement noise
    measurement_noise.setZero(measurementsize,measurementsize);
    measurement_noise = 1* Eigen::MatrixXd::Identity(measurementsize,measurementsize);
    ros::param::get("~measurement_noise_0", measurement_noise(0,0));
    ros::param::get("~measurement_noise_1", measurement_noise(1,1));
    ros::param::get("~measurement_noise_2", measurement_noise(2,2));
    ros::param::get("~measurement_noise_3", measurement_noise(3,3));
    R = measurement_noise;             
    //set process noise
    process_noise.setZero(statesize,statesize);
    process_noise = 1* Eigen::MatrixXd::Identity(statesize,statesize);
    ros::param::get("~process_noise_0", process_noise(0,0));
    ros::param::get("~process_noise_1", process_noise(1,1));
    ros::param::get("~process_noise_2", process_noise(2,2));
    ros::param::get("~process_noise_3", process_noise(3,3));
    Q = process_noise;              
    /////////////////////////////////////////////////////

    while(ros::ok()){
        current_time = ros::Time::now();
        dt = current_time.toSec() - previous_time.toSec();
        previous_time = current_time;
        nh.getParam("/ukf_flag",ukf_flag);
        ////////////////////Update ground truth//////////////////
        rs_1 << crazyflie_1_mocap.pose.position.x, crazyflie_1_mocap.pose.position.y, crazyflie_1_mocap.pose.position.z;
        rs_2 << crazyflie_2_mocap.pose.position.x, crazyflie_2_mocap.pose.position.y, crazyflie_2_mocap.pose.position.z;
        rs_3 << crazyflie_3_mocap.pose.position.x, crazyflie_3_mocap.pose.position.y, crazyflie_3_mocap.pose.position.z;
        r_target <<target_pose.pose.position.x, target_pose.pose.position.y, model_height/2;
        r_target_last = r_target;
        rpy_crazyflie_1 = quaternionToRPY(crazyflie_1_mocap.pose.orientation.x,crazyflie_1_mocap.pose.orientation.y,crazyflie_1_mocap.pose.orientation.z,crazyflie_1_mocap.pose.orientation.w);
        if(rpy_crazyflie_1.yaw>2*pi) rpy_crazyflie_1.yaw = rpy_crazyflie_1.yaw - 2*pi;
        else if(rpy_crazyflie_1.yaw<0) rpy_crazyflie_1.yaw = rpy_crazyflie_1.yaw + 2*pi;
        ///////////////////////////////////////////////////////////////

        ////////////////////Coordination transformation//////////////////
        //global frame to crazyflie_1 body frame
        rotation_crazyflie_1_x <<  1,                0,                 0,
                       0, cos(rpy_crazyflie_1.roll), sin(rpy_crazyflie_1.roll),
                       0, -sin(rpy_crazyflie_1.roll), cos(rpy_crazyflie_1.roll);
        rotation_crazyflie_1_y << cos(rpy_crazyflie_1.pitch), 0, -sin(rpy_crazyflie_1.pitch),
                               0,         1,       0,
                    sin(rpy_crazyflie_1.pitch), 0, cos(rpy_crazyflie_1.pitch);
        rotation_crazyflie_1_z << cos(rpy_crazyflie_1.yaw), sin(rpy_crazyflie_1.yaw),    0,
                     -sin(rpy_crazyflie_1.yaw), cos(rpy_crazyflie_1.yaw),    0,
                            0,                0,              1;
        //crazyflie body frame to camera frame
        rotationB2C_crazyflie_x <<  1,      0,       0,
                        0, cos(0),  sin(0),
                        0, -sin(0), cos(0);
        rotationB2C_crazyflie_y <<  cos(pi/2), 0, -sin(pi/2),
                            0,       1,        0,
                          sin(pi/2), 0, cos(pi/2);
        rotationB2C_crazyflie_z <<  cos(-pi/2), sin(-pi/2),  0,
                         -sin(-pi/2), cos(-pi/2),  0,
                                0,       0,      1;
        rot_g2c.setZero(3,3);
        rot_g2c = (rotationB2C_crazyflie_z*rotationB2C_crazyflie_y*rotation_crazyflie_1_x*rotation_crazyflie_1_y*rotation_crazyflie_1_z);
       ///////////////////////////////////////////////////////////////
       
        if(ukf_flag){
            optimal_sensing_mode = true;
            /////////////////////UKF process//////////////////
            if(initial_finish_flag){
                predict();

                //calculate measurement noise
                std::random_device rd_y1;
                std::default_random_engine disturbance_generator_y1= std::default_random_engine(rd_y1());
                std::normal_distribution<float> disturbance_distribution_y1(0.0,sqrt(R(0,0)));
                float rd_noise_y1 = disturbance_distribution_y1(disturbance_generator_y1);
                std::random_device rd_y2;
                std::default_random_engine disturbance_generator_y2= std::default_random_engine(rd_y2());
                std::normal_distribution<float> disturbance_distribution_y2(0.0,sqrt(R(1,1)));
                float rd_noise_y2 = disturbance_distribution_y2(disturbance_generator_y2);
                std::random_device rd_y3;
                std::default_random_engine disturbance_generator_y3= std::default_random_engine(rd_y3());
                std::normal_distribution<float> disturbance_distribution_y3(0.0,sqrt(R(2,2)));
                float rd_noise_y3 = disturbance_distribution_y3(disturbance_generator_y3);
                std::random_device rd_y4;
                std::default_random_engine disturbance_generator_y4= std::default_random_engine(rd_y4());
                std::normal_distribution<float> disturbance_distribution_y4(0.0,sqrt(R(3,3)));
                float rd_noise_y4 = disturbance_distribution_y4(disturbance_generator_y4);
                
                //save measurement states for correct
                center_u_1_model = (double)-fx*(cos(rpy_crazyflie_1.yaw)*(rs_1(1)-r_target(1))-sin(rpy_crazyflie_1.yaw)*(rs_1(0)-r_target(0)))/(cos(rpy_crazyflie_1.yaw)*(rs_1(0)-r_target(0))+sin(rpy_crazyflie_1.yaw)*(rs_1(1)-r_target(1))) + (double)cx +rd_noise_y1;
                center_v_1_model = (double)-fy*(rs_1(2)-r_target(2))/(cos(rpy_crazyflie_1.yaw)*(rs_1(0)-r_target(0))+sin(rpy_crazyflie_1.yaw)*(rs_1(1)-r_target(1))) + (double)cy +rd_noise_y2;
                rho_2 = sqrt(pow(rs_2(0)-r_target(0),2) + pow(rs_2(1)-r_target(1),2))+rd_noise_y3;
                beta_3 = atan2(rs_3(1)-r_target(1),rs_3(0)-r_target(0))+rd_noise_y4;
                measure_vector<<(double)center_u_1_model, (double)center_v_1_model, (double)rho_2, (double)beta_3;
                correct(measure_vector);
            }
            ///////////////////////////////////////////////////////////////

            ////////////////////Publish UKF data//////////////////
            //true state
            true_value.cmode.data = ukf_flag;
            true_value.target_pose.x = (float)(r_target(0));
            true_value.target_pose.y = (float)(r_target(1));
            true_value.target_vel.x = target_gvel(0);
            true_value.target_vel.y = target_gvel(1);
            true_pub.publish(true_value);
            //estimated state
            estimate_value.cmode.data = ukf_flag;
            estimate_value.target_pose.x = x(0);
            estimate_value.target_pose.y = x(1);
            estimate_value.target_vel.x = x(2);
            estimate_value.target_vel.y = x(3);
            estimate_pub.publish(estimate_value);
            //error state
            error_value.cmode.data = ukf_flag;
            error_value.target_pose.x = true_value.target_pose.x - estimate_value.target_pose.x;
            error_value.target_pose.y = true_value.target_pose.y - estimate_value.target_pose.y;
            error_value.target_vel.x = true_value.target_vel.x - estimate_value.target_vel.x;
            error_value.target_vel.y = true_value.target_vel.y - estimate_value.target_vel.y;
            error_pub.publish(error_value);
            //determinant of state covariance (position only)
            det_P_msg.data = det_P;
            det_P_pub.publish(det_P_msg);
            //measurment state
            measurement_value.center_u_1.data = center_u_1_model;
            measurement_value.center_v_1.data = center_v_1_model;
            measurement_value.rho_2.data = rho_2;
            measurement_value.beta_3.data = beta_3;
            measurement_pub.publish(measurement_value);
            //full state covariance
            tf::matrixEigenToMsg(P,P_final);
            covariance_pub.publish(P_final);
            ///////////////////////////////////////////////////////////////
            initial_finish_flag = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
