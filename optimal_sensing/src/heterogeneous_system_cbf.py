#!/usr/bin/python

import rospy
from math import sin,cos,sqrt,atan2,acos,pi
import numpy as np
import threading
from scipy.optimize import minimize
from crazyflie_driver.msg import VelocityWorld
from optimal_sensing.msg import output_uavs_pose
from optimal_sensing.msg import output_uavs_controller
from optimal_sensing.msg import output_ukf
from optimal_sensing.msg import output_FOV
from optimal_sensing.msg import output_tracking_distance
from optimal_sensing.msg import output_inter_uavs_distance
from optimal_sensing.msg import output_occlusion
from optimal_sensing.msg import output_measurement
from std_msgs.msg import Float64MultiArray

fx,fy,lx,ly = 381,381,640,480
target_length,target_width,target_height = 0.138,0.178,0.16
Pc,Pr,Pb,theta_c,u_desired,u1_bar, v1_bar,A,b = None,None,None,None,None,None,None,None,None
Pq_hat,Vq_hat,Pq,Vq = None,None,None,None
uavs_pose_flag = False
ukf_flag = False
uavs_controller_flag = False
optimal_sensing_mode = False
tracking_distance = output_tracking_distance()
tracking_distance_hat = output_tracking_distance()
FOV = output_FOV()
FOV_hat = output_FOV()
inter_uavs_distance = output_inter_uavs_distance()
occlusion = output_occlusion()
camera_cmd = VelocityWorld()
ranging_cmd = VelocityWorld()
bearing_cmd = VelocityWorld()

d_safe_car = 0.5
d_measuring_car = 2.0
d_safe_uav = 0.3
theta_fov_fix = 30
height_safe = 0.2
height_conmunication = 1.1
bound_x = 1.2 
bound_y = 1.7
theta_occ = 15
gamma_safe_car = 1
gamma_measuring = 1
gamma_safe_uav = 1.0
gamma_fov = 0.01
gamma_height = 1
gamma_bound_x = 1
gamma_bound_y = 1
gamma_occ = 1

def object_fun(x):
        return sqrt((x[0]-u_desired[0])**2+(x[1]-u_desired[1])**2+(x[2]-u_desired[2])**2+(x[3]-u_desired[3])**2+(x[4]-u_desired[4])**2\
		+(x[5]-u_desired[5])**2+(x[6]-u_desired[6])**2+(x[7]-u_desired[7])**2+(x[8]-u_desired[8])**2+0.0001*(x[9]-u_desired[9])**2)

def cons_maker(i=0):
	def constraint(x):
		return b[i] - A[i,0]*x[0] - A[i,1]*x[1] - A[i,2]*x[2] - A[i,3]*x[3] - A[i,4]*x[4] - A[i,5]*x[5] - A[i,6]*x[6] - A[i,7]*x[7] - A[i,8]*x[8] - A[i,9]*x[9] - x[i+10]
	return constraint

def cons_maker1(i=0):
	def constraint(x):
		return x[i+10]
	return constraint

def uavs_pose_cb(msg):
	global Pc,Pr,Pb,theta_c,uavs_pose_flag
	Pc = np.array([msg.rx_c.data, msg.ry_c.data, msg.rz_c.data])
	Pr = np.array([msg.rx_r.data, msg.ry_r.data, msg.rz_r.data])
	Pb = np.array([msg.rx_b.data, msg.ry_b.data, msg.rz_b.data])
	theta_c = msg.theta_c.data
	if msg != None:
		uavs_pose_flag = True

def target_cb(msg):
	global Pq,Vq
	Pq = np.array([msg.target_pose.x, msg.target_pose.y, target_height/2])
	Vq = np.array([msg.target_vel.x, msg.target_vel.y, 0])
	

def ukf_cb(msg):
	global Pq_hat,Vq_hat,optimal_sensing_mode,ukf_flag
	Pq_hat = np.array([msg.target_pose.x, msg.target_pose.y, target_height/2])
	Vq_hat = np.array([msg.target_vel.x, msg.target_vel.y, 0])
	optimal_sensing_mode = msg.cmode.data
	if msg != None:
		ukf_flag = True
	

def uavs_controller_cb(msg):
	global u_desired,uavs_controller_flag
	u_desired = np.array([msg.vx_1.data, msg.vy_1.data, msg.vz_1.data,msg.vx_2.data, msg.vy_2.data, msg.vz_2.data,msg.vx_3.data, msg.vy_3.data, msg.vz_3.data,msg.wz_1.data, msg.wz_2.data, msg.wz_3.data])
	if msg != None:
		uavs_controller_flag = True

def measurement_cb(msg):
	global u1_bar, v1_bar
	u1_bar = msg.center_u_1.data
	v1_bar = msg.center_v_1.data
		

def CBF():
	global A,b,tracking_distance,tracking_distance_hat,inter_uavs_distance,FOV,FOV_hat,occlusion	
	r_cq_hat= np.array([Pc[0] - Pq_hat[0],Pc[1] - Pq_hat[1],Pc[2] - Pq_hat[2]])
	r_cq_xy_hat= np.array([Pc[0] - Pq_hat[0],Pc[1] - Pq_hat[1],0])
	r_rq_hat= np.array([Pr[0] - Pq_hat[0],Pr[1] - Pq_hat[1],Pr[2] - Pq_hat[2]])
	r_bq_hat= np.array([Pb[0] - Pq_hat[0],Pb[1] - Pq_hat[1],Pb[2] - Pq_hat[2]])
	r_qc_hat = -r_cq_hat
	r_qc_xy_hat = -r_cq_xy_hat
	r_qr_hat = -r_rq_hat
	r_qb_hat = -r_bq_hat
	r_qc_norm_hat = np.linalg.norm(r_qc_hat)
	r_qr_norm_hat = np.linalg.norm(r_qr_hat)
	r_qb_norm_hat = np.linalg.norm(r_qb_hat)
	#ground truth
	r_cr= np.array([Pc[0] - Pr[0],Pc[1] - Pr[1],Pc[2] - Pr[2]])
	r_cb= np.array([Pc[0] - Pb[0],Pc[1] - Pb[1],Pc[2] - Pb[2]])
	r_rb= np.array([Pr[0] - Pb[0],Pr[1] - Pb[1],Pr[2] - Pb[2]])
	r_rc = -r_cr
	r_bc = -r_cb
	r_br = -r_rb
	r_cr_norm = np.linalg.norm(r_cr)
	r_cb_norm = np.linalg.norm(r_cb)
	r_rb_norm = np.linalg.norm(r_rb)
	r_rc_norm = np.linalg.norm(r_rc)
	r_bc_norm = np.linalg.norm(r_bc)
	r_br_norm = np.linalg.norm(r_br)
	nc = np.array([cos(theta_c),sin(theta_c),0])
	nc_dot = np.array([-sin(theta_c),cos(theta_c),0])
	r_qc = np.array([Pq[0] - Pc[0],Pq[1] - Pc[1],Pq[2] - Pc[2]])
	r_qr = np.array([Pq[0] - Pr[0],Pq[1] - Pr[1],Pq[2] - Pr[2]])
	r_qb = np.array([Pq[0] - Pb[0],Pq[1] - Pb[1],Pq[2] - Pb[2]])
	r_qc_norm = np.linalg.norm(r_qc)
	r_qr_norm = np.linalg.norm(r_qr)
	r_qb_norm = np.linalg.norm(r_qb)
	tracking_distance.rho_1.data = r_qc_norm_hat
	tracking_distance.rho_2.data = r_qr_norm_hat
	tracking_distance.rho_3.data = r_qb_norm_hat
	tracking_distance_pub.publish(tracking_distance)
	tracking_distance_hat.rho_1.data = r_qc_norm
	tracking_distance_hat.rho_2.data = r_qr_norm
	tracking_distance_hat.rho_3.data = r_qb_norm
	tracking_distance_hat_pub.publish(tracking_distance_hat)
	inter_uavs_distance.rho_12.data = r_cr_norm
	inter_uavs_distance.rho_13.data = r_cb_norm
	inter_uavs_distance.rho_23.data = r_br_norm
	inter_uavs_distance_pub.publish(inter_uavs_distance)
	image_radius = sqrt((u1_bar-lx/2)**2+(v1_bar-ly/2)**2)
	theta_qc_hat =  180*atan2(image_radius,fx)/pi
	theta_qc =  180*acos(np.dot(nc,r_qc)/r_qc_norm)/pi
	FOV.theta_qc.data = theta_qc
	theta_FOV_pub.publish(FOV)
	FOV_hat.theta_qc.data = theta_qc_hat
	theta_FOV_hat_pub.publish(FOV_hat)
	# theta_occ_cr = (180*acos(np.dot(r_rc,r_qc)/r_rc_norm/r_qc_norm)/pi)
	# theta_occ_cb = (180*acos(np.dot(r_bc,r_qc)/r_bc_norm/r_qc_norm)/pi)
	# theta_occ_rc = (180*acos(np.dot(r_cr,r_qr)/r_cr_norm/r_qr_norm)/pi)
	# theta_occ_rb = (180*acos(np.dot(r_br,r_qr)/r_br_norm/r_qr_norm)/pi)
	# theta_occ_bc = (180*acos(np.dot(r_cb,r_qb)/r_cb_norm/r_qb_norm)/pi)
	# theta_occ_br = (180*acos(np.dot(r_rb,r_qb)/r_rb_norm/r_qb_norm)/pi)
	# occlusion.theta_occ_cr.data = theta_occ_cr
	# occlusion.theta_occ_cb.data = theta_occ_cb
	# occlusion.theta_occ_rc.data = theta_occ_rc
	# occlusion.theta_occ_rb.data = theta_occ_rb
	# occlusion.theta_occ_bc.data = theta_occ_bc
	# occlusion.theta_occ_br.data = theta_occ_br
	# theta_occ_pub.publish(occlusion)
	A = np.array([[-2*r_cq_hat[0], -2*r_cq_hat[1], -2*r_cq_hat[2]]+[0]*7, \
				  [0]*3+[-2*r_rq_hat[0], -2*r_rq_hat[1], -2*r_rq_hat[2]]+[0]*4, \
				  [0]*6+[-2*r_bq_hat[0], -2*r_bq_hat[1], -2*r_bq_hat[2]]+[0]*1, \
				  [2*r_cq_hat[0], 2*r_cq_hat[1], 2*r_cq_hat[2]]+[0]*7, \
				  [0]*3+[2*r_rq_hat[0], 2*r_rq_hat[1], 2*r_rq_hat[2]]+[0]*4, \
				  [0]*6+[2*r_bq_hat[0], 2*r_bq_hat[1], 2*r_bq_hat[2]]+[0]*1, \
				  [-2*r_cr[0], -2*r_cr[1], -2*r_cr[2]]+[0]*7, \
				  [-2*r_cb[0], -2*r_cb[1], -2*r_cb[2]]+[0]*7, \
				  [0]*3+[-2*r_rc[0], -2*r_rc[1], -2*r_rc[2]]+[0]*4, \
				  [0]*3+[-2*r_rb[0], -2*r_rb[1], -2*r_rb[2]]+[0]*4, \
				  [0]*6+[-2*r_bc[0], -2*r_bc[1], -2*r_bc[2]]+[0]*1, \
				  [0]*6+[-2*r_br[0], -2*r_br[1], -2*r_br[2]]+[0]*1, \
				  np.append(np.append(-(np.dot(nc,r_qc_hat)*r_qc_hat/r_qc_norm_hat**3-nc/r_qc_norm_hat)/sqrt(1 - np.dot(nc,r_qc_hat)**2/r_qc_norm_hat**2),[0]*6),-np.dot(nc_dot,r_qc_hat)/r_qc_norm_hat/sqrt(1 - np.dot(nc,r_qc_hat)**2/r_qc_norm_hat**2)), \
				  [0]*2+[-1]+[0]*7, \
				  [0]*5+[-1]+[0]*4, \
				  [0]*8+[-1]+[0]*1, \
				  [0]*2+[1]+[0]*7, \
				  [0]*5+[1]+[0]*4, \
				  [0]*8+[1]+[0]*1, \
				  [2*Pc[0]]+[0]*9\
				  #np.append(np.append(-((r_qc-r_rc)/r_rc_norm/r_qc_norm_hat+(np.dot(r_rc,r_qc)*r_rc)/r_rc_norm**3/r_qc_norm_hat+(np.dot(r_rc,r_qc)*r_qc)/r_rc_norm/r_qc_norm_hat**3)/sqrt(1 - np.dot(r_rc,r_qc)**2/r_rc_norm**2/r_qc_norm_hat**2), \
				  #-(r_qc/r_rc_norm/r_qc_norm_hat-(np.dot(r_rc,r_qc)*r_rc)/r_rc_norm**3/r_qc_norm_hat)/sqrt(1 - np.dot(r_rc,r_qc)**2/r_rc_norm**2/r_qc_norm_hat**2)),[0]*4), \
				  #np.append(np.append(-((r_qc-r_bc)/r_bc_norm/r_qc_norm_hat+(np.dot(r_bc,r_qc)*r_bc)/r_bc_norm**3/r_qc_norm_hat+(np.dot(r_bc,r_qc)*r_qc)/r_bc_norm/r_qc_norm_hat**3)/sqrt(1 - np.dot(r_bc,r_qc)**2/r_bc_norm**2/r_qc_norm_hat**2),[0]*3),\
				  #np.append(-(r_qc/r_bc_norm/r_qc_norm_hat-(np.dot(r_bc,r_qc)*r_bc)/r_bc_norm**3/r_qc_norm_hat)/sqrt(1 - np.dot(r_bc,r_qc)**2/r_bc_norm**2/r_qc_norm_hat**2),[0]*1)), \
				  #np.append(np.append(-((r_qr-r_cr)/r_cr_norm/r_qr_norm_hat+(np.dot(r_cr,r_qr)*r_cr)/r_cr_norm**3/r_qr_norm_hat+(np.dot(r_cr,r_qr)*r_qr)/r_cr_norm/r_qr_norm_hat**3)/sqrt(1 - np.dot(r_cr,r_qr)**2/r_cr_norm**2/r_qr_norm_hat**2), \
				  #-(r_qr/r_cr_norm/r_qr_norm_hat-(np.dot(r_cr,r_qr)*r_cr)/r_cr_norm**3/r_qr_norm_hat)/sqrt(1 - np.dot(r_cr,r_qr)**2/r_cr_norm**2/r_qr_norm_hat**2)),[0]*4), \
				  #np.append(np.append([0]*3,-((r_qr-r_br)/r_br_norm/r_qr_norm_hat+(np.dot(r_br,r_qr)*r_br)/r_br_norm**3/r_qr_norm_hat+(np.dot(r_br,r_qr)*r_qr)/r_br_norm/r_qr_norm_hat**3)/sqrt(1 - np.dot(r_br,r_qr)**2/r_br_norm**2/r_qr_norm_hat**2)), \
				  #np.append(-(r_qr/r_br_norm/r_qr_norm_hat-(np.dot(r_br,r_qr)*r_br)/r_br_norm**3/r_qr_norm_hat)/sqrt(1 - np.dot(r_br,r_qr)**2/r_br_norm**2/r_qr_norm_hat**2),[0]*1)), \
				  #np.append(np.append(-((r_qb-r_cb)/r_cb_norm/r_qb_norm_hat+(np.dot(r_cb,r_qb)*r_cb)/r_cb_norm**3/r_qb_norm_hat+(np.dot(r_cb,r_qb)*r_qb)/r_cb_norm/r_qb_norm_hat**3)/sqrt(1 - np.dot(r_cb,r_qb)**2/r_cb_norm**2/r_qb_norm_hat**2),[0]*3), \
				  #np.append(-(r_qb/r_cb_norm/r_qb_norm_hat-(np.dot(r_cb,r_qb)*r_cb)/r_cb_norm**3/r_qb_norm_hat)/sqrt(1 - np.dot(r_cb,r_qb)**2/r_cb_norm**2/r_qb_norm_hat**2),[0]*1)), \
				  #np.append(np.append([0]*3,-((r_qb-r_rb)/r_rb_norm/r_qb_norm_hat+(np.dot(r_rb,r_qb)*r_rb)/r_rb_norm**3/r_qb_norm_hat+(np.dot(r_rb,r_qb)*r_qb)/r_rb_norm/r_qb_norm_hat**3)/sqrt(1 - np.dot(r_rb,r_qb)**2/r_rb_norm**2/r_qb_norm_hat**2)), \
				  #np.append(-(r_qb/r_rb_norm/r_qb_norm_hat-(np.dot(r_rb,r_qb)*r_rb)/r_rb_norm**3/r_qb_norm_hat)/sqrt(1 - np.dot(r_rb,r_qb)**2/r_rb_norm**2/r_qb_norm_hat**2),[0]*1)) \
				  ])

	b = np.array([gamma_safe_car*(r_qc_norm_hat**2 - d_safe_car**2)-2*np.dot(-r_qc_hat,Vq_hat), \
				  gamma_safe_car*(r_qr_norm_hat**2 - d_safe_car**2)-2*np.dot(-r_qr_hat,Vq_hat), \
				  gamma_safe_car*(r_qb_norm_hat**2 - d_safe_car**2)-2*np.dot(-r_qb_hat,Vq_hat), \
				  gamma_measuring*(d_measuring_car**2 - r_qc_norm_hat**2)+2*np.dot(-r_qc_hat,Vq_hat), \
				  gamma_measuring*(d_measuring_car**2 - r_qr_norm_hat**2)+2*np.dot(-r_qr_hat,Vq_hat), \
				  gamma_measuring*(d_measuring_car**2 - r_qb_norm_hat**2)+2*np.dot(-r_qb_hat,Vq_hat), \
				  (gamma_safe_uav/2)*(r_cr_norm**2 - d_safe_uav**2), \
				  (gamma_safe_uav/2)*(r_cb_norm**2 - d_safe_uav**2), \
				  (gamma_safe_uav/2)*(r_rc_norm**2 - d_safe_uav**2), \
				  (gamma_safe_uav/2)*(r_rb_norm**2 - d_safe_uav**2), \
				  (gamma_safe_uav/2)*(r_bc_norm**2 - d_safe_uav**2), \
				  (gamma_safe_uav/2)*(r_br_norm**2 - d_safe_uav**2), \
				  gamma_fov*(theta_fov_fix - theta_qc_hat)-np.dot((np.dot(nc,r_qc_hat)*r_qc_hat/r_qc_norm_hat**3-nc/r_qc_norm_hat)/sqrt(1 - np.dot(nc,r_qc_hat)**2/r_qc_norm_hat**2),Vq_hat),\
				  gamma_height*(Pc[2] - height_safe), \
				  gamma_height*(Pr[2] - height_safe), \
				  gamma_height*(Pb[2] - height_safe), \
				  gamma_height*(height_conmunication-Pc[2] ), \
				  gamma_height*(height_conmunication-Pr[2]), \
				  gamma_height*(height_conmunication-Pb[2]) \
				  #gamma_occ*(theta_occ - theta_occ_cr)+(r_rc/r_rc_norm/r_qc_norm_hat - np.dot(r_rc,r_qc)*r_rc/r_rc_norm/r_qc_norm_hat**3)/(sqrt(1 - np.dot(r_rc,r_qc)**2/r_rc_norm**2/r_qc_norm_hat**2)), \
				  #gamma_occ*(theta_occ - theta_occ_cb)+(r_bc/r_bc_norm/r_qc_norm_hat - np.dot(r_bc,r_qc)*r_bc/r_bc_norm/r_qc_norm_hat**3)/(sqrt(1 - np.dot(r_bc,r_qc)**2/r_bc_norm**2/r_qc_norm_hat**2)), \
				  #gamma_occ*(theta_occ - theta_occ_rc)+(r_cr/r_cr_norm/r_qr_norm_hat - np.dot(r_cr,r_qr)*r_cr/r_cr_norm/r_qr_norm_hat**3)/(sqrt(1 - np.dot(r_cr,r_qr)**2/r_cr_norm**2/r_qr_norm_hat**2)), \
				  #gamma_occ*(theta_occ - theta_occ_rb)+(r_br/r_br_norm/r_qr_norm_hat - np.dot(r_br,r_qr)*r_br/r_br_norm/r_qr_norm_hat**3)/(sqrt(1 - np.dot(r_br,r_qr)**2/r_br_norm**2/r_qr_norm_hat**2)), \
				  #gamma_occ*(theta_occ - theta_occ_bc)+(r_cb/r_cb_norm/r_qb_norm_hat - np.dot(r_cb,r_qb)*r_cb/r_cb_norm/r_qb_norm_hat**3)/(sqrt(1 - np.dot(r_cb,r_qb)**2/r_cb_norm**2/r_qb_norm_hat**2)), \
				  #gamma_occ*(theta_occ - theta_occ_br)+(r_rb/r_rb_norm/r_qb_norm_hat - np.dot(r_rb,r_qb)*r_rb/r_rb_norm/r_qb_norm_hat**3)/(sqrt(1 - np.dot(r_rb,r_qb)**2/r_rb_norm**2/r_qb_norm_hat**2)) \
				  ])


def	qpsolver():
	global camera_cmd,ranging_cmd,bearing_cmd
	
	cons = []
	
	for i in range (b.size):
		cons.append({'type': 'eq', 'fun': cons_maker(i)})
	for i in range (b.size):
		cons.append({'type': 'ineq', 'fun': cons_maker1(i)})
	
	ini = tuple(np.zeros(b.size + 10))
	bnds = ((-0.3, 0.3),)*9+((-3.0, 3.0),)*1+ ((0, np.inf),)*b.size
	
	optimal = minimize(object_fun, ini, method='SLSQP', bounds=bnds, constraints=cons,options={'maxiter':15}).x
	print(camera_cmd)
	camera_cmd.vel.x = optimal[0]
	camera_cmd.vel.y = optimal[1]
	camera_cmd.vel.z = optimal[2]
	ranging_cmd.vel.x = optimal[3]
	ranging_cmd.vel.y = optimal[4]
	ranging_cmd.vel.z = optimal[5]
	bearing_cmd.vel.x = optimal[6]
	bearing_cmd.vel.y = optimal[7]
	bearing_cmd.vel.z = optimal[8]
	camera_cmd.yawRate = optimal[9]
	ranging_cmd.yawRate = u_desired[10]
	bearing_cmd.yawRate = u_desired[11]

	
	
if __name__ == '__main__':
	try:
		rospy.init_node('controller')
		cf_vel_pub1 = rospy.Publisher('/crazyflie1/cmd_velocity_world', VelocityWorld, queue_size=1)
		cf_vel_pub2 = rospy.Publisher('/crazyflie2/cmd_velocity_world', VelocityWorld, queue_size=1)
		cf_vel_pub3 = rospy.Publisher('/crazyflie3/cmd_velocity_world', VelocityWorld, queue_size=1)
		tracking_distance_pub = rospy.Publisher('/tracking_distance', output_tracking_distance, queue_size=1)
		tracking_distance_hat_pub = rospy.Publisher('/tracking_distance_hat', output_tracking_distance, queue_size=1)
		theta_FOV_pub = rospy.Publisher('/FOV', output_FOV, queue_size=1)
		theta_FOV_hat_pub = rospy.Publisher('/FOV_hat', output_FOV, queue_size=1)
		inter_uavs_distance_pub = rospy.Publisher('/inter_uavs_distance', output_inter_uavs_distance, queue_size=1)
		theta_occ_pub = rospy.Publisher('/occlusion', output_occlusion, queue_size=1)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():			
			rospy.Subscriber("/uavs_pose", output_uavs_pose, uavs_pose_cb)
			rospy.Subscriber("/true_data", output_ukf, target_cb)
			rospy.Subscriber("/estimated_data", output_ukf, ukf_cb)
			rospy.Subscriber("/formation_controller", output_uavs_controller, uavs_controller_cb)	
			rospy.Subscriber("/measurement_data", output_measurement, measurement_cb)	
			if uavs_pose_flag & uavs_controller_flag:
				CBF()
				qpsolver()
				if optimal_sensing_mode:
					cf_vel_pub1.publish(camera_cmd)
					cf_vel_pub2.publish(ranging_cmd)
					cf_vel_pub3.publish(bearing_cmd)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
