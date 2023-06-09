#!/usr/bin/env python3

import rospy
import sys
import numpy as np

from pdfc_fr.Algorithm import Algorithm
from pdfc_fr.NodeHelper import NodeHelper
from pdfc_fr.srv import Commander
from geometry_msgs.msg import Twist
from pdfc_fr.msg import Map
from pdfc_fr.GradientMap import GradientMap
from pdfc_fr.EKF import EKF

def Node(id:int):

    SR = rospy.get_param("agent_sensing_radius")
    n_agents = rospy.get_param("number_of_agents")
    max_speed = rospy.get_param("agent_max_vel")
    freq = rospy.get_param("compute_frequency")
    cf_radius = rospy.get_param("cf_radius")
    noise_std = rospy.get_param("noise_std")
    cw = np.asarray(rospy.get_param("control_weights"))

    collision_params = {'max_speed': max_speed, 'time_step_size': 1 / freq, 'cf_radius': cf_radius, 'noise_std': noise_std}
    pub_vel = rospy.Publisher('node'+str(id)+'/cmd_vel', Twist, queue_size=10)

    rospy.init_node('node'+str(id))
    rate = rospy.Rate(100)
    
    map = GradientMap()
    rospy.Subscriber('publisher/map', Map, map.map_update)
    # Kalman filter
    f = EKF(dim_x=4, dim_z=2)
    al = Algorithm(id, SR, map, cw, collision_params, n_agents)
    nh = NodeHelper(n_agents, SR, id, f, al, pub_vel)
    rospy.Service('node'+str(id)+'/commander', Commander, nh.handle_commander)

    while (not nh.run) or (rospy.Time.secs == 0):
        rate.sleep()

    # state vector [x, y, Vx, Vy]
    f.x_hat = np.array([[nh.get_curr_pos()[id-1,0]], [nh.get_curr_pos()[id-1,1]], [0], [0]])


    f.H = np.array([[1, 0, 0, 0],[0, 1, 0, 0]])
    f.Q *= 0.1
    f.R *= noise_std * 2
    f.B[2,0] = 1
    f.B[3,1] = 1
    f.F[0,2] = 0.05
    f.F[1,3] = 0.05

    #nh.p = [f.x_hat[0,0], f.x_hat[1,0], nh.get_curr_pos()[nh.id-1,2]]
    #nh.publish_estimated_pos(nh.p)
        
    print("Node" + str(id) + " starting")

    while nh.run:
        rate.sleep()

    print("Node" + str(id) + " stopping")    

if __name__ == '__main__':
    try:
        Node(int(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
