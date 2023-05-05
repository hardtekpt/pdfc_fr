#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Pose
from pdfc_fr.srv import CommanderResponse
from pdfc_fr.msg import TowerData, TowerDataArray
class NodeHelper:

    def __init__(self, n_agents, SR, id, f, al, pub_vel):

        self.n_agents = n_agents
        self.SR = SR
        self.id = id
        self.run = False
        self.f = f
        self.al = al

        self.max_speed = rospy.get_param("agent_max_vel")

        self.pub_vel = pub_vel
        self.pos_subs = np.ndarray((n_agents,),rospy.Subscriber)
        self.vel_subs = np.ndarray((n_agents,),rospy.Subscriber)
        #self.tower_data_subs = np.ndarray((n_agents,),rospy.Subscriber)

        initial_pos_val = Pose()
        initial_pos_val.position.x = None
        initial_pos_val.position.y = None
        initial_pos_val.position.z = None
        self.curr_pos = np.full((n_agents,3), None)

        initial_vel_val = Twist()
        initial_vel_val.linear.x = None
        initial_vel_val.linear.y = None
        initial_vel_val.linear.z = None
        self.curr_vel = np.full((n_agents,3), None)
        
        self.dist = np.zeros((n_agents,))
        self.aux_dist = np.zeros((n_agents,))

        #for i in range(n_agents):
            #self.pos_subs[i] = rospy.Subscriber('node'+str(i+1)+'/pose', PoseStamped, self.save_pos, (i,))
            #self.vel_subs[i] = rospy.Subscriber('node'+str(i+1)+'/twist', TwistStamped, self.save_vel, (i,))
        self.tower_data_sub = rospy.Subscriber('node'+str(id)+'/tower_data', TowerDataArray, self.save_tower_data)

        self.pub_pos_estimate = rospy.Publisher('node'+str(id)+'/pose_estimate', PoseStamped, queue_size=10)

        
        self.vel = np.zeros((2,))


    def save_pos(self, p:PoseStamped, args):
        
        i = args[0]
        self.curr_pos[i] = p.pose

    def save_vel(self, v:TwistStamped, args):

        i = args[0]
        self.curr_vel[i] = v.twist

    def save_tower_data(self, tower_data_array:TowerData):

        #print(self.id -1, tower_data)
        #print("node ", self.id, len(tower_data_array.data))

        reset = False

        for j in range(len(tower_data_array.data)):

            tower_data = tower_data_array.data[j]
        
            i = tower_data.id
            self.curr_pos[i] = tower_data.p
            self.curr_vel[i] = tower_data.v
            
            if self.run:
                self.dist[i] = 1
                self.aux_dist[i] = 1

            if tower_data_array.target == self.id-1:
                reset = True
        
        #self.p = [self.get_curr_pos()[self.id-1,0],self.get_curr_pos()[self.id-1,1],self.get_curr_pos()[self.id-1,2]]

        if self.run == True:

            
            # run algorithm
            curr_pos = self.curr_pos
            curr_vel = self.curr_vel

            self.f.predict_step(self.vel)
            self.f.update_step(curr_pos[self.id-1,0:2])  

            #print(self.id, tower_data.id,  curr_pos[self.id-1,0:2], tower_data.p)      

            self.p = [self.f.x_hat[0,0], self.f.x_hat[1,0], curr_pos[self.id-1,2]]

            #self.p = [curr_pos[self.id-1,0], curr_pos[self.id-1,1], curr_pos[self.id-1,2]]

            #print(self.id-1, self.f.x_hat, self.p)

            #print("node ", self.id, " neighbours ", np.where(self.dist == 1)[0])


            self.vel = self.al.update_movement(curr_pos[:,0:2], curr_vel[:,0:2], self.dist)
            self.vel = self.al.check_for_obstacle_collisions(self.vel, self.p[0:2])
            self.vel = self.al.check_for_boundaries(self.vel, self.p[0:2])

            aux = self.vel 
            self.vel = self.al.check_for_agent_collisions(self.vel, curr_pos[:,0:2], self.dist)

            #print("node ", self.id, aux==self.vel)

            if np.linalg.norm(self.vel) > self.max_speed:
                self.vel = self.al.normalize(self.vel) * self.max_speed
            
            self.publish_estimated_pos(self.p)

            v = Twist()
            v.linear.x = self.vel[0]
            v.linear.y = self.vel[1]
            v.linear.z = 0
            self.pub_vel.publish(v)

            rospy.sleep(self.al.collision_params['time_step_size']*0.5)

            v.linear.x = 0
            v.linear.y = 0
            v.linear.z = 0
            self.pub_vel.publish(v)

        if reset:
            for k in range(len(self.aux_dist)):
                if self.aux_dist[k] == 0:
                    self.dist[k] = 0
            self.aux_dist = np.zeros((self.n_agents,))

            #self.dist = np.zeros((self.n_agents,))
        

    def get_curr_pos(self):

        return self.curr_pos

    def get_curr_vel(self):

        return self.curr_vel

    def get_dist(self):

        # for i in range(self.n_agents):
        #     my_pose = Pose()
        #     my_pose = self.curr_pos[self.id - 1]
        #     possible_neighbour_pose = Pose()
        #     possible_neighbour_pose = self.curr_pos[i]

        #     my_pos = np.asarray([my_pose.position.x, my_pose.position.y, my_pose.position.z])
        #     possible_neighbour_pos = np.asarray([possible_neighbour_pose.position.x, possible_neighbour_pose.position.y, possible_neighbour_pose.position.z])
        #     d = np.linalg.norm(my_pos - possible_neighbour_pos)
        #     if d <= self.SR:
        #         self.dist[i] = 1

        return self.dist

    def corrupt_pos(self, data, std_d):

        corrupted_data = np.ndarray((len(data),3))
        noise = np.random.normal(0,std_d,3)

        for i in range(len(noise)):
            if noise[i] > 3 * std_d:
                noise[i] = 3 * std_d
            if noise[i] < -3 * std_d:
                noise[i] = -3 * std_d
        
        for i in range(len(data)):
            pose = Pose()
            pose = data[i]
            corrupted_data[i,0] = pose.position.x + noise[0]
            corrupted_data[i,1] = pose.position.y + noise[1]
            corrupted_data[i,2] = pose.position.z + noise[2]
        return corrupted_data

    def corrupt_vel(self, data, std_d):

        corrupted_data = np.ndarray((len(data),3))
        noise = np.random.normal(0,std_d,3)

        for i in range(len(noise)):
            if noise[i] > 3 * std_d:
                noise[i] = 3 * std_d
            if noise[i] < -3 * std_d:
                noise[i] = -3 * std_d

        for i in range(len(data)):
            twist = Twist()
            twist = data[i]
            corrupted_data[i,0] = twist.linear.x + noise[0]
            corrupted_data[i,1] = twist.linear.y + noise[1]
            corrupted_data[i,2] = twist.linear.z + noise[2]
        return corrupted_data

    def handle_commander(self, req):
        
        if req.start:
            self.run = True
        if req.stop:
            self.run = False
        return CommanderResponse(True)
    
    def publish_estimated_pos(self, p):

        pose_estimate = PoseStamped()
        pose_estimate.header.stamp = rospy.Time.now()
        pose_estimate.pose.position.x = p[0]
        pose_estimate.pose.position.y = p[1]
        pose_estimate.pose.position.z = p[2]
        self.pub_pos_estimate.publish(pose_estimate)
    