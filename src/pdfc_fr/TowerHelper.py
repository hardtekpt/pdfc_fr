#!/usr/bin/env python3

import rospy
import numpy as np
import random
from pdfc_fr.msg import Towers, Tower
import math

class TowerHelper():

    def __init__(self, map_dimensions):

        self.number_of_towers = rospy.get_param("number_of_towers")
        self.tower_angle = rospy.get_param("tower_angle")
        
        self.strip_half_rads = math.radians(self.tower_angle/2)
        self.strip_half_rads_sin = math.sin(self.strip_half_rads)
        self.strip_half_rads_cos = math.cos(self.strip_half_rads)

        self.map_dimensions = map_dimensions

        self.towers = np.ndarray((self.number_of_towers, 5),)

        self.pub_tower = rospy.Publisher('publisher/towers', Towers, queue_size=10)

        for i in range(self.number_of_towers):

            l = random.random() * map_dimensions[0]
            a = random.random()

            l = 18
            a = 1

            if a < 0.5:
                self.towers[i,0] = 0
                self.towers[i,1] = l

            if a >= 0.5 :
                self.towers[i,0] = map_dimensions[0]
                self.towers[i,1] = l

            self.towers[i,4] = self.tower_angle

            self.towers[i,2] = 0
            self.towers[i,3] = 0

            print(self.towers[i,0:2])


    def get_new_dir(self):

        pass

    def publish_tower(self):

        pass

    def get_strip_start_end_points_from_vector(self, vector: np.ndarray((2,)), tower_position: np.ndarray((2,)), space_size: float):

        m = vector[1] / vector[0]
        b = tower_position[1] - m * tower_position[0]
        points = np.zeros((2, 2))
        # Point at x = 0:
        points[0, :] = np.array([0, b])

        if b < 0:

            points[0, :] = np.array([-b/m, 0])

        if b > space_size:

            points[0, :] = np.array([(space_size-b)/m, space_size])

        # Point at x = space_size
        points[1, :] = np.array([space_size, m * space_size + b])

        if m * space_size + b < 0:

            points[1, :] = np.array([-b/m, 0])
        
        if m * space_size + b > space_size:

            points[1, :] = np.array([(space_size-b)/m, space_size])




        return points
    
    def rotate_vector(self, vector: np.ndarray((2,)), sin: float, cos: float):

        x = vector[0] * cos - vector[1] * sin
        y = vector[0] * sin + vector[1] * cos
        return np.array([x, y])
        

    def run_towers(self, curr_pos, idx):

        for i in range(self.number_of_towers):
            
            p = curr_pos[idx, :]

            # Align tower with agent
            self.towers[i,2] = p[0] - self.towers[i,0]
            self.towers[i,3] = p[1] - self.towers[i,1]
            #n = np.linalg.norm(self.towers[i,2:4])
            #self.towers[i,2] /= n
            #self.towers[i,3] /= n

            # Calculate neighbours

            # Publish agent p info to all neighbours

            # Publish tower to vizualization
            lower_vector = self.rotate_vector(np.array(self.towers[i,2:4]), -1 * self.strip_half_rads_sin, self.strip_half_rads_cos)
            upper_vector = self.rotate_vector(np.array(self.towers[i,2:4]), self.strip_half_rads_sin, self.strip_half_rads_cos)

            lower_boundary = self.get_strip_start_end_points_from_vector(lower_vector, self.towers[i,0:2], self.map_dimensions[0])
            upper_boundary = self.get_strip_start_end_points_from_vector(upper_vector, self.towers[i,0:2], self.map_dimensions[0])

            t = Tower()
            t.pos = self.towers[i,0:2]
            t.dir = self.towers[i,2:4]
            t.angle = self.towers[i,4]

            t.lower_b_s = lower_boundary[0,:]
            t.upper_b_s = upper_boundary[0,:]
            t.lower_b_e = lower_boundary[1,:]
            t.upper_b_e = upper_boundary[1,:]

            tt = Towers()
            tt.towers.append(t)

            self.pub_tower.publish(tt)
                


