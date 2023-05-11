#!/usr/bin/env python3

import rospy
import numpy as np
import random
from pdfc_fr.msg import Towers, Tower, TowerData, TowerDataArray
import math

class TowerHelper():

    def __init__(self, map_dimensions, pub_tower_data):

        self.number_of_towers = rospy.get_param("number_of_towers")
        self.tower_angle = rospy.get_param("tower_angle")

        self.cf_radius = rospy.get_param("cf_radius")
        self.noise_std = rospy.get_param("noise_std")
        
        self.strip_half_rads = math.radians(self.tower_angle/2)
        self.strip_half_rads_sin = math.sin(self.strip_half_rads)
        self.strip_half_rads_cos = math.cos(self.strip_half_rads)

        self.map_dimensions = map_dimensions

        self.towers = np.ndarray((self.number_of_towers, 5),)

        self.pub_tower = rospy.Publisher('publisher/towers', Towers, queue_size=10)
        self.pub_tower_data = pub_tower_data

        self.corrupted_pos = np.ndarray((self.number_of_towers,3))
        self.corrupted_vel = np.ndarray((self.number_of_towers,3))

        for i in range(self.number_of_towers):

            l = random.random() * map_dimensions[0]
            a = random.random()

            l = 10
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

        


    def get_new_dir(self):

        pass

    def publish_tower(self):

        pass

    def get_strip_start_end_points_from_vector(self, vector: np.ndarray((2,)), tower_position: np.ndarray((2,)), space_size: float, p):

        m = vector[1] / vector[0]
        b = tower_position[1] - m * tower_position[0]
        points = np.zeros((2, 2))

        points[0, :] = np.array([0, b])
        
        if b > space_size:
            
            points[0, :] = np.array([(space_size-b)/m, space_size])

            unit_vector_1 = (tower_position-p) / np.linalg.norm((tower_position-p))
            unit_vector_2 = (tower_position-points[0,:]) / np.linalg.norm((tower_position-points[0,:]))
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)

            if angle > 2.5:
                points[0, :] = np.array([space_size, 0])

        if b < 0:
            points[0, :] = np.array([-b/m, 0])

        points[1, :] = np.array([space_size, m * space_size + b])

        if m * space_size + b > space_size:
            points[1, :] = np.array([(space_size-b)/m, space_size])

            unit_vector_1 = (tower_position-p) / np.linalg.norm((tower_position-p))
            unit_vector_2 = (tower_position-points[1,:]) / np.linalg.norm((tower_position-points[1,:]))
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)

            if angle > 2.5:
                points[1, :] = np.array([0, 0])

        if m * space_size + b < 0:
            points[1, :] = np.array([-b/m, 0])

        return points
    
    def rotate_vector(self, vector: np.ndarray((2,)), sin: float, cos: float):

        x = vector[0] * cos - vector[1] * sin
        y = vector[0] * sin + vector[1] * cos
        return np.array([x, y])
    
    def order_boundaries(self, lower_boundary, upper_boundary):

        if lower_boundary[0, 1] != upper_boundary[0, 1]:
            if lower_boundary[0, 1] < upper_boundary[0, 1]:
                return lower_boundary, upper_boundary
            return upper_boundary, lower_boundary

        if lower_boundary[1, 1] < upper_boundary[1, 1]:
            return lower_boundary, upper_boundary
        return upper_boundary, lower_boundary
    
    def get_neighbours(self, curr_pos, lower_boundary, upper_boundary):

        dist = np.zeros((len(curr_pos),))

        m_l = (lower_boundary[1,1]-lower_boundary[0,1]) / (lower_boundary[1,0]-lower_boundary[0,0])
        b_l = lower_boundary[0,1] - m_l * lower_boundary[0,0]

        m_u = (upper_boundary[1,1]-upper_boundary[0,1]) / (upper_boundary[1,0]-upper_boundary[0,0])
        b_u = upper_boundary[0,1] - m_l * upper_boundary[0,0]

        radius = self.cf_radius + 6 * self.noise_std

        for i in range(len(curr_pos)):

            x = curr_pos[i, 0]
            y = curr_pos[i, 1]

            # Check if point is above lower boundary and below lower boundary
            if (y >= m_l * x + b_l + radius) and (y <= m_u * x + b_u - radius):
                dist[i] = 1

        return dist

    def corrupt_data(self, data):

        corrupted_data = np.ndarray((len(data),3))
        
        
        for i in range(len(data)):

            noise = np.random.normal(0,self.noise_std,3)

            for j in range(len(noise)):
                if noise[j] > 3 * self.noise_std:
                    noise[j] = 3 * self.noise_std
                if noise[j] < -3 * self.noise_std:
                    noise[j] = -3 * self.noise_std

            corrupted_data[i,0] = data[i,0] + noise[0]
            corrupted_data[i,1] = data[i,1] + noise[1]
            corrupted_data[i,2] = data[i,2] + noise[2]
        return corrupted_data
    
    def get_actuation_vector(self, curr_pos):

        n = np.zeros((len(curr_pos)),)

        for i in range(self.number_of_towers):

            for idx in range(len(curr_pos)):
            
                p = curr_pos[idx, :]

                self.towers[i,2] = p[0] - self.towers[i,0]
                self.towers[i,3] = p[1] - self.towers[i,1]

                lower_vector = self.rotate_vector(np.array(self.towers[i,2:4]), -1 * self.strip_half_rads_sin, self.strip_half_rads_cos)
                upper_vector = self.rotate_vector(np.array(self.towers[i,2:4]), self.strip_half_rads_sin, self.strip_half_rads_cos)

                lower_boundary = self.get_strip_start_end_points_from_vector(lower_vector, self.towers[i,0:2], self.map_dimensions[0],p[0:2])
                upper_boundary = self.get_strip_start_end_points_from_vector(upper_vector, self.towers[i,0:2], self.map_dimensions[0],p[0:2])
                lower_boundary, upper_boundary = self.order_boundaries(lower_boundary,upper_boundary)

                neighbours = self.get_neighbours(curr_pos, lower_boundary, upper_boundary)
                neighbours[idx] = 1

                for j in np.where(neighbours == 1)[0]:
                    n[int(j)] += 1
        
        return n
        

    def run_towers(self, curr_pos, curr_vel, idx):

        if idx == 0:
            self.corrupted_pos = self.corrupt_data(curr_pos)
            self.corrupted_vel = self.corrupt_data(curr_vel)

            self.n = self.get_actuation_vector(curr_pos)

        corrupted_pos = self.corrupted_pos
        corrupted_vel = self.corrupted_vel

        for i in range(self.number_of_towers):
            
            p = curr_pos[idx, :]

            # Align tower with agent
            self.towers[i,2] = p[0] - self.towers[i,0]
            self.towers[i,3] = p[1] - self.towers[i,1]
            #n = np.linalg.norm(self.towers[i,2:4])
            #self.towers[i,2] /= n
            #self.towers[i,3] /= n

            lower_vector = self.rotate_vector(np.array(self.towers[i,2:4]), -1 * self.strip_half_rads_sin, self.strip_half_rads_cos)
            upper_vector = self.rotate_vector(np.array(self.towers[i,2:4]), self.strip_half_rads_sin, self.strip_half_rads_cos)

            lower_boundary = self.get_strip_start_end_points_from_vector(lower_vector, self.towers[i,0:2], self.map_dimensions[0],p[0:2])
            upper_boundary = self.get_strip_start_end_points_from_vector(upper_vector, self.towers[i,0:2], self.map_dimensions[0],p[0:2])

            lower_boundary, upper_boundary = self.order_boundaries(lower_boundary,upper_boundary)

            # Calculate neighbours
            neighbours = self.get_neighbours(curr_pos, lower_boundary, upper_boundary)
            neighbours[idx] = 1

            #print("neighbours of node ", idx, np.where(neighbours == 1)[0])

            #print("s", idx,  neighbours)

            # Publish agent p info to all neighbours
            data_array = TowerDataArray()
             
            for j in range(len(curr_pos)):
                if neighbours[j] == 1:
                    data = TowerData()

                    data.p = corrupted_pos[j, :]
                    data.v = corrupted_vel[j, :]
                    data.id = j
                    data.n = int(self.n[j])

                    data_array.data.append(data)
            for j in range(len(curr_pos)):
                if neighbours[j] == 1:
                    # publish p to i
                    data_array.target = idx
                    self.pub_tower_data[j].publish(data_array)


            # Publish tower to vizualization
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
                


