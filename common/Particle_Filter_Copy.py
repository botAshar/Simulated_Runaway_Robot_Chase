#!/usr/bin/env python
# coding: utf-8

# In[ ]:



# User Instructions
#
#The following code provides the filtering, planning, localization, smoothing functions
# for a robots moving in a 2 dimmentional grid as provided
#
from statistics import mean, median 
from math import *
import random
from robot import *
import numpy as np
#from robot import robot


# this is the particle filter class
#

class particles:

    # --------
    # init: 
    #	creates particle set with given initial position
    #

    def __init__(self, x, y, theta, 
                 steering_noise, distance_noise, measurement_noise, N = 100):
        self.N = N
        self.steering_noise    = steering_noise
        self.distance_noise    = distance_noise
        self.measurement_noise = measurement_noise
        
        self.data = []
        for i in range(self.N):
            r = robot(x, y, theta)
            #r.set()
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            self.data.append(r)


    # --------
    #
    # extract position from a particle set
    # 
    
    def get_position(self):
        x = 0.0
        y = 0.0
        heading = 0.0

        div=1

        for i in range(int(self.N/div)):
            x += self.data[i].x
            y += self.data[i].y
            # heading is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            heading += (((self.data[i].heading
                              - self.data[0].heading + pi) % (2.0 * pi)) 
                            + self.data[0].heading - pi)
        return [x / self.N, y / (self.N/div), heading / (self.N/div)]

    # --------
    #
    # motion of the particles
    # 

    def move(self, steer, speed):
        newdata = []

        for i in range(self.N):
            r = self.data[i].move_2( steer, speed)
            newdata.append(r)
        self.data = newdata

    # --------
    #
    # sensing and resampling
    # 

    def sense(self, Z):
        w = []
        #sw = 0
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))

        # resampling (careful, this is using shallow copy)
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        
        sw = sum(w)
        if sw>0:
            [w[i]/sw for i in range(len(w))]
        
        else:
            return sw
        mw = max(w)
        sw= sum(w)
        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            #beta+=random.gauss(2.0 * mw, mw/4.0)
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.data[index])
        self.data = p3
        
        return sw


    



    

# --------


# In[ ]:




