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
from utils import *
import turtle  # You need to run this locally to use the turtle module
import time as t
#from robot import robot


# this is the particle filter class
#

class particles_non_random:

    # --------
    # init: 
    #	creates particle set with given initial position
    #

    def __init__(self, measurements, radius, center, Num_oscilations, 
                 steering_noise, distance_noise, measurement_noise, N = 100):

        """
        window = turtle.Screen()
        window.bgcolor('white')
        size=0.3
        size_multiplier = 20.0  # change Size of animation
        broken_robot = turtle.Turtle()
        broken_robot.shape('turtle')
        broken_robot.color('green')
        broken_robot.resizemode('user')
        broken_robot.shapesize(size*4, size*4, size*4)
        broken_robot.penup()

        broken_robot.setheading(0.0001 * 180 / pi)
        broken_robot.goto(measurements[-1][0] * size_multiplier, measurements[-1][1] * size_multiplier - 200)
        broken_robot.stamp()

        broken_robot.color('orange')
        broken_robot.shapesize(size,size,size)
        """

        self.N = N
        self.steering_noise    = steering_noise
        self.distance_noise    = distance_noise
        self.measurement_noise = measurement_noise

        theta=2*pi/Num_oscilations
        phi= pi-theta
        edge_length = radius * 2 * sin(theta/2)
        self.data = []

        for i in range(self.N):
            j=self.N-i-1
            
            x=measurements[-j][0]
            y=measurements[-j][1]

            #print(j,x,y)

            m = (y-center[1])/(x-center[0])
            angle = atan(m)
            heading = angle + phi/2
            if x<center[0]:
                heading+= pi

            r = robot(x, y, heading)
            """
            broken_robot.setheading(r.heading * 180 / pi)
            broken_robot.goto(r.x * size_multiplier, r.y * size_multiplier - 200)
            broken_robot.stamp()
            """
            #print(angle * 180/pi)
            
            

           # for i in range(1,35):
               # print((r.heading + theta*i) * 180/pi)


            for k in range(j):
                r.move(theta,edge_length)
                """
                #broken_robot.setheading(r.heading * 180 / pi)
                broken_robot.color('orange')
                broken_robot.goto(r.x * size_multiplier, r.y * size_multiplier - 200)
                broken_robot.stamp()
                
                t.sleep(2)
            broken_robot.color('blue')
            broken_robot.goto(r.x * size_multiplier, r.y * size_multiplier - 200)
            broken_robot.stamp()
            """
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            
            #print()
            #t.sleep(30)
            

            self.data.append(r)
        
        #broken_robot.clear()


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


    def sense_2(self, Z,mx,my,R):
        w = []
        w2= []
        #sw = 0
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))
        for i in range(self.N):
            x=self.data[i].x
            y=self.data[i].y
            w2.append(abs(distance_between([mx,my],[x,y])-R)/R)
        sw2 = sum(w2)
        if sw2>0:
            w2=[w2[i]/sw2 for i in range(len(w2))]
        
        # resampling (careful, this is using shallow copy)
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        
        sw = sum(w)
        if sw>0:
            w=[w[i]/sw for i in range(len(w))]
        
        else:
            return sw
        
        w=[w[i]*w2[i] for i in range(len(w))]
        w=[w[i]/sw for i in range(len(w))]
        mw = max(w)
        sw= sum(w)
        st=np.std(w)

        for i in range(self.N):
            beta += random.random() * 1.0 * mw
            #beta+=random.gauss(2.0 * mw, mw/4.0)
            num=0.0
            b=0.0
            while beta > w[index]:
                beta -= w[index]
                if abs(beta-b)<st/4:
                    num+=1
                    if num>=20:
                        index=w.index(mw)
                        break
                index = (index + 1) % self.N
                b=beta
            p3.append(self.data[index])
        self.data = p3
        
        return sw


    



    

# --------


# In[ ]:




