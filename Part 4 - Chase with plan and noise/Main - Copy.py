# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot. 
# But this time, your speed will be about the same as the runaway bot. 
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3. Again, try to catch the target in as few steps as possible.
from distutils.log import error
from statistics import *
import math
import sys
from turtle import resizemode, shapesize

from sqlalchemy import true
sys.path.append('C:\\Users\\alash\OneDrive - Higher Education Commission\\Other stuff\\Courses\\Robot\\runaway-robot\\common')
from robot import *
from math import *
from matrix import *
import random



sys.path.append('C:\\Users\\alash\OneDrive - Higher Education Commission\\Other stuff\\Courses\\Robot\\runaway-robot\\common')
from predictor import ExtendedKalmanFilter
from robot import *
from predictor import *
from utils import *
from Particle_Filter_Copy import*
#from common.Particle_Filter_Copy import*
import time as t
import numpy as np
#from common.P_filter_non_random import *
from P_filter_non_random import *
global p_list,num5,first_calc,last,P_initialized,predictor,num2
num2=0
first_calc=True
P_initialized=False
are_transformed=False
num_particles=100
pos_list=[]
num5 = 0
steps=0
p_list=[]
predictor=particles(0,0,0,0,0,0)
edge_length=0.0
theta=0.0
move_num=0
pred_pos=[]
#import matplotlib.pyplot as plt
#import seaborn as sns


def cir_transform(x_measured,y_measured,x_center,y_center,radius):

    m = (y_measured-y_center)/(x_measured-x_center)

    x1 = ((radius**2)/(m**2 + 1))**0.5 + x_center
    y1 = m*x1-m*x_center+y_center

    x2=-((radius**2)/(m**2 + 1))**0.5 + x_center
    y2 = m*x2-m*x_center+y_center

    if distance_between([x1,y1],[x_measured,y_measured])<distance_between([x2,y2],[x_measured,y_measured]):
        x=x1
        y=y1
    else:
        x=x2
        y=y2
    return x,y

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
#global r_list,mx_list,my_list,radius_list,x_list,y_list,num_list
r_list=[]
list3=[]
a_list=[]
radius_list=[]
x_list=[]
y_list=[]
mx_list=[]
my_list=[]
num_list=[]

def next_pos(measurement, OTHER = None):

    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    cond3=False
    global last,first_calc,num5,P_initialized,predictor,num_list,num2,are_transformed,num_particles,theta,\
        edge_length,measuring_tol,a_list,list3
    

    
    if num5<=1:
        if not OTHER:
            OTHER = []
        OTHER.append([measurement[0],measurement[1]])
        #print(len(OTHER),num5,num2)
        x_list.append(measurement[0])
        y_list.append(measurement[1])
        if len(OTHER) == 1:
            x = OTHER[0][0]
            y = OTHER[0][1]
            xy_estimate = (x, y)
        elif len(OTHER) == 2:
            x1 = OTHER[0][0]
            y1 = OTHER[0][1]
            x2 = OTHER[1][0]
            y2 = OTHER[1][1]
            dx = x2 - x1
            dy = y2 - y1
            xy_estimate = (dx+x2, dy+y2)
            
        else:
            headings = []
            dists = []
            for i in range(1, len(OTHER)):
                p1 = (OTHER[i][0], OTHER[i][1])
                p2 = (OTHER[i-1][0], OTHER[i-1][1])
                dist = distance_between(p1, p2)
                dx = p1[0] - p2[0]
                dy = p1[1] - p2[1]
                heading = atan2(dy, dx)
                dists.append(dist)
                headings.append(heading)

            turnings = []
            for i in range(1, len(headings)):
                turnings.append(headings[i] - headings[i-1])

            est_dist = sum(dists) / len(dists)
            est_turning = sum(turnings) / len(turnings)
            est_heading = angle_trunc(headings[-1] + est_turning)
            x = OTHER[-1][0]
            y = OTHER[-1][1]
            est_x = x + est_dist * cos(est_heading)
            est_y = y + est_dist * sin(est_heading)
            xy_estimate = (est_x, est_y)
            
        
    else:
        Num_oscilations = mean(num_list)
        radius=radius_list[-1]
        cx = mx_list[-1]
        cy = my_list[-1]

        theta=2*pi/Num_oscilations
        phi= pi-theta
        edge_length = radius * 2 * sin(theta/2)

        if not are_transformed:
            for i in range(len(OTHER)):
                #print(type(OTHER[i]))
                OTHER[i][0],OTHER[i][1]=cir_transform(OTHER[i][0],OTHER[i][1],cx,cy,radius)
            are_transformed=True
        x,y=cir_transform(measurement[0],measurement[1],cx,cy,radius)
        num2+=1
        OTHER.append([x,y])
        x_list.append(measurement[0])
        y_list.append(measurement[1])
        xy_estimate=OTHER[-1]
        
        #Now starting the estimation calculation
        headings = []
        dists = []
        for i in range(1, len(OTHER)):
            p1 = (OTHER[i][0], OTHER[i][1])
            p2 = (OTHER[i-1][0], OTHER[i-1][1])
            dist = distance_between(p1, p2)
            dx = p1[0] - p2[0]
            dy = p1[1] - p2[1]
            heading = atan2(dy, dx)
            dists.append(dist)
            headings.append(heading)

        turnings = []
        for i in range(1, len(headings)):
            turnings.append(headings[i] - headings[i-1])

        est_dist = sum(dists) / len(dists)
        est_turning = sum(turnings) / len(turnings)
        if est_dist<0 or est_dist>4*edge_length:
            est_dist=edge_length
            est_turning=theta
        est_heading = angle_trunc(headings[-1] + est_turning)
        m = (y-cy)/(x-cx)
        angle = atan(m)
        est_heading = angle + phi/2
        if x<cx:
                est_heading+= pi

        x = OTHER[-1][0]
        y = OTHER[-1][1]
        est_x = x + est_dist * cos(est_heading)
        est_y = y + est_dist * sin(est_heading)
        xy_estimate = (est_x, est_y)
        
        #First estimating and moving at least 2 times with circular transformed measurements\
        # then applying the particle filter
        if num2>2:
                if not P_initialized:
                    predictor=particles(x,y,headings[-1],measurement_noise\
                        ,measurement_noise,measurement_noise)
                    P_initialized=True

                if len(OTHER)<num_particles:
                    predictor.sense([x,y])
                    predictor.move(est_turning,est_dist)
                    est=predictor.get_position()
                    #xy_estimate=cir_transform(est[0],est[1],cx,cy,radius)
                    xy_estimate=[est[0],est[1]]

                #"""
                else:
                    if len(OTHER)%num_particles/4==0:
                        #print(OTHER[-100:])
                        predictor=particles_non_random(OTHER[-(num_particles):],radius,[cx,cy],\
                            Num_oscilations,measurement_noise,measurement_noise,measurement_noise,num_particles)
                    #predictor.sense_2([x,y],cx,cy,radius)
                    predictor.sense([x,y])
                    predictor.move(est_turning,est_dist)
                    est=predictor.get_position()
                    xy_estimate=[est[0],est[1]]
                    xy_estimate=cir_transform(est[0],est[1],cx,cy,radius)
                    
                #"""
        

    if len(OTHER)>30:
        if first_calc:
            last=1
            #"""
            measuring_cond=distance_between([OTHER[-1][0],OTHER[-1][1]],[OTHER[last][0],OTHER[last][1]])<\
            (measurement_noise+0.0001)*measuring_tol
        else:
            if OTHER[-1][0]>mx_list[-1] and OTHER[-1][1]<my_list[-1]:
                measuring_cond=True
            else:
                measuring_cond=distance_between([OTHER[-1][0],OTHER[-1][1]],[OTHER[last][0],OTHER[last][1]])<\
            (measurement_noise+0.0001)*measuring_tol
            #"""
        #measuring_cond=distance_between([OTHER[-1][0],OTHER[-1][1]],[OTHER[last][0],OTHER[last][1]])<\
         #   (measurement_noise+0.0001)*measuring_tol    
        if measuring_cond and len(OTHER)-last-1>30 :
            first_calc=False
            num5+=1
            num_list.append(len(OTHER)-last-1)
            last=len(OTHER)-1
            mx=mean(x_list)
            my=mean(y_list)    
            for i in range(len(OTHER)):
                r=(distance_between([mx,my],[OTHER[i][0],OTHER[i][1]]))
                list3.append(r)
                a_list.append(r**2)
                r_list.append(1/r**2)
            
            #r1=exp(mean(a_list))
            r1=(mean(a_list))**0.5
            r2=(1/mean(r_list))**0.5
            radius=mean([r1,r2,mean(list3)])
            radius_list.append(radius)
            #radius_list.append(r1)
            mx_list.append(mx)
            my_list.append(my)
            cond3=True
            #print("center: "+str(mx)+","+str(my))
            #print('radius: '+str(radius))
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    
    return xy_estimate, OTHER,cond3




def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 
    global pos_list,move_num,pred_pos,steps
    next_position,OTHER,condition=next_pos(target_measurement,OTHER)
    pos_list.append(next_position)

    if num5<1:
        distance=distance_between(hunter_position,next_position)
        if distance>max_distance:
            distance=max_distance
        heading=get_heading(hunter_position,next_position)
        turning=angle_trunc(heading-hunter_heading)
        #hunter=robot(hunter_position[0],hunter_position[-1],hunter_heading)
        #hunter.move(turning,distance)
    else:
        x=next_position[0]
        y=next_position[1]
        Num_oscilations = mean(num_list)
        radius=radius_list[-1]
        cx = mx_list[-1]
        cy = my_list[-1]

        theta=2*pi/Num_oscilations
        phi= pi-theta
        edge_length = radius * 2 * sin(theta/2)
        if move_num<=0:
            steps=floor(2*radius/max_distance)

        m = (y-cy)/(x-cx)
        angle = atan(m)
        est_heading = angle + phi/2
        if x<cx:
                est_heading+= pi
        prediction=robot(x,y,est_heading)
        for i in range(steps-move_num-1):
            prediction.move(theta,edge_length)
        pred_pos=cir_transform(prediction.x,prediction.y,cx,cy,radius)

        heading=get_heading(hunter_position,pred_pos)
        turning=angle_trunc(heading-hunter_heading)
        distance=max_distance 
        dist=distance_between(hunter_position,pred_pos)
        if dist<distance or move_num>=steps-1:
            #t.sleep(2)
            #print(next_position)
            distance=dist
            heading=get_heading(hunter_position,next_position)
            turning=angle_trunc(heading-hunter_heading)
        move_num+=1
        if move_num>=steps and dist>max_distance:
            move_num = 0



    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 1.0 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.04 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    error_list=[]
    ctr_list=[] 

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        
        
        
        # Check to see if the hunter has caught the target.
        
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        #print(pos_list)
        #if len(pos_list)>=1:
           # hunter_position=pos_list[ctr-1]
        separation = distance_between(hunter_position, target_position)
        #i0f separation<max_distance*1.5:
        error_list.append(separation)
        if separation < separation_tolerance:
            print(min(error_list),mean(error_list))
            print ("You got it right! It took you ", ctr, " steps to catch the target.")
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        if ctr%100==0:
            print(min(error_list[-100:]),mean(error_list[-100:]))

        ctr += 1            
        if ctr >= 1000:
            print ("It took too many steps to catch the target.")
    return caught



def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER


def demo_grading_visualized(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    size=0.3
    
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print("You got it right! It took you ", ctr, " steps to catch the target.")

            #turtle.done()
            
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        if separation > separation_tolerance and ctr>=100:
            if ctr==100:
                window = turtle.Screen()
                window.bgcolor('white')
                size_multiplier = 25.0 #change Size of animation

                chaser_robot = turtle.Turtle()
                chaser_robot.shape('circle')
                chaser_robot.color('blue')
                chaser_robot.resizemode('user')
                chaser_robot.shapesize(size, size, size)
                broken_robot = turtle.Turtle()
                broken_robot.shape('turtle')
                broken_robot.color('green')
                broken_robot.resizemode('user')

                mid_point=turtle.Turtle('circle')
                mid_point.color('black')
                mid_point.resizemode('user')
                mid_point.shapesize(size,size,size)
                mid_point.hideturtle()
                mid_point.penup()
                mid_point.goto(mx_list[-1]*size_multiplier,my_list[-1]*size_multiplier-300)
                mid_point.showturtle()


                broken_robot.shapesize(size, size, size)
                chaser_robot.hideturtle()
                chaser_robot.penup()
                chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-300)
                chaser_robot.showturtle()
                broken_robot.hideturtle()
                broken_robot.penup()
                broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-300)
                broken_robot.showturtle()
                measuredbroken_robot = turtle.Turtle()
                measuredbroken_robot.shape('circle')
                measuredbroken_robot.color('red')
                measuredbroken_robot.penup()
                measuredbroken_robot.resizemode('user')
                measuredbroken_robot.shapesize(size, size, size)
                broken_robot.pendown()
                chaser_robot.pendown()

            #measuredbroken_robot.setheading(target_bot.heading*180/pi)
            mid_point.goto(mx_list[-1]*size_multiplier,my_list[-1]*size_multiplier-300)
            mid_point.stamp()
            measuredbroken_robot.goto(pos_list[ctr][0]*size_multiplier, pos_list[ctr][1]*size_multiplier-300)
            measuredbroken_robot.stamp()
            #broken_robot.setheading(target_bot.heading*180/pi)
            broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-300)
            #chaser_robot.setheading(hunter_bot.heading*180/pi)
            chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-300)

            if ctr%30==0:
                measuredbroken_robot.clear()
                chaser_robot.clear()
            #if num5>1:
            #    t.sleep(0.5)
            #End of visualization
        ctr += 1
        
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 0.1 * target.distance
target.set_noise(0.0, 0.0, measurement_noise)
#measuring_tol = 1.3 * max([measurement_noise,target.distance])/min([measurement_noise,target.distance])
measuring_tol = 4.0
hunter = robot(-10.0, -10.0, 0.0)

demo_grading(hunter, target, next_move)
#demo_grading_visualized(hunter,target,next_move)
print(radius_list[-1])
print(mean(num_list))
print(num5)
print(measuring_tol)






