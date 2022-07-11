# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position.
#
# ----------
# GRADING
#
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
#from common.robot import *  # Check the robot.py tab to see how this works.
from pydoc import tempfilepager
from statistics import mean
import sys
from traceback import print_list
from turtle import distance
from requests import head
from sqlalchemy import false

from sympy import N
sys.path.append('C:\\Users\\alash\OneDrive - Higher Education Commission\\Other stuff\\Courses\\Robot\\runaway-robot\\common')
from predictor import ExtendedKalmanFilter
from robot import *
from predictor import *
from utils import *
from P_filter import*
#from common.P_filter import*
import time as t
import numpy as np
global p_list,num5,cond,last,cond4,predictor
cond=False
cond4=True
num5 = 0
p_list=[]
predictor=particles(0,0,0,0,0,0)
#import matplotlib.pyplot as plt
#import seaborn as sns

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
global r_list,mx_list,my_list,radius_list,x_list,y_list,num_list
r_list=[]
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
    global last,cond,num5,cond4,predictor,num_list

    if not OTHER:
        OTHER = []
    OTHER.append(measurement)
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
        
        if num5>1:
            
            theta=2*pi/mean(num_list)
            est_turning=theta
            est_dist=radius_list[-1]*2*sin(theta/2)
            est_x = x + est_dist * cos(est_heading)
            est_y = y + est_dist * sin(est_heading)
            xy_estimate = (est_x, est_y)
            #print('distance')
            #print(est_dist,est_dist_2)
            #print('turning')
            #print(est_turning,est_turning_2,theta)

        #""" particle buddy
        if num5>1:
                if cond4:
                    predictor=particles(measurement[0],measurement[1],headings[-1],measurement_noise\
                        ,measurement_noise,measurement_noise)
                    cond4=False
                predictor.sense(measurement,mx_list[-1],my_list[-1],radius_list[-1])
                predictor.move(est_turning,est_dist)
                est=predictor.get_position()
                xy_estimate=[est[0],est[1]]
        #"""

        if len(OTHER)>30:
            
            if not cond:
                last=1
                cond=True
            if(distance_between([OTHER[-1][0],OTHER[-1][1]],[OTHER[last][0],OTHER[last][1]]))<(measurement_noise+0.0001)*4\
            and len(OTHER)-last-1>30 :
                num5+=1
                num_list.append(len(OTHER)-last-1)
                last=len(OTHER)-1
                mx=mean(x_list)
                my=mean(y_list)    
                for i in range(len(OTHER)):
                    r_list.append(distance_between([mx,my],[OTHER[i][0],OTHER[i][1]]))
                radius=mean(r_list)
                radius_list.append(radius)
                mx_list.append(mx)
                my_list.append(my)
                cond3=True
            #print("center: "+str(mx)+","+str(my))
            #print('radius: '+str(radius))
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    return xy_estimate, OTHER,cond3
# This is here to give you a sense for how we will be running and grading
def est_pos(measurement, OTHER = None):
    cond=False
    if not OTHER:
        OTHER = []
    OTHER.append(measurement)
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

    return xy_estimate,OTHER,cond
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER=None):
    localized = False
    distance_tolerance = 0.02 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    error_list=[]
    ctr_list=[]
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER,condition = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)

        # x = OTHER[1]
        # print (angle_trunc(x.value[0][0]), x.value[1][0], x.value[2][0]), \
        #     (target_bot.orientation, target_bot.turning, target_bot.distance), \
        #     position_guess, true_position
        
        error = distance_between(position_guess, true_position)
        error_list.append(error)
        if condition:
            ctr_list.append(ctr)
            
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            #localized = True    
       
        if ctr == 1000:
            print("the final results are:")
            #print("Number of fails are "+str(num5))
            #print(min(p_list))
            print(len(ctr_list))
            print(len(mx_list))
            
            
            
            
            #t.sleep(0.2)
            print(min(error_list))
            print(mean(error_list))
            print("Sorry, it took you too many steps to localize the target.")
    return localized

def demo_grading_2(func1,func2, target_bot, OTHER=None):
    localized = False
    distance_tolerance = 0.02 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    error_list=[]
    error_list2=[]
    ctr_list=[]
    OTHER_1=OTHER
    OTHER_2=OTHER
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess_1, OTHER_1,condition_1 = func1(measurement, OTHER_1)
        position_guess_2, OTHER_2,condition_2 = func2(measurement, OTHER_2)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)

        # x = OTHER[1]
        # print (angle_trunc(x.value[0][0]), x.value[1][0], x.value[2][0]), \
        #     (target_bot.orientation, target_bot.turning, target_bot.distance), \
        #     position_guess, true_position
        
        error1 = distance_between(position_guess_1, true_position)
        error2 = distance_between(position_guess_2, true_position)

        error_list.append(error1)
        error_list2.append(error2)

        if condition_2:
            ctr_list.append(ctr)
            
        if error1 <= distance_tolerance or error2 <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            print(min(error_list))
            print(mean(error_list))
            print(min(error_list2))
            print(mean(error_list2))
            localized = True    
       
        if ctr == 1000:
            print("the final results are:")
            #print("Number of fails are "+str(num5))
            #print(min(p_list))
            print(len(ctr_list))
            print(len(mx_list))
            
            
            
            
            #t.sleep(0.2)
            print(min(error_list))
            print(mean(error_list))
            print(min(error_list2))
            print(mean(error_list2))
            print("Sorry, it took you too many steps to localize the target.")
    return localized

def demo_grading_visualize(estimate_next_pos_fcn, target_bot, OTHER=None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    # For Visualization
    import turtle  # You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size=0.2
    size_multiplier = 20.0  # change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(size*4, size*4, size*4)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('square')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(size, size, size)
    prediction = turtle.Turtle()
    prediction.shape('circle')
    prediction.color('orange')
    prediction.resizemode('user')
    prediction.shapesize(size*2, size*2, size*2)
    prediction.pendown()
    broken_robot.penup()
    measured_broken_robot.penup()
    
    error_list=[]
    # End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER,condition = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        

        # x = OTHER[1]
        # print (angle_trunc(x.value[0][0]), x.value[1][0], x.value[2][0]), \
        #     (target_bot.heading, target_bot.turning, target_bot.distance), \
        #     position_guess, true_position
        
        error = distance_between(position_guess, true_position)
        error_list.append(error)
        if ctr%30==0:
            print(min(error_list))
            
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            #localized = True
        #if ctr == 1000:
        #    print "Sorry, it took you too many steps to localize the target."
        # More Visualization
        if condition:
            measured_broken_robot.color('blue')
        else:
            measured_broken_robot.color('red')
        measured_broken_robot.setheading(target_bot.heading * 180 / pi)
        measured_broken_robot.goto(measurement[0] * size_multiplier, measurement[1] * size_multiplier - 200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading * 180 / pi)
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading * 180 / pi)
        prediction.goto(position_guess[0] * size_multiplier, position_guess[1] * size_multiplier - 200)
        prediction.stamp()
        #t.sleep(0.2)
        if ctr%60==0:
            broken_robot.clearstamps(-60)
            measured_broken_robot.clearstamps(-60)
            prediction.clearstamps(-60)
        # End of Visualization
    return localized



# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 2 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

#demo_grading_visualize(next_pos, test_target)
demo_grading(next_pos, test_target)
print(radius_list[-1])
print(mx_list[-1],my_list[-1])
#demo_grading_2(est_pos, next_pos, test_target)
#demo_grading(est_pos, test_target)
