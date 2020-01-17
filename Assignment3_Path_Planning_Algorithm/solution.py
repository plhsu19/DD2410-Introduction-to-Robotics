#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# PEI-LUN HSU
# {student id}
# {student email}

from dubins import *
import math

dt = 0.01

class Node(object):

    #resolution for grid map (grid width = 0.2)
    linear_res = 0.3
    angular_res = 2.0 * math.pi / 6.0

    def __init__(self, car, x, y, theta, costG = 0.0, parent = None):
        self.car = car
        self.x = x
        self.y = y
        self.theta = theta
        self.costG = costG #1
        self.parent = parent #2

        # Instance variables initialized by the instance method:
        self.costH = 0.0 #initialized by set_cost()
        self.cost = 0.0  #initialized by set_cost()

        # Other instance variables initialized by the function "update_next_nodes()"
        self.time = 0 #3
        self.phi = 0 #4 turning anle used to drive from parent node

    def set_cost(self):
        #expected ideal cost(distance) to target
        self.costH = math.sqrt(((self.x - self.car.xt)**2 + (self.y - self.car.yt)**2))
        self.cost = 1 * self.costG + 2 * self.costH
    
    def verify_goal(self):
        d = math.sqrt(((self.x - self.car.xt)**2 + (self.y - self.car.yt)**2))
        if d <= 1.0:
            return True
        else: 
            return False
def update_next_nodes(car, c_node, open_set, closed_set, timesteps = 45):
    # three steering directions
    steering = [-math.pi/4.0, 0.0, math.pi/4.0]
    # generate primitive three next nodes
    for dir in steering: #for all δ do
        k = -1
        xn = c_node.x
        yn = c_node.y
        thetan = c_node.theta

        # n′ ← succeeding state of n using μ(nθ , δ)
        for i in range(timesteps):
            k = i 
            xn, yn, thetan = step(car, xn, yn, thetan, dir)
            #check if next step nodes are within boundaries and not near obstacles
            if not verify_node(car, xn, yn):
                k = 404
                break
        '''print("k = ", k)
        print("xn = ", xn)
        print("yn = ", yn)
        print("thetan = ", thetan)'''
        # n′ ← succeeding state of n using μ(nθ , δ): initialize neighbor node 
        n_node = Node(car, xn, yn, thetan)
        n_node_idx = calc_grid_index(n_node)
        # print("n_node_idx = ", n_node_idx)
        # if n′ ∈/ C then
        if not n_node_idx in closed_set:
            # Check if n_node in obstacle/collision:
            # k = (timesteps - 1) --> free space; k = 404 --> collided or out of bounds

            # if m (n′ ) = obstacle then
            if k == 404:
                #C ← C ∪ {n′}
                closed_set[n_node_idx] = n_node 

            #elseif ∃ n∈O: n = n′ then
            elif (k == (timesteps - 1)) and (n_node_idx in open_set):
                #compute new costs g′
                n_costG = c_node.costG + 0.01 * timesteps #(distance/timestep = 0.01, velocity = 1)
                n_node.set_cost() #initialize cost_H and cost
                n_node.parent = c_node
                n_node.time = c_node.time + timesteps
                n_node.phi = dir
                #if g′ < g value of existing node in O then replace existing node in O with n′
                if n_node.cost < open_set[n_node_idx].cost:
                    open_set[n_node_idx] = n_node
            #else
            elif k == (timesteps - 1):
                # O ← O ∪ {n′}
                n_costG = c_node.costG + 0.01 * timesteps 
                n_node.set_cost() #initialize cost_H and cost
                n_node.parent = c_node
                n_node.time = c_node.time + timesteps
                n_node.phi = dir
                open_set[n_node_idx] = n_node

            '''if k == (timesteps - 1):
            n_costG = c_node.costG + 0.01 * timesteps #(distance/timestep = 0.01, velocity = 1)
            n_node = Node(car, xn, yn, thetan, n_costG, c_node) 
            n_node.set_cost() #initialize cost_H and cost
            n_node.time = c_node.time + timesteps
            n_node.phi = dir
            next_nodes.append(n_node)'''

    return True
    
def calc_grid_index(node):
    # lineat resolution = 0.2
    x_idx = int(node.x / Node.linear_res) 
    y_idx = int(node.y / Node.linear_res)
    # angular resolution = 2 pi / 6 (6 states / grid)
    theta_pos = node.theta % (2.0 * math.pi)
    theta_idx = int(theta_pos / Node.angular_res)
    return (x_idx, y_idx, theta_idx)

def verify_node(car, x, y):
    if x > car.xub:
        return False
    if x < car.xlb:
        return False
    if y > (car.yub - 0.2):
        return False
    if y < (car.ylb + 0.1):
        return False
    for ob in car.obs:
        if math.sqrt((x - ob[0])**2 + (y - ob[1])**2) < (ob[2] + 0.15): # within (obstacles' radius + 0.05 m) --> collision
            return False
    return True

def solution(car):

    ''' <<< write your code below >>> '''
    print(__file__ + " start!!")
    #list of turning angle
    controls=[] 
    #list of turning time times[0]-->times[1] wrt controls[0]
    times=[]
    #initialize start node, initialize cost function
    theta0 = 0.0
    nstar = Node(car, car.x0, car.y0, theta0) 
    nstar.set_cost()
    print("start position: (", nstar.x, ", ", nstar.y, ")")
    print("goal position: (", car.xt, ", ", car.yt, ")")
    #initial open set, closed set:
    open_set = dict() 
    closed_set = dict()
    #add start node into openset
    open_set[calc_grid_index(nstar)] = nstar
    c = 1
    #while loop to explore the path when openset is not empty
    while len(open_set) != 0:
       '''print("Visited node number: ", c)
       print("\n")
       print("\n")'''

       #choose the node with smallest cost value from open set
       c_idx = min(open_set, key=lambda o: open_set[o].cost)
       c_node = open_set[c_idx]

       '''print("current node position: (", c_node.x, ", ", c_node.y, ")")
       print("current costG = ", c_node.costG)
       print("current theta = ", c_node.theta)
       print("current phi = ", c_node.phi)
       print("current costH = ", c_node.costH)
       print("current cost = ", c_node.cost)
       print("current time = ", c_node.time)
       if c_node.parent == None:
           print("current parent = None")
       else:
           print("current parent != None")
       print("\n")
       print("\n")'''

       #remove current node from open set
       del open_set[c_idx]
       '''print("open_set: ")
       print(open_set)'''
       #add current node into close set
       closed_set[c_idx] = c_node
       '''print("closed_set: ")
       print(closed_set)'''
       #check if current node reach the goal or run too many loops
       if c_node.verify_goal() or c >= 5600:
           if c_node.verify_goal():
               print("Find goal")
               print("c_node x, y = ", c_node.x, ", ", c_node.y)
               print("\n")
           else:
                print("Failed to reach goal")
                print("c = ", c)
                print("\n")
           # start to contruct the path by building times list and controls list
           times.insert(0, c_node.time * dt)
           controls.insert(0, c_node.phi)
           #trace back until there is no parent node, ie, current node is the start node
           while True:
               if c_node.parent == None:
                   controls.pop(0)
                   break
               else:
                   c_node = c_node.parent
                   times.insert(0, c_node.time * dt)
                   controls.insert(0, c_node.phi) # control needs to be poped out if c_node is the start node
           break
       else:
           #print("update neighbor nodes!")
           #update neighbor/next nodes
           update_next_nodes(car, c_node, open_set, closed_set)

       c += 1

    ''' <<< write your code below >>> '''
    
    #print(controls)
    #print(times)
    return controls, times
