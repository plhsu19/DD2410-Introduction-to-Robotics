#! /usr/bin/env python3
import math

"""
    # PEI-LUN HSU
    # {student email}
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """
    L0 = 0.07
    L1 = 0.3
    L2 = 0.35
    x0 = x - L0
    y0 = y
    z0 = z
    
    c2 = (x0 * x0 + y0 * y0 - L1*L1 - L2*L2) / (2 * L1 * L2)
    s2 = math.sqrt(1 - c2 * c2)
    theta2 = math.atan2(s2, c2)

    theta1 = math.atan2(y0, x0) - math.atan2(L2 * s2, L1 + L2 * c2)

    q[0] = theta1
    q[1] = theta2
    q[2] = z0
 
    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q
