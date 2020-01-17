#!/usr/bin/env python3

"""
    # {Pei-Lun Hsu}
    # {890319-T412}
    # {plhsu@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs, sqrt

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        
        """print("angle_min: ", scan.angle_min)
        print("angle_max: ", scan.angle_max)
        print("range_min: ", scan.range_min)
        print("range_max: ", scan.range_max)
        print("angle_increment: ", scan.angle_increment)
        print("ranges[0]: ", scan.ranges[0])
        print("ranges[1]: ", scan.ranges[1])
        print("ranges[2]: ", scan.ranges[2])
        print("occupied_space: ", self.occupied_space)
        print("free_space: ", self.free_space)        
        print("c_space: ", self.c_space)
        print("unknown_space: ", self.unknown_space)
        print("grid_map._GridMap__map: ", grid_map._GridMap__map)
        print("shape of grid_map._GridMap__map: ", grid_map._GridMap__map.shape)
        print("origin of grid_map: ", grid_map.get_origin())
        print("origin position of grid_map: ", grid_map.get_origin().position)
        print("origin orientation of grid_map: ", grid_map.get_origin().orientation)
        print("grid_map.get_width: ", grid_map.get_width())
        print("grid_map.get_height: ", grid_map.get_height())
        """




        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        ranges = scan.ranges

        """
        Fill in your solution here
        """
        x_laser = -1
        y_laser = -1
        x_min = 0
        y_min = 0
        x_max = 0
        y_max = 0
        x_r = int((pose.pose.position.x - origin.position.x) / resolution) #index of robot
        y_r = int((pose.pose.position.y - origin.position.y) / resolution)

        # fill in free space
        for i, range_laser in enumerate(ranges):

            if range_laser <= scan.range_min or range_laser >= scan.range_max: #check if range is within acceptable bounds
                continue

            angle_laser = scan.angle_min + i * scan.angle_increment
            #print("angle_max: ", scan.angle_max)
            #print("angle_laser: ", angle_laser)
           
            #convert laser bearing and rage(distance) to the laser frame and then to map frame, ie, robot frame --> map frame
            x_laser = pose.pose.position.x + range_laser * cos(robot_yaw + angle_laser)
            y_laser = pose.pose.position.y + range_laser * sin(robot_yaw + angle_laser)


            #convert the coordinate of the range_laser to indices of grid map
            index_x = int((x_laser - origin.position.x) / resolution)
            index_y = int((y_laser - origin.position.y) / resolution)
            #if not self.is_in_bounds(grid_map, index_x, index_x):
            #    continue

            #use raytracing function to obtain the indices list of fee space cells in grid map
            start = (x_r, y_r)
            end = (index_x, index_y)
            cells_free = self.raytrace(start, end)
            
            #fill in the  freespace cells (index x, y) with free_space values in grid_map, record the updated min and max of index x, y
            for cell in cells_free:
                x_free = cell[0]
                y_free = cell[1]

                if self.add_to_map(grid_map, x_free, y_free, self.free_space):
                    pass

                    #if x_free < x_min:
                    #    x_min = x_free
                    #elif x_free > x_max:
                    #    x_max = x_free
                    #if y_free < y_min:
                    #    y_min = y_free
                    #elif y_free > y_max:
                    #    y_max = y_free


        #fill in occupied space
        for i, range_laser in enumerate(ranges):

            if range_laser <= scan.range_min or range_laser >= scan.range_max: #check if range is within acceptable bounds
                continue

            angle_laser = scan.angle_min + i * scan.angle_increment
            #print("angle_max: ", scan.angle_max)
            #print("angle_laser: ", angle_laser)
           
            #convert laser bearing and rage(distance) to the laser frame and then to map frame, ie, robot frame --> map frame
            x_laser = pose.pose.position.x + range_laser * cos(robot_yaw + angle_laser)
            y_laser = pose.pose.position.y + range_laser * sin(robot_yaw + angle_laser)


            #convert the coordinate of the range_laser to indices of grid map
            index_x = int((x_laser - origin.position.x) / resolution)
            index_y = int((y_laser - origin.position.y) / resolution)
            #if not self.is_in_bounds(grid_map, index_x, index_x):
            #    continue
            
            #fill in the occupied cells (index x, y) with occupied values in grid_map
            if self.add_to_map(grid_map, index_x, index_y, self.occupied_space):
                if index_x < x_min:
                    x_min = index_x
                elif index_x > x_max:
                    x_max = index_x
                if index_y < y_min:
                    y_min = index_y
                elif index_y > y_max:
                    y_max = index_y

        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = x_min
        # The minimum y index in 'grid_map' that has been updated
        update.y = y_min
        # Maximum x index - minimum x index + 1
        update.width = x_max - x_min + 1
        # Maximum y index - minimum y index + 1
        update.height = y_max - y_min + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []
        for y_u in range(y_min, y_max + 1):
            for x_u in range(x_min, x_max + 1):
                updated_value = grid_map[x_u, y_u]
                update.data.append(updated_value)

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """

        width = grid_map.get_width()
        height = grid_map.get_height()
        radius = self.radius
        #fill in the C space cells whose distance to occupied cells <= robot radius
        for x_grid in range(width):
            for y_grid in range(height):

                if grid_map[x_grid, y_grid] == self.occupied_space:
                    x_0 = x_grid - radius
                    y_0 = y_grid - radius

                    for delta_x in range(2 * radius + 1):
                        for delta_y in range(2 * radius + 1):
                            x_check = x_0 + delta_x
                            y_check = y_0 + delta_y
                            if sqrt((x_check - x_grid)**2 + (y_check - y_grid)**2) <= radius and grid_map[x_check, y_check] != self.occupied_space:
                                self.add_to_map(grid_map, x_check, y_check, self.c_space)


        # Return the inflated map
        return grid_map
