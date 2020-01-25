# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import Empty, SetBool, SetBoolResponse



class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class Pick(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):
        rospy.loginfo("Initialising pick behaviour.")
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        
        self.tried = False
        self.done = False

        super(Pick, self).__init__("Pick!")

    def update(self):
        
    # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.pick_req = self.pick_srv()
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING
        
class Place(pt.behaviour.Behaviour):
    def __init__(self):
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
        rospy.wait_for_service(self.place_srv_nm, timeout=30)
        
        self.tried = False
        self.done = False

        super(Place, self).__init__("Place!")
    def update(self):

        if self.done:
            return pt.common.Status.SUCCESS

            # try if not tried
        elif not self.tried:

            # command
            self.place_req = self.place_srv()
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class Check(pt.behaviour.Behaviour):
    def __init__(self):
        self.cube_PoseStamped_new = PoseStamped()
        # self.cube_PoseStamped_new.header.seq = 10
        rospy.sleep(5)
        # self.Seq = 10
        self.cube_poseq = rospy.get_param(rospy.get_name() + '/cube_poseq')
        self.aruco_pose_subs = rospy.Subscriber(self.cube_poseq, PoseStamped, self.aruco_set_pose)
        
        if (self.cube_PoseStamped_new.pose.position.x > 0.45 and self.cube_PoseStamped_new.pose.position.x < 0.65):
            print("Check completed: cube detected!!")
            self.flag = 0
            print(self.flag)
        else:
            print("Check completed: didn't detect the cube, go back to Table1")
            self.flag = 1
            print(self.flag)
        
        # if self.Seq == 10:
        #     print("Check completed: didn't detect the cube, go back to Table1")
        #     self.flag = 0
        #     print(self.flag)
        # else:
        #     print("Check completed: cube detected!!")
        #     self.flag = 1
        #     print(self.flag)
        
        super(Check, self).__init__("Check!")

    def aruco_set_pose(self, aruco_pose_msg):
        self.cube_PoseStamped_new = aruco_pose_msg

    def update(self):
        if (self.flag == 0):
            return pt.common.Status.FAILURE
        if (self.flag == 1):
            return pt.common.Status.SUCCESS

class CheckDebug(pt.behaviour.Behaviour):
    def __init__(self):
        super(CheckDebug, self).__init__("CheckDebug!")
    def update(self):
        return pt.common.Status.FAILURE

class CheckDebugSUCCESS(pt.behaviour.Behaviour):
    def __init__(self):
        super(CheckDebugSUCCESS, self).__init__("CheckDebug!")
    def update(self):
        return pt.common.Status.SUCCESS



