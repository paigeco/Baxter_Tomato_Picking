#!/usr/bin/env python3

# Paige Cody :)

"""
Baxter RSDK Joint Position Waypoints Example
"""

import struct

import rospy

import baxter_interface

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from std_msgs.msg import String

import baxter_external_devices

from scipy.spatial.transform import Rotation as R

from baxter_interface import CHECK_VERSION

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)



class Movement(object):
    def __init__(self, limb, speed, accuracy):
        # Create baxter_interface limb instance
        self._arm = limb
        self.not_picked = True
        self._limb = baxter_interface.Limb(self._arm)
        
        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()


        self._gripper = baxter_interface.Gripper('right', CHECK_VERSION)
        self._gripper.set_velocity(0.3)
        self._gripper.set_moving_force(-5)
        self._gripper.set_holding_force(-5)
        self._gripper.calibrate()
        self._gripper.open()

        #self._waypoints = [{'right_s0': -0.23390617454920604, 'right_s1': -0.860654233603857, 'right_e0': 0.3518394894903826, 'right_e1': 1.6200705592321814, 'right_w0': -0.7459483742477713, 'right_w1': 0.3188102384572288, 'right_w2': 0.6197558400314158},
        #                   {'right_e0': -0.17333982903105175, 'right_e1': 1.4990827249610206, 'right_s0': 1.189218605808167, 'right_s1': -0.2389175077131532, 'right_w0': 0.29797576804674164, 'right_w1': -1.207626375262792, 'right_w2': -3.0541557486798587}]
        # Create Navigator I/O
        #self._navigator_io = baxter_interface.Navigator(self._arm)

    def move_arm(self, limb_rotations):
        """
        Loops playback of recorded joint position waypoints until program is
        exited
        """
        rospy.sleep(1.0)

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)
        if not(rospy.is_shutdown()):
            self._limb.move_to_joint_positions(limb_rotations, timeout=20.0, threshold=self._accuracy)

        rospy.sleep(1.0)

        # Set joint position speed back to default
        self._limb.set_joint_position_speed(0.3)

    def clean_shutdown(self):
        print("\nExiting example...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def inverse_kinematics(self,location):
        ns = "ExternalTools/" + self._arm + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        
        location_pose = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=float(location['x']),
                        y=float(location['y']),
                        z=float(location['z']),
                    ),
                    orientation=Quaternion(
                        x=float(location['qx']),
                        y=float(location['qy']),
                        z=float(location['qz']),
                        w=float(location['qw']),
                    ),
                ),
        )

        # Check the IK server for results
        ikreq.pose_stamp.append(location_pose)
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logerr("Service call failed!")
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        return 0
    
    def go_to(self, location):
        limb_rots = self.inverse_kinematics(location)
        if limb_rots == 0:
            print("INVALID ENTRY")
            return
        self.move_arm(limb_rots)


    def pick(self,pose):
        # Pull away
        #rospy.sleep(1.0)
        #self.go_to({'x':0.45,'y':-0.2,'z':0.3,'qx':1,'qy':1,'qz':0,'qw':0})
        #rospy.sleep(1.0)

        # Get in picking position.
        self.go_to({'x':0.9,'y':-0.2,'z':0.2,'qx':0.77,'qy':0,'qz':0.77,'qw':0})
        #rospy.sleep(1.0)

        comp = [0.1,0.15,-0.07]

        #self.go_to({'x':pose.position.x+0.25,'y':pose.position.y-0.2+comp[1],'z':pose.position.z+comp[2],'qx':0.77,'qy':0,'qz':0.77,'qw':0})
        self.go_to({'x':pose.position.x+0.45+comp[0],'y':pose.position.y-0.2+comp[1],'z':pose.position.z+comp[2],'qx':0.77,'qy':0,'qz':0.77,'qw':0})
        print("grasp")

        self._gripper.close()
        #self.go_to({'x':pose.position.x+0.25+comp[0],'y':pose.position.y-0.2+comp[1],'z':pose.position.z+comp[2],'qx':0.77,'qy':0,'qz':0.77,'qw':0})
        #rospy.sleep(1.0)

        #rospy.sleep(1.0)

        self.go_to({'x':0.8,'y':-0.2,'z':0.3,'qx':0.77,'qy':0,'qz':0.77,'qw':0})
        
        self.go_to({'x':0.65,'y':-0.2,'z':0.2,'qx':1,'qy':1,'qz':0,'qw':0})
        self.go_to({'x':0.65,'y':-0.2,'z':-0.15,'qx':1,'qy':1,'qz':0,'qw':0})
        #rospy.sleep(2.0)
        print("drop")
        self._gripper.open()
        #rospy.sleep(2.0)

        self.go_to({'x':0.45,'y':-0.2,'z':0.3,'qx':1,'qy':1,'qz':0,'qw':0})
        print("ready for next pick")
        self.not_picked = False
        
        run_tomato_finder.publish("GO")




    
def main():
    """RSDK Joint Position Waypoints Example

    Records joint positions each time the navigator 'OK/wheel'
    button is pressed.
    Upon pressing the navigator 'Rethink' button, the recorded joint positions
    will begin playing back in a loop.
    """

    limb='right'
    speed = 1.0
    accuracy = baxter_interface.settings.JOINT_ANGLE_TOLERANCE

    print("Initializing node... ")
    rospy.init_node("baxter_tomato_control", anonymous=True)

    global run_tomato_finder
    run_tomato_finder = rospy.Publisher("/find_tomato_flag", String, queue_size=1)
    
    mv = Movement(limb, speed, accuracy)
    rospy.Subscriber("/tomato_position", Pose, mv.pick)
    mv._gripper.open()

    # Register clean shutdown
    rospy.on_shutdown(mv.clean_shutdown)

    # Align the gripper to help align camera
    mv.go_to({'x':0.45,'y':-0.2,'z':0.0,'qx':1,'qy':1,'qz':0,'qw':0})
    input("Press enter when camera is set up")

    # Move camera out of the way
    mv.go_to({'x':0.45,'y':-0.2,'z':0.3,'qx':1,'qy':1,'qz':0,'qw':0})
    rospy.sleep(1.0)
    # Run the tomato finder.
    run_tomato_finder.publish("GO")

    while (not rospy.is_shutdown()):# and mv.not_picked:
        rospy.sleep(0.1)
    # Go for the pick

    # Go for that pick!
if __name__ == '__main__':
    main()
