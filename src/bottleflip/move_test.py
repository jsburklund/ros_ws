#!/usr/bin/python2

from copy import copy
import math
import rospy
import baxter_interface
import actionlib
import sys
from baxter_interface import CHECK_VERSION
from baxter_interface import Gripper, Limb

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from sensor_msgs.msg import JointState

from std_msgs.msg import Header, Empty

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint

from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from tf.transformations import quaternion_from_euler

from std_msgs.msg import (UInt16,)

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


global right_gripper
global right_limb

if __name__ == '__main__':
  rospy.init_node("move_test")

  ns = "ExternalTools/right/PositionKinematicsNode/IKService"
	
  rospy.wait_for_message("/robot/sim/started", Empty)
		
  iksvc = rospy.ServiceProxy(ns, SolvePositionIK)	
  global right_gripper

  pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                      UInt16, queue_size=10)
  left_arm = baxter_interface.limb.Limb("left")
  right_arm = baxter_interface.limb.Limb("right")
  left_joint_names = left_arm.joint_names()
  right_joint_names = right_arm.joint_names()
  
  _rate = 500.0
  #rospy.sleep(rospy.Duration(5,0))

  rs = baxter_interface.RobotEnable(CHECK_VERSION)
  rs.enable()

  right_gripper = Gripper('right');
  right_limb = Limb('right');

  right_gripper.calibrate()
  hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	
  print "opening gripper"
  right_gripper.open()

  current_angles = [right_limb.joint_angle(joint) for joint in right_limb.joint_names()]

  orient_quaternion_components = quaternion_from_euler(math.pi, 0,math.pi/2)
  orient_down = Quaternion()
  orient_down.x = orient_quaternion_components[0]
  orient_down.y = orient_quaternion_components[1]
  orient_down.z = orient_quaternion_components[2]
  orient_down.w = orient_quaternion_components[3]
	
  highPose = PoseStamped(header=hdr, pose=Pose(position=Point(0, -0.428, -0.57), orientation=orient_down))
  gripPose = PoseStamped(header=hdr, pose=Pose(position=Point(0, -0.428, -0.71), orientation=orient_down))
  liftPose = PoseStamped(header=hdr, pose=Pose(position=Point(0, -0.428, -0.5), orientation=orient_down))
	
  ikreq = SolvePositionIKRequest()
  ikreq.pose_stamp.append(highPose)
  ikreq.pose_stamp.append(gripPose)
  ikreq.pose_stamp.append(liftPose)
  seedstate = JointState()
  seedstate.name=('right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2')
  seedstate.position=current_angles
  #ikreq.seed_angles.append(seedstate)
  #ikreq.seed_angles.append(seedstate)


  try:
    rospy.wait_for_service(ns, 5.0)
    resp = iksvc(ikreq)
  except (rospy.ServiceException, rospy.ROSException), e:
    rospy.logerr("Service call failed to IK service: %s" % (e))
    print "Service call failed"

  print resp

  # S0 S1 E0 E1 W0 W1 W2
  reset_angles = [-0.2731084643631423, 1.047000013603367, -0.0030268105104838128, 0.4970772672879358, -0.08320234128402682, 0.026794408527332614, 0.026429631701369694]
  #reset_angles = [-0.2731084643631423, 1, -0.0030268105104838128, 0.4970772672879358, -0.08320234128402682, 0.026794408527332614, 0.026429631701369694]

  traj = Trajectory('right')
  traj.add_point(current_angles, 0.0)
  traj.add_point(resp.joints[0].position, 3.0)
  traj.add_point(resp.joints[1].position, 6.0)
  
  traj.start()
  traj.wait(20.0)
  print(current_angles)
  right_gripper.close()
  
  current_angles = [right_limb.joint_angle(joint) for joint in right_limb.joint_names()]
  traj.clear('right')
  traj.add_point(current_angles, 0)
  traj.add_point(resp.joints[2].position, 2.5)
  traj.start()
  traj.wait(3)
  print('Object picked up')
  print(rospy.get_rostime().secs)
  
  #Get the arm ready for launch
  traj.clear('right')
  current_angles = [right_limb.joint_angle(joint) for joint in right_limb.joint_names()]
  launch_ready = current_angles
  launch_ready[3] = math.pi/2
  launch_ready[5] = 0 
  traj.add_point(current_angles, 0.0)
  traj.add_point(launch_ready, 10.0)  # Go to launch prep slow for now
  traj.start()
  traj.wait(20.0)
  print('At launch point')
  print(rospy.get_rostime().secs)


  '''traj.clear('right')
  current_angles = [right_limb.joint_angle(joint) for joint in right_limb.joint_names()]
  launch_end = current_angles
  launch_end[3] = math.pi/2-(0.9996*0.5)
  launch_end[5] = (0.9996*0.5)
  launch_end   = [0, 0, 0, math.pi/2-(0.9996*0.5), math.pi/2-(0.9996), 0, 0]  
  traj.add_point(current_angles, 0.0)
  traj.add_point(launch_end, 10.0)     # Launch fast?
  print('Launching in...')
  for i in range(0,5):
  print(5-i)
  rospy.sleep(1.0)
  print('Launching')
  traj.start()
  traj.wait(20)
  right_gripper.open()'''
  
  '''print('Launching in...')
  for i in range(0,5):
  print(5-i)
  rospy.sleep(1.0)
  print('Launching')'''


  # Set the control mode
        

  
  print("Getting robot state...")
  
  # Set joint state publishing to 500Hz
  pub_rate.publish(_rate)
  
  # Set the arms to the neutral position
  #left_arm.move_to_neutral()
  #right_arm.move_to_neutral()
  
  rate = rospy.Rate(_rate)

  # Make function for velocities
  def get_v_function(A_params):
    def v_func(t):
      return -1*(A_params[1] + 2*A_params[2]*t + 3*A_params[3]*t*t)

    return v_func

  #rjoint_vels = dict([(joint, 0.0) for i, joint in enumerate(right_joint_names)])
  #ljoint_vels = dict([(joint, 0.0) for i, joint in enumerate(left_joint_names)])
  #print(rjoint_vels)
  rjoint_vels = {'right_e1':0, 'right_w1':0}
  #rjoint_vels['right_e1'] = -1.75
  #rjoint_vels['right_w1'] = -3.5
  my_A_params = [0, 0, 3157729/999600, 0] 
  print('Starting velocities')
  start = rospy.Time.now().to_sec()
  launchtime = 0.556

  e1_fun = get_v_function(my_A_params)
  w1_fun = get_v_function(my_A_params)
  while rospy.Time.now().to_sec()-start < launchtime:
    elapsed = rospy.Time.now().to_sec() - start
    pub_rate.publish(_rate)
    rjoint_vels['right_e1'] = e1_fun(elapsed)
    rjoint_vels['right_w1'] = 2*w1_fun(elapsed)
    right_arm.set_joint_velocities(rjoint_vels)
    rate.sleep()

  right_gripper.open()
  print('Gripper released')
  # Spin the velocities at zero for a few seconds to help release
  rjoint_vels = dict([(joint, 0.0) for i, joint in enumerate(right_joint_names)])
  spintime = 1.0
  print('Spin time')
  elapsed = rospy.Time.now() - start
  while rospy.Time.now().to_sec()-start < spintime:
    pub_rate.publish(_rate)
    right_arm.set_joint_velocities(rjoint_vels)
    rate.sleep()
  print('Spin done')
