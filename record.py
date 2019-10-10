import rospy
from intera_core_msgs.msg import (
    JointCommand,
    EndpointState,
    EndpointStates,
    CollisionDetectionState,
)
import numpy as np
import pandas as pd
import csv
import os

def callback(data):
  global start_time
  if start_time is None:
    start_time = rospy.Time.now().to_sec()
  f.write("%f," % ((rospy.Time.now().to_sec() - start_time,)))
  f.write("%f,%f,%f,%f,%f,%f,%f,%f,%f\n"%(data.pose.position.x, data.pose.position.y, data.pose.position.z, 
      data.twist.linear.x, data.twist.linear.y, data.twist.linear.z, 
      data.wrench.force.x, data.wrench.force.y, data.wrench.force.z))
        
     
def listener():
  rospy.init_node('record')
  rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, callback)
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()



if __name__ == '__main__':
  f = open('record2', 'w')
  f.write('time, position_x, position_y, position_z,'+ 
      'velocity_x, velocity_y, velocit_y, force_x, force_y, force_z\n')
  states = []
  start_time = None
  listener()
  f.close()

