#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae, Vladimir Ermakov
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233
#    - Use mavros.setpoint module for topics

import rospy
import thread
import threading
import time
import mavros

from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler


class SetpointPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_position_local(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'local'),
                                    SP.PoseStamped, self.reached)

        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()

    def navigate(self):
        rate = rospy.Rate(10)   # 10hz

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

        while not rospy.is_shutdown():
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
            quaternion = quaternion_from_euler(0, 0, self.yaw)
	    #msg.pose.orientation.x = quaternion[0]
	    #msg.pose.orientation.y = quaternion[1]
	    #msg.pose.orientation.z = quaternion[2]
	    #msg.pose.orientation.w = quaternion[3]

            # For demo purposes we will lock yaw/heading to north.
            #yaw_degrees = 180  # North
            #yaw = radians(yaw_degrees)
            #quaternion = quaternion_from_euler(0, 0, self.yaw)
            msg.pose.orientation = SP.Quaternion(*quaternion)

            self.pub.publish(msg)
            rate.sleep()

    def set(self, x, y, z, yaw, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        if wait:
            rate = rospy.Rate(10)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()

        time.sleep(delay)

    def reached(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
	    #rospy.loginfo("Position %s: local: %d, target: %d, abs diff: %d",
                           #msg, x, y, abs(x - y))
            return abs(x - y) < 0.2
	  #I added is_near_quet since it rotates sometimes with abs diff of 1 (I will fix it later )
        def is_near_quet(msg, x, y):
	    rospy.logdebug("orientation %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) <= 2.0	
	quaternionf = quaternion_from_euler(0, 0, self.yaw)
        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z) and \
	   is_near_quet('x', topic.pose.orientation.x, quaternionf[0]) and \
	   is_near_quet('y', topic.pose.orientation.y, quaternionf[1]) and \
	   is_near_quet('z', topic.pose.orientation.z, quaternionf[2]) and \
	   is_near_quet('w', topic.pose.orientation.w, quaternionf[3]):
	   
            self.done = True
            self.done_evt.set()
            
#def my_range(start, end, step, num):
    #while start <= end:
        #yield start
        #if num < 0:
	  #start -= step
	#else:
	  #start += step

def setpoint_demo():
    rospy.init_node('setpoint_position_demo')
    #mavros.set_namespace()  # initialize mavros module with default namespace
    mavros.set_namespace('/iris/mavros')    
    rate = rospy.Rate(10)

    setpoint = SetpointPosition()
    
    #read from a file
    theFile = open("/home/randa/workspace/catkin_ws/src/aircraft_inspection/inspection/scripts/path_check.txt", "r")
    
    rospy.loginfo("File Open")
    
    begin= rospy.get_time()    # stamp should update
    
    rospy.loginfo("TAKEOFF 1")
    setpoint.set(4.5, 24.0, 5.0, 3.14, 5)
    rospy.loginfo("TAKEOFF 2")
    setpoint.set(4.5, 24.0, 7.0, 3.14, 6)
    rospy.loginfo("TAKEOFF 4")
    setpoint.set(4.5, 24.0, 7.0, 2.3562, 6)
    
    
    rospy.loginfo("MOVING using data from the file")
    #setpoint.set(4.0, -29.0, 9.0, -2.3562, 8)
    data = theFile.readlines()
    temp_vals = []
    temp_vals.append(7.0)
    index=0
    for line in data:
      index = index+1
      poses = line.split()
      print float(poses[0])
      print float(poses[1])
      print float(poses[2])
      print float(poses[3])
      temp_vals.append(float(poses[2]))
      diff = temp_vals[index]-temp_vals[index-1]
      setpoint.set(float(poses[0]), float(poses[1]), temp_vals[index-1], float(poses[3]), 10)
      print diff
      if diff>1.5: # and temp_vals[index]>1.5:
	print "diff>1"
	start = temp_vals[index-1]
	end = float(poses[2])
	step = 1.5
	while start < end: # and temp_vals[index]>1.5:
	    start += step
	    setpoint.set(float(poses[0]), float(poses[1]), start, float(poses[3]), 4)
      elif diff<-1.5: # and temp_vals[index]>1.5:
	print "diff<-1"
	start = temp_vals[index-1]
	end = float(poses[2])
	step = 1.5
	while start > end:
	    start -= step
	    setpoint.set(float(poses[0]), float(poses[1]), start, float(poses[3]), 4)
      else:
	print "nothing"
	#if temp_vals[index]>1.5:
	setpoint.set(float(poses[0]), float(poses[1]), float(poses[2]), float(poses[3]), 4)#12

    ######testing###
    #setpoint.set(4.0, -29.0, 9.0, -2.3562, 8)
    #setpoint.set(4.0, -29.0, 9.0, 2.3562, 8)
    #rospy.loginfo("MOVING 3")
    #setpoint.set(6.0, -29.0, 7.0, 3.14159, 8)
    #setpoint.set(6.0, -29.0, 5.0, 2.3562, 8)
    end= rospy.get_time()   # stamp should update
    elapsed =end - begin 
    rospy.loginfo("elapsed time: %d s",elapsed)
    rospy.loginfo("DONE")
    theFile.close()

    #Simulate a slow landing.
    #setpoint.set(4.0, -28.0,  3.0, 180, 5)
    #setpoint.set(4.0, -28.0,  2.0, 180, 2)
    #setpoint.set(4.0, -28.0,  1.0, 180, 2)
    #setpoint.set(4.0, -28.0,  0.0, 180, 2)
    #setpoint.set(4.0, -28.0, -0.2, 180, 2)

    rospy.loginfo("Bye!")


if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass
