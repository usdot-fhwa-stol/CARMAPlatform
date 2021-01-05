#!/usr/bin/env python

# Copyright (C) 2020-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import rospy
import rostopic
import rosgraph
import sys
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# USAGE:
# This script converts geometry_msgs/Quaternion into extrinsic Roll Pitch Yaw values
# The output is a PoseArray message where the position x,y,z fields are actually set to the RPY values in radians and the orientation field is set to the original quaternion.
#
# python extract_rpy_from_quat.py <fully namespaced input topic> <fully namespaced output topic> <string describing the address of quaternion> <optional: string describing address of list which should be used as base reference for element address> 
#
# Examples:
#  If someone is publishing a PoseStamped topic I would call this like
#  python extract_rpy_from_quat.py /my_in_topic /my_out_topic pose.orientation
#
#  Or if there was a PoseArray being published I would call it using the list optional arg like
#  python extract_rpy_from_quat.py /my_in_topic /my_out_topic orientation poses
#
#  To be more specific this is what it would look like for /guidance/base_waypoints running inside carma exec where I copied the script to /opt/carma/maps
#  python extract_rpy_from_quat.py /guidance/base_waypoints /base_waypoint_rpy pose.pose.orientation waypoints
#

class Converter:
    def __init__(self, pub, element_attr, list_attr=''):
        self.element_attr_array = element_attr.split('.')
        self.list_attr_array = list_attr.split('.')
        self.pub = pub

    def getElementFromAttributeArray(self, attr_array, data):
      attr_list = []
      for attr in attr_array:
        if len(attr_list) == 0:
          attr_list.append(getattr(data, attr))
        else:
          attr_list.append(getattr(attr_list[-1], attr))
      
      return attr_list[-1]

    def normalize(self, array):
        """ 
        Normalize a 4 element array/list/numpy.array for use as a quaternion
        :param quat_array: 4 element list/array
        :returns: normalized array
        :rtype: numpy array
        """
        quat = np.array(array)
        return quat / np.sqrt(np.dot(quat, quat))

    def convertQuatToVector3(self, quat):
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        orientation_list = self.normalize(orientation_list)
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list, axes='sxyz')
        vector_msg = Vector3()
        vector_msg.x = roll
        vector_msg.y = pitch
        vector_msg.z = yaw
        return vector_msg

    def handle_list(self, msg):
      list_ele = self.getElementFromAttributeArray(self.list_attr_array, msg)

      pose_array_msg = PoseArray()
      for e in list_ele:
        quat = self.getElementFromAttributeArray(self.element_attr_array, e)
        fake_pose = Pose()
        fake_pose.position = self.convertQuatToVector3(quat)
        fake_pose.orientation = quat
        pose_array_msg.poses.append(fake_pose)

      self.pub.publish(pose_array_msg)

    def handle_individual(self, msg):
      quat = self.getElementFromAttributeArray(self.element_attr_array, msg)

      pose_array_msg = PoseArray()

      fake_pose = Pose()
      fake_pose.position = self.convertQuatToVector3(quat)
      fake_pose.orientation = quat
      pose_array_msg.poses.append(fake_pose)
      
      self.pub.publish(pose_array_msg)



def run(args):
  rospy.init_node('extract_rpy_from_quat')

  in_topic = args[1]
  out_topic = args[2]
  element_accessor = args[3]

  pub = rospy.Publisher(out_topic, PoseArray, queue_size=10)


  converter = Converter(pub, element_accessor)
  callback = converter.handle_individual

  if len(args) == 5:
    converter = Converter(pub, element_accessor, args[4])
    callback = converter.handle_list

  print(in_topic)
  msg_class, _, _ = rostopic.get_topic_class(in_topic, True)

  sub = rospy.Subscriber(in_topic, msg_class, callback)

  r = rospy.Rate(20)
  while not rospy.is_shutdown():
      r.sleep()


if __name__ == '__main__':
    try:
      if (len(sys.argv) < 4 or len(sys.argv) > 5):
        print 'At one-two arguments required extract_rpy_from_quat <in topic> <out topic> <element path> <list path>'

      run(sys.argv)
    except rospy.ROSInterruptException:
        print("ROS Exception")
        pass
        