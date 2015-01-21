#!/usr/bin/env python
import rospy, os, datetime
import numpy as np
import scipy.io as sio
# Messages
from geometry_msgs.msg import Wrench, WrenchStamped
from takktile_ros.msg import Touch
from std_srvs.srv import Empty

# Helpers
DEFAULT_TYPE = np.float32


class TakktileCalibration(object):
  def __init__(self):
    # Read from parameter server
    self.filename_key = rospy.get_param('~filename_key', 'calibration')
    self.folder = rospy.get_param('~folder', 'takktile_data')
    # Initial values
    self.whole_data = dict()
    self.whole_data['time'] = []
    self.whole_data['wrench'] = []
    self.whole_data['pressure'] = []
    self.netft_msg = None
    self.takktile_msg = None
    self.data_count = 0
    # Set up zeroing service
    zero_srv_name = '/takktile/zero'
    rospy.loginfo('Waiting for [%s] service' % zero_srv_name)
    rospy.wait_for_service(zero_srv_name)
    zero_srv = rospy.ServiceProxy(zero_srv_name, Empty)
    # Set-up subscribers
    rospy.Subscriber('/takktile/calibrated', Touch, self.takktile_cb)
    rospy.Subscriber('/netft/data', WrenchStamped, self.netft_cb)
    rospy.loginfo('Waiting for topics to start logging')
    while None in [self.netft_msg, self.takktile_msg]:
      if rospy.is_shutdown():
        exit(0)
      rospy.sleep(0.1)
    # Shutdown hookup for saving the matfile
    rospy.on_shutdown(self.shutdown)
    rospy.spin()
  
  def takktile_cb(self, msg):
    self.takktile_msg = msg
    if None in [self.netft_msg, self.takktile_msg]:
      return
    self.whole_data['time'].append(rospy.Time.now().to_sec())
    self.whole_data['pressure'].append(toNumpyArray(self.takktile_msg.pressure))
    self.whole_data['wrench'].append(toNumpyArray(self.netft_msg.wrench))
    if (self.data_count % 1000  == 0):
      rospy.loginfo('Logged %d data points' % (self.data_count + 1))
    self.data_count += 1
  
  def netft_cb(self, msg):
    self.netft_msg = msg
  
  def shutdown(self):
    # Add timestamp to the filename
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    filename = os.path.expanduser('~/%s/%s-%s.mat' % (self.folder, self.filename_key, timestamp))
    # In case the folder doesn't exists, to make it
    try:
      os.makedirs(os.path.split(filename)[0])
    except OSError, e:
      pass
    sio.savemat(filename, self.whole_data, oned_as='column')
    rospy.loginfo('Saved %d data points to: %s' % (self.data_count, filename))

def toNumpyArray(data):
  array = None
  if type(data) is Wrench:
    array = np.zeros((1,7), dtype=DEFAULT_TYPE)
    array[0,0] = data.force.x
    array[0,1] = data.force.y
    array[0,2] = data.force.z
    array[0,3] = data.torque.x
    array[0,4] = data.torque.y
    array[0,5] = data.torque.z
  elif type(data) in [list, tuple]:
    array = np.array(data, dtype=DEFAULT_TYPE)
  return array


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  tc = TakktileCalibration()
