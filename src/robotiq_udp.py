#! /usr/bin/env python
import rospy, math, os, time
import numpy as np
# Sockets
import socket, struct
from cStringIO import StringIO

# Messages
from takktile_ros.msg import RobotiqTouch, Touch
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class RobotiqUDP():
  # Initialise
  def __init__(self):
    self.ns = rospy.get_namespace()
    # Set up sender socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.write_ip = rospy.get_param('~write_ip', '127.0.0.1')
    self.write_port = int(rospy.get_param('~write_port', 6052))
    # Set up receiver socket
    self.read_port = int(rospy.get_param('~read_port', 6051))
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind(('', self.read_port))
    rospy.loginfo('UDP Socket receiving on port [%d]' % (self.read_port))
    self.step = int(rospy.get_param('~sensors_per_finger', 4))
    # Set up zeroing service
    zero_srv_name = '/takktile/zero'
    rospy.loginfo('Waiting for [%s] service' % zero_srv_name)
    rospy.wait_for_service(zero_srv_name)
    zero_srv = rospy.ServiceProxy(zero_srv_name, Empty)
    # Set-up publishers/subscribers
    rospy.Subscriber('/takktile/calibrated', Touch, self.takktile_cb)
    self.calibrated_msg = False
    rate = rospy.Rate(100)
    rospy.loginfo('Waiting for [%s] topic' % ('/takktile/calibrated'))
    while not self.calibrated_msg:
      if rospy.is_shutdown():
        return
      rate.sleep()
    rospy.loginfo('UDP Socket sending to [udp://%s:%d]' % (self.write_ip, self.write_port))
    while not rospy.is_shutdown():
      msg = Bool()
      udp_data = self.recv_timeout(0.01)
      if udp_data:
        msg.deserialize(udp_data)
        if msg.data:
          zero_srv()
  
  def recv_timeout(self, timeout=0.001):
    self.read_socket.setblocking(0)
    total_data=[]
    data=''
    begin=time.time()
    while 1:
      #if you got some data, then timeout break 
      if total_data and time.time()-begin>timeout:
        break
      #if you got no data at all, wait a little longer
      elif time.time()-begin>timeout*2:
        break
      try:
        data=self.read_socket.recv(8192)
        if data:
          total_data.append(data)
          begin=time.time()
      except:
        pass
    return ''.join(total_data)
  
  def takktile_cb(self, msg):
    self.calibrated_msg = True
    touch_msg = RobotiqTouch()
    touch_msg.f0 = msg.pressure[:self.step]
    touch_msg.f1 = msg.pressure[self.step:2*self.step]
    touch_msg.f2 = msg.pressure[2*self.step:]
    # Send over udp the tactile values
    file_str = StringIO()
    touch_msg.serialize(file_str)
    self.write_socket.sendto(file_str.getvalue(), (self.write_ip, self.write_port))


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  robotiq = RobotiqUDP()
  rospy.loginfo('Shuting down [%s] node' % node_name)
