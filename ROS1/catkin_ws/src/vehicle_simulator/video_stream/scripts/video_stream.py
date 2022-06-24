#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image

def main():
  rospy.init_node('video_stream')
  pub = rospy.Publisher('camera/raw',Image, queue_size=10)
  
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    ret, frame = cap.read()

    img = Image()
    img.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    img.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    

    img.encoding = cv2.CAP_PROP_FOURCC
    img.header.frame_id = "camera_link"
    img.data = frame.data
    print()
    print()
    print(cap.get(cv2.CAP_PROP_FOURCC))
    print(cap.get(cv2.CAP_PROP_POS_FRAMES))
    print(cap.get(cv2.CAP_PROP_POS_MSEC))
    print(cap.get(cv2.CAP_PROP_FORMAT))


    pub.publish(img)
    rate.sleep()

  cap.release()


if __name__ == '__main__':
  cap = cv2.VideoCapture(0)
  try:
    main()
  except rospy.ROSInterruptException:
    cap.release()
    pass



# sensor_msgs/Image

# std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
# uint32 height
# uint32 width
# string encoding
# uint8 is_bigendian
# uint32 step
# uint8[] data