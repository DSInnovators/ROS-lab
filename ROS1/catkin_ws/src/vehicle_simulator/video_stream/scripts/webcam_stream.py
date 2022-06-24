#!/usr/bin/env python3
from email.header import Header
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

vid = None

def main():
  global vid
  pub = rospy.Publisher('camera/raw', Image, queue_size=10)
  rospy.init_node('video_stream')
  rate = rospy.Rate(20)
  cvbridge_obj = CvBridge()

  vid = cv2.VideoCapture(0)
  counter=0
  while not rospy.is_shutdown():
    ret, frame = vid.read()

    print(type(frame))
    header = Header()
    header.seq = counter
    header.stamp = rospy.Time.now()


    newframe = cvbridge_obj.cv2_to_imgmsg(header, frame, "bgr8")

    cv2.imshow('frame', frame)
    #hello_str = "hello world %s" % rospy.get_time()
    #rospy.loginfo(hello_str)

    pub.publish(newframe)
    rate.sleep()
    counter+=1

if __name__ == '__main__':
    try:
      main()
    except rospy.ROSInterruptException:
      pass
    finally:
      vid.release()
      cv2.destroyAllWindows()



# After the loop release the cap object
