#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from final_project.msg import GuessedLetter
from std_msgs.msg import String

class demo_cam:
    def __init__(self):
        rospy.init_node('letter_finder')
        self.grab_status = rospy.Subscriber('next_step', String, self.action_check)
        self.letter_pub = rospy.Publisher('cur_letter', GuessedLetter, queue_size=1)
        cv2.namedWindow('feed', 1)
        self.latest_window = None
        self.new_image = False
        self.vc = cv2.VideoCapture(0)

    def camera_feed(self, msg):
        if msg is not None:
            if self.vc.isOpened(): # try to get the first frame
                self.new_image = True
                self.rval, self.frame = self.vc.read()
            else:
                self.rval = False
        else:
            while self.check == None:
                rospy.sleep(0.5)
    
    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.new_image:
                cv2.imshow("feed", self.frame)
                self.rval, self.frame = self.vc.read()
                cv2.waitKey(20)
                self.new_image = False
            rate.sleep()
        self.vc.release()
        cv2.destroyWindow("feed")

if __name__ == '__main__':
    demo = demo_cam()
    demo.run()