#!/usr/bin/python3

import cv2
import tensorflow as tf
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CATEGORIES = ["grapes", "pineapple", "watermelon"]

FROM_NAV_TOPIC = "/cpptopy"
TO_NAV_TOPIC = "/pytocpp"

model = tf.keras.models.load_model("src/StrongA/src/bestcolored_cnn.model")
bridge = CvBridge()
pub = rospy.Publisher(TO_NAV_TOPIC, String, queue_size=1)

def callback(ros_data):
    cv_img = bridge.imgmsg_to_cv2(ros_data, desired_encoding="bgr8")
    prediction = model.predict(cv_img)
    category = CATEGORIES[int(np.argmax(prediction, axis=-1))]
    print(category)
    if category == 'watermelon':
        pub.publish("True")
    else:
        pub.publish("False")

if __name__ == '__main__':
    rospy.init_node('watermelon_identifier', anonymous=True)
    sub = rospy.Subscriber(FROM_NAV_TOPIC, Image, callback, queue_size=1)
    rospy.spin()
    
