#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
import tensorflow as tf
import numpy as np
import sys
import cv2

CATEGORIES = ["grapes", "pineapple","watermelon"]

def prepare(img):
    IMG_SIZE = 100 
    new_array = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
    return new_array.reshape(-1, IMG_SIZE, IMG_SIZE, 3)

def callback_camera(image_data):
	rospy.loginfo("Getting image")


	img = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	#cv2.namedWindow("Robot View", cv2.WINDOW_AUTOSIZE)
	#cv2.imshow("Robot View", img)
	#cv2.waitKey(0)
	#cv2.destroyWindow("Robot View")
	#rospy.loginfo("Showed the image")

	model = tf.keras.models.load_model("bestcolored_cnn.model")

	prediction = model.predict([prepare(img)])
	predicted_fruit = CATEGORIES[int(np.argmax(prediction, axis=-1))]
	if (predicted_fruit == "watermelon"):
		return True
	else:
		return False

def camera_subscriber(img):
	return

if __name__ == '__main__':
	try:

		rospy.init_node('watermelon_finder')
	
		rospy.Subscriber("/camera/rgb/image_raw", Image, camera_subscriber)
		client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		client.wait_for_server()

		goal = MoveBaseGoal()
		
		# watermelon -6.38,-1.5, 1.0, 0.0....-7.16,-2.18, 1.0, -0.08
		#pineapple 
		goal_coords = [[4.05, -4.43, 1.0, 0.0],[-7.16,-2.18, 1.0, -0.08],[5.42, 2.72, -0.7, 0.7]]
		msg = None
		for x,y,z,w in goal_coords:
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
			goal.target_pose.pose.orientation.z = z
			goal.target_pose.pose.orientation.w = w

			print("Sending goal");
			client.send_goal(goal)
			client.wait_for_result()
			
			if (client.get_state() == GoalStatus.SUCCEEDED):
				rospy.loginfo("The base successfully moved to (%.2f, %.2f)", x, y)
				msg = rospy.wait_for_message("/camera/rgb/image_raw", Image)
				value = callback_camera(msg)
				if (value == True):
					print("Found a watermelon! :)")
				else:
					print("No watermelon here :(")
			else:
				rospy.loginfo("The base failed to move to (%.2f, %.2f)", x, y)
			rospy.sleep(2)
	except rospy.ROSInterruptException:
		rospy.loginfo("Terminated")
	
