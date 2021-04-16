#! /usr/bin/env python
import rospy

import cv2
import tensorflow as tf
import numpy as np

#THIS IS JUST TO UPLOAD SOME PICTURES AND MAKE PREDICTIONS

def prepare(filepath):
    IMG_SIZE = 100 
    img_array = cv2.imread(filepath)
    new_array = cv2.resize(img_array, (IMG_SIZE, IMG_SIZE))
    return new_array.reshape(-1, IMG_SIZE, IMG_SIZE, 3)

if __name__ == "__main__":
	CATEGORIES = ["grapes", "pineapple","watermelon"]

	model = tf.keras.models.load_model("bestcolored_cnn.model")

	prediction = model.predict([prepare('watermelon.jpeg')])
	print(CATEGORIES[int(np.argmax(prediction, axis=-1))])
