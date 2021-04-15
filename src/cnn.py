import cv2
import tensorflow as tf
import numpy as np

CATEGORIES = ["grapes", "pineapple","watermelon"]

def prepare(filepath):
    IMG_SIZE = 50  # 50 in txt-based
    img_array = cv2.imread(filepath)
    new_array = cv2.resize(img_array, (IMG_SIZE, IMG_SIZE))
    return new_array.reshape(-1, IMG_SIZE, IMG_SIZE, 3)

model = tf.keras.models.load_model("bestcolored_cnn.model")

prediction = model.predict([prepare('grapee.jpg')])
print(CATEGORIES[int(np.argmax(prediction, axis=-1))])
