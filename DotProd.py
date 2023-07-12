from PIL import Image
import numpy as np
import pandas as pd

img = Image.open("/home/markus/Pictures/Azov2.jpeg")
img.load
data = np.asarray(img, dtype="int32")
print(data)


