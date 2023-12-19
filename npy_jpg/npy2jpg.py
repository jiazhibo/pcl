import cv2
import numpy as np
import sys

import PIL
from PIL import Image

np.set_printoptions(threshold=sys.maxsize)

data = np.load("./arr0.npy")
#print(data.shape)
print(data)
a = np.squeeze(data, 0)
print(a)
for i,k in enumerate(a):
	a[i]=(k+1)*255
print(a)
cv2.imwrite("./arr0.jpg", a)

im1 = cv2.imread('./arr0.jpg')
print(im1.shape)
print(im1)


#np.savetxt('data.txt', a, fmt='%0.18f')


np.savetxt('2.txt',a,fmt='%s',newline='\n')
print('---------------------boxes--------------------------')

