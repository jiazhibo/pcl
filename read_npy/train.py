'''
import numpy as np
import cv2
import matplotlib.pyplot as plt
image = np.load("./arr0.npy")
print(image)
for i in range(0,image.shape[0]):
    plt.imshow(image[i,:,:])
    cv2.imwrite(str(i)+".png",image[i,:,:])
    plt.show()
'''

import numpy as np
 
 
import sys
np.set_printoptions(threshold=sys.maxsize)

boxes=np.load('./01.npy')
#boxes=np.load('./proj0.npy')
print(boxes)

np.savetxt('2.txt',boxes.reshape((1,43,64,3)),fmt='%s', header=str(boxes.shape))
print('---------------------boxes--------------------------')

