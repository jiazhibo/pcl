
import numpy as np
from PIL import Image
import os
import tensorflow as tf
import cv2
# #####################  im 2 npy  #################


if __name__=='__main__':


  im1 = cv2.imread('./01.jpg')
  im2=np.array(im1)
  # print(type(im2))
  im3 = np.expand_dims(im2, 0)
  np.save('./01.npy',im2)
  np.save('./02.npy',im3)
  print(im2.shape)
  print(im3.shape)

  print("------show npy's shape_size---------")
  data = np.load('./01.npy')
  print(data.shape)
  data2 = np.load('./02.npy')
  print(data2.shape)
  print(data)

