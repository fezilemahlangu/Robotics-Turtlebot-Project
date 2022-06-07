import matplotlib.pyplot as plt
from cv2 import imread, threshold, THRESH_BINARY
import numpy as np
from skimage.morphology import binary_erosion, binary_opening, disk, square
import math

# converts world co ords to pixel co ords
def conversion(x,y):
    r=math.round((6.381-x)/0.0341)
    c=math.round((12.504-y)/0.0341)
    return r,c

img = imread('img.png', 2)
plt.imshow(img, cmap='gray')
ret, bw_img = threshold(img, 127, 255, THRESH_BINARY)

# change the image to binary
# 255 is where the black areas are. We want to keep that.
cond = bw_img == 255
cond2 = bw_img < 255
bw_img[cond] = 1
bw_img[cond2] = 0


# create the kernel to perform erosion, number is how big we want the erosion to be
kernel = square(12)
# can use disk too if we want circular erosion. Depends on what is better

# smooth the image
bw_img = binary_opening(bw_img, kernel)

# erode it to make the black areas bigger
bw_img = binary_erosion(bw_img, kernel)

mapArr = np.zeros(bw_img.shape)
for i in range(bw_img.shape[0]):
    for j in range(bw_img.shape[1]):
        if (bw_img[i][j] == False):
            #obstacle
            mapArr[i][j] = 0
        else:
            #no obstacle
            mapArr[i][j] = 1
print(mapArr)

# =================================================
# IF PYTHON FILE DOES NOT WORK STILL:

# load image array for use in another place
# imgArr=[]
# with open("array.txt") as textFile:
#     for line in textFile:
#         lines=line.split(',')
#         imgArr.append(lines)

# mapArr=np.array(imgArr).astype(int)

# ======================================================

# CHANGING THE WORLD CO ORDS TO PIXEL CO ORDS:
# _______________________________________

#YOU WILL BE USING THE mapArr variable

#get goal co ord from user->
# e.g.
# goalx=int(input())
# goaly=int(input())
# convert to pixel co ords:
# goalx,goaly=conversion(goalx,goaly)

#get curr position of bot from rospy->
# e.g.
# currPosX=int(input())
# currPosY=int(input())
# convert to pixel co ords:
# currPosX, currPosY=conversion(currPosX,currPosY)





