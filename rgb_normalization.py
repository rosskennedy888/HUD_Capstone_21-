import numpy as np
import cv2 as cv

def normalized(img):
        img_width = len(img[0])
        img_height = len(img)
        normalized_img = np.zeros((img_height, img_width, 3), np.uint8)
        all_red = img[:, :, 2]
        all_blue = img[:, :, 1]
        all_green = img[:, :, 0]
        sum_of_all_colors = all_red + all_blue + all_green
        normalized_img[:, :, 2] = (all_red/sum_of_all_colors) * 255.0
        normalized_img[:, :, 1] = (all_blue/sum_of_all_colors) * 255.0
        normalized_img[:, :, 0] = (all_green/sum_of_all_colors) * 255.0
        return cv.convertScaleAbs(normalized_img)
        
    
img = cv.imread('rgb_coor.jpg')
rgb_img = normalized(img)
print(img)
print(rgb_img)
cv.imshow("frame",rgb_img)
#print(rgb_img)
img = cv.imwrite('bull1.jpg', rgb_img)