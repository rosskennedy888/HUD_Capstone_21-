#this program takes GPS data from the arduino and uses images taken from
#raspberrypi to perform template matching on in order to detect 3 lights
#
#Authurs: Ross Kennedy, Ashely Bey
#Senior Capstone 2020/2021

import serial
import haversine as hs    #converts GPS coordinates to KM/miles
from haversine import Unit
from picamera import PiCamera
from signal import pause
from time import strftime
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
import time
import I2C_LCD_driver

import numpy as np
import argparse
import imutils
import glob
import cv2

mylcd = I2C_LCD_driver.lcd()
 
camera = PiCamera()

#Three light locations
light_A = (39.51036,-84.73078) 
light_B = (39.51031,-84.73916)
light_C = (39.51035,-84.73483)

lat = 0
long = 0
distance_detect = 0.08    #distance threshold for detecting a light 

#file to save test images to
with open("images_and_coordinates.txt", "a") as f:
    f.write("Start\n")    #begin writting to file

def tempate_matching():
    # construct the argument parser and parse the arguments
    template = "green_template.jpg"
    # load the image image, convert it to grayscale, and detect edges
    template = cv2.imread('green_template.jpg')
    template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    template = cv2.Canny(template, 50, 200)
    (tH, tW) = template.shape[:2]
    visualize = True
    cv2.imshow("template", template)
    # loop over the images to find the template in
    for imagePath in glob.glob(('/home/pi/Downloads' + full_date_time + '.jpg')):
        # load the image, convert it to grayscale, and initialize the
        # bookkeeping variable to keep track of the matched region
        image = cv2.imread(imagePath)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = None
        # loop over the scales of the image
        for scale in np.linspace(0.2, 1.0, 20)[::-1]:
            # resize the image according to the scale, and keep track
            # of the ratio of the resizing
            resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
            r = gray.shape[1] / float(resized.shape[1])
            # if the resized image is smaller than the template, then break
            # from the loop
            if resized.shape[0] < tH or resized.shape[1] < tW:
                break
            # detect edges in the resized, grayscale image and apply template
            # matching to find the template in the image
            edged = cv2.Canny(resized, 50, 200)
            result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
            # check to see if the iteration should be visualized
            if visualize:
                # draw a bounding box around the detected region
                clone = np.dstack([edged, edged, edged])
                cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),
                    (maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
                mylcd.lcd_display_string("Light Matched!",4,2)    #Show if there was a box drawn
                print("Light Detected!!")
                #cv2.imshow("Visualize", clone)
                #cv2.waitKey(0)
            # if we have found a new maximum correlation value, then update
            # the bookkeeping variable
            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc, r)
        # unpack the bookkeeping variable and compute the (x, y) coordinates
        # of the bounding box based on the resized ratio
        (_, maxLoc, r) = found
        (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
        (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
        # draw a bounding box around the detected result and display the image
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
        #cv2.imshow("Image", image)
        #cv2.waitKey(0)

def take_picture_with_camera():
    global led
    full_date_time = strftime("_%m-%d-%y_%I:%M:%S%p") # Results in _23/10/17_09:20AM
    image_path = ('/home/pi/Downloads' + full_date_time + '.jpg')
    with open("images_and_coordinates.txt", "a") as f:    #open image file
        f.write("\n" + image_path + "\n")
    print(image_path)
    camera.capture(image_path)
    print('Took photo')

if __name__ == '__main__':

    mylcd.backlight(1);
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    #Display latitiude and longitude to shell from arduino
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            if(line.find("latitude") != -1):
                lat = float(line[10:])
                light_found = 1
            elif(line.find("longitude") != -1):
                long = float(line[11:])
                light_found = 1
            else:
                  print(line)
                  light_found = 0
            
            if (light_found == 1):
                loc = (lat, long)
                print("Distance from light A: ")
                distance = hs.haversine(loc, light_A, unit=Unit.MILES)
                print(distance)
                #when within threshold of light A, write distasnce from it in miles and display 
                if (distance <= distance_detect):
                    print("LIGHT A")
                    take_picture_with_camera()
                    with open("images_and_coordinates.txt", "a") as f:
                        f.write("Distance from LightA: " + str(distance) + "\n")
                        f.write("Lat: " + str(lat) + "\n")
                        f.write("Long: " + str(long) + "\n")
                    mylcd.lcd_display_string("Light Ahead!",1,5)
                    mylcd.lcd_display_string("Prepare to stop",3,2)
    
    
                print("Distance from light B: ")
                distance = hs.haversine(loc, light_B, unit=Unit.MILES)
                print(distance)
                #when within threshold of light B, write distasnce from it in miles and display
                if (distance <= distance_detect):
                    print("LIGHT B")
                    take_picture_with_camera()
                    with open("images_and_coordinates.txt", "a") as f:
                        f.write("Distance from LightB: " + str(distance) + "\n")
                        f.write("Lat: " + str(lat) + "\n")
                        f.write("Long: " + str(long) + "\n")
                    mylcd.lcd_display_string("Light Ahead!",1,5)
                    mylcd.lcd_display_string("Prepare to stop",3,2)
                    
                print("Distance from light C: ")
                distance = hs.haversine(loc, light_C, unit=Unit.MILES)
                print(distance)
                #when within threshold of light C, write distasnce from it in miles and display
                if (distance <= distance_detect):
                    print("LIGHT C")
                    take_picture_with_camera()
                    with open("images_and_coordinates.txt", "a") as f:
                        f.write("Distance from LightC: " + str(distance) + "\n")
                        f.write("Lat: " + str(lat) + "\n")
                        f.write("Long: " + str(long) + "\n")
                    mylcd.lcd_display_string("Light Ahead!",1,5)
                    mylcd.lcd_display_string("Prepare to stop",3,2)
            else:
                mylcd.lcd_clear()
                