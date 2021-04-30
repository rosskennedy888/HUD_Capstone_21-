import cv2
import numpy as np

cap = cv2.VideoCapture(0)
if cap.isOpened():
    while(True):
        ret, frame = cap.read()
        # blurring the frame that's captured
        frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
        # converting BGR to HSV
        hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)
        # the range of blue color in HSV
        lower_green = np.array([29, 86, 6]) # red values
        higher_green = np.array([64, 255, 255])
        # getting the range of green color in frame
        #green_range = cv2.inRange(hsv, lower_green, higher_green)  #not sure why this was here
        #res_green = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=green_range)
        green_s_gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
        canny_edge = cv2.Canny(green_s_gray, 50, 240)
        # applying HoughCircles
        circles = cv2.HoughCircles(canny_edge, cv2.HOUGH_GRADIENT, dp=1, minDist=6, param1=10, param2=10, minRadius=10, maxRadius=50)
        cir_cen = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                # drawing on detected circle and its center
                # cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                # cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
                # cir_cen.append((i[0],i[1]))
        print (cir_cen)
        cv2.imshow('circles', frame)
        cv2.imshow('gray', green_s_gray)
        cv2.imshow('canny', canny_edge)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()
else:
    print ('no cam')