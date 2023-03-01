import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac

print(dir(ac))

MAX_DISTANCE = 4

def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame 

class UserRect():
    def __init__(self) -> None:
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

selectRect = UserRect()

followRect = UserRect()

        

def avoid_obstacles(frame1):  
    #from camera we receiving grascale image
    #cv2 have only RGB to HSV or BRG to HSV converters
    #so just convert uor grascale to BRG first
    framebgr = cv2.cvtColor(frame1, cv2.COLOR_GRAY2BGR)   
 
    # Cropping an image
    cropped_framebgr = framebgr[60:120, 80:160]



    #now apply blut filter
    frame = cv2.blur(cropped_framebgr,(20,20))
    #and convert ho HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #define colors for mask
    lower_black = np.array([0,0,0])
    upper_black = np.array([180,255,40])
    #get mask from HSV image
    mask = cv2.inRange(hsv, lower_black, upper_black)
    #
    res = cv2.bitwise_not(mask)
    cv2.imshow('mask',mask)
    contours,_ = cv2.findContours(res,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, 0, (0,255,0), 3)
    #cv2.imshow('res',res)
    coordinates = (30,20)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.5
    color = (255,0,255)
    thickness = 1


    try:
        text = ""
        cnt = contours[0]

        M = cv2.moments(cnt)
        try:
            #avoid obstacles LEFT\RIGHT direction
            cx = 0
            cx = int(M['m10']/M['m00']) 
            #avoid obstacles UP\DOWN direction
            cy = 0
            cy = int(M['m01']/M['m00'])

        except ZeroDivisionError: # if there is no obstacles
            text = "forward"


        if cy<30:
            text = "down"
            #Send signal to motor driver to turn right
        else:
            text = "up" 
            #Send signal to motor driver to turn left

        if cx<40:
            text += " right"
            #Send signal to motor driver to turn right
        else:
            text += " left" 
            #Send signal to motor driver to turn left

    except IndexError:
        text = "forward"
    cv2.imshow("Contours Window",frame)
    frame = cv2.putText(framebgr, text, coordinates, font, fontScale, color, thickness, cv2.LINE_AA)	
    cv2.imshow("Navigation Window",framebgr)	


if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.init(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        print("Failed to start camera")
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)    
    while True:
        frame = cam.requestFrame(200)
        if frame != None:
            #depth_buf = frame.getDepthData()
            amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)                  
            amplitude_buf*=(255/1024)
            amplitude_buf = np.clip(amplitude_buf, 0, 255)
            amplitude_buf = amplitude_buf.astype(np.uint8)
            avoid_obstacles(amplitude_buf)


            key = cv2.waitKey(1)
            if key == ord("q"):
                exit_ = True
                cam.stop()
                sys.exit(0)
