import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac
import os
import time
from datetime import datetime

MAX_DISTANCE = 4
file_count = 0
folder_created = False
folderpath = ""
image_saving_started = False # used to record the time when we processed last frame
prev_frame_time = 0
show_fps = False
print(dir(ac))


def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame 

        
def usage(argv0):
    print("Usage: python "+argv0+" [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")

def calcFpsString():
    global prev_frame_time
    # used to record the time at which we processed current frame    
    new_frame_time = time.time()
    # Calculating the fps  
    # fps will be number of frame processed in given time frame
    # since their will be most of time error of 0.001 second
    # we will be subtracting it to get more accurate result
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time

    # converting the fps into integer
    fps = int(fps)

    # converting the fps to string so that we can display it on frame
    # by using putText function
    return str(fps)
              
def imageSaver():
    global image_saving_started
    global folder_created
    image_saving_started = not (image_saving_started)
    if image_saving_started:
        print("Saving Images Started")
    else:                                        
        folder_created = False
        print("Saving Images Stopped")

def setShowFps():
    global show_fps
    show_fps = not show_fps

def getCdtString():
    # datetime object containing current date and time
    now = datetime.now()
         
    cdt_string = now.strftime("%Y-%m-%d_%H:%M:%S")
    return cdt_string

def saveImages(result_image, amplitude_frame):
    global file_count  
    global folder_created
    global folderpath
    if folder_created == False:
        folderDT = getCdtString()
        folderpath = "./" +  f'{folderDT}'
        os.mkdir(folderpath)
        folder_created = True
        folderpath += "/"
                                                                        
    file_count += 1
    if file_count < 10:
        filecountstr = "000" + f'{file_count}'
    elif (file_count > 9) and (file_count <100):
        filecountstr = "00" + f'{file_count}'
    elif (file_count > 99) and (file_count <1000):
        filecountstr = "0" + f'{file_count}'
    elif (file_count > 999):
        filecountstr =  f'{file_count}'
                                
    CDT_string = getCdtString()
    fileimagename = folderpath + filecountstr + "_" + "DepthImage" + "_" + f'{CDT_string}' + ".png"                
    amplitudeImageFileName = folderpath + filecountstr + "_" + "AmplitudeImage" + "_" + f'{CDT_string}' + ".png"    
    cv2.imwrite(fileimagename,result_image)
    cv2.imwrite(amplitudeImageFileName,amplitude_frame) 

if __name__ == "__main__":
    
    cam = ac.ArducamCamera()
    if cam.init(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        print("Failed to start camera")
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)
    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    #cv2.setMouseCallback("preview",on_mouse)
        
    video_buffer_started = False    
    while True:
        frame = cam.requestFrame(200)
        if frame != None:
            depth_buf = frame.getDepthData()
            amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)
            amplitude_buf*=(255/1024)
            amplitude_buf = np.clip(amplitude_buf, 0, 255)
            
            amplitude_frame = amplitude_buf.astype(np.uint8)
            

            result_image = process_frame(depth_buf,amplitude_buf)
            dat = result_image
            #result_image = cv2.GaussianBlur(result_image, (5,5), 0)
            result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
            #cv2.rectangle(result_image,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1)
            #cv2.rectangle(result_image,(followRect.start_x,followRect.start_y),(followRect.end_x,followRect.end_y),(255,255,255), 1)
            #print("select Rect distance:",np.mean(depth_buf[selectRect.start_x:selectRect.end_x,selectRect.start_y:selectRect.end_y]))
            
            if image_saving_started:
                saveImages(result_image, amplitude_frame)          
            
            # putting the FPS count on the frame
            if show_fps:
                cv2.putText(amplitude_frame, calcFpsString(), (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

            cv2.imshow("preview_amplitude",amplitude_frame)
            cv2.imshow("preview",result_image)
            key = cv2.waitKey(1)
            if key == ord("w"):                                
                saveImages(result_image, amplitude_frame)                  
                print("Files saved")
            if key == ord("s"):
                imageSaver()
            if key == ord("f"):
                setShowFps()
            if key == ord("q"):
                exit_ = True
                cam.stop()
                sys.exit(0)
