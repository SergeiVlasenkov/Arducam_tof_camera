import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac
import os
import time
import math
from datetime import datetime

MAX_DISTANCE = 4
file_count = 0
folder_created = False
folderpath = ""
image_saving_started = False # used to record the time when we processed last frame
prev_frame_time = 0
show_fps = False
image_width = float(240)  # b
image_height = float(180)  # a
#fx = 240 / (2 * math.tan(0.5 * math.pi * 64.3 / 180));
#fy = 180 / (2 * math.tan(0.5 * math.pi * 50.4 / 180));


triangle_top_angle = math.radians(90  / 2.0)
print(dir(ac))

def max_pool(img, factor: int):
    """ Perform max pooling with a (factor x factor) kernel"""
    ds_img = np.full((img.shape[0] // factor, img.shape[1] // factor), -float('inf'), dtype=img.dtype)
    np.maximum.at(ds_img, (np.arange(img.shape[0])[:, None] // factor, np.arange(img.shape[1]) // factor), img)
    return ds_img

def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    #result_frame = cv2.medianBlur(result_frame, 7)
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

def saveImages(result_image, amplitude_frame, depth_image255):
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
    fileimagedepth255name = folderpath + filecountstr + "_" + "DepthImage255" + "_" + f'{CDT_string}' + ".png"
    amplitudeImageFileName = folderpath + filecountstr + "_" + "AmplitudeImage" + "_" + f'{CDT_string}' + ".png"    
    cv2.imwrite(fileimagename,result_image)
    cv2.imwrite(amplitudeImageFileName,amplitude_frame) 
    cv2.imwrite(fileimagedepth255name,depth_image255) 

def get_drone_rect_size(median, drone_size):
    # диагонали равны. Половинка диагонали - это гипотенуза для треугольника, у которого одна из сторон будет равно
    # половине ширины или длины основания
    base_diag = 2 * math.sin(triangle_top_angle) * (median / math.cos(triangle_top_angle))
    # тупой угол диагоналей основания
    # β = 2 arctg a/b
    base_diag_obtuse_angle = 2 * math.atan(image_width / image_height)
    base_diag_angle = (math.pi - base_diag_obtuse_angle) / 2

    width = math.cos(base_diag_angle) * base_diag
    # height = math.sin(base_diag_angle)*base_diag

    #240    x
    #1120  180

    drone_rect = math.floor(image_width * drone_size / width)
    return drone_rect

def check_flat(depth_bufer, land_size):
    #result_img = np.zeros(shape=(80, 60))
    result_img = np.zeros(shape=(60, 80))    
    result_img = result_img.astype(np.uint8)
    #multiplier = (land_size**2)
    
    # Iterate over the rows
    for i in range(60):
        # Iterate over the columns
        for j in range(80):
            #check_value_sum = 0
            # img[i, j] = individual pixel value
            # Get the current matrix
            submatrix = depth_bufer[i:i+land_size, j:j+land_size] # reduce fps to 11-12
            #check_value = submatrix[0][0]*multiplier
            #check_value = submatrix
            #check_value[0:len(submatrix),0:len(submatrix[0]) ] = check_value[0,0]
            #check_value_sum = np.sum(submatrix) # very slow. # reduce fps to 1-2
            #check_value_sum = submatrix.sum(axis=1)

            #this cycle slow too # reduce fps to 1-2 <<
            #for g in range( len(submatrix)):
                #for h in range(len(submatrix[g])):
                    #check_value_sum += submatrix[g][h]
            #this cycle slow too >> 
            #if check_value == check_value_sum:            
            if compare_matrix(submatrix[0][0], submatrix):   #using this function i'm getting 6 fps
                result_img[i:i+land_size, j:j+land_size] = 255
    
    return result_img

def compare_matrix(val, matrix):    
    #using this function i'm getting 6 fps
    for g in range( len(matrix)):
        for h in range(len(matrix[g])):    
            if (matrix[g][h] != val) :
                return(False)
    return(True)
                
    


def process_depth_image255(depth_buff_multiply):
    depth_buff_multiply = np.nan_to_num(depth_buff_multiply, False, 2.5)
    depth_buff_multiply = depth_buff_multiply*100                      
    depth_buff_multiply[depth_buff_multiply==0]=255
    depth_buff_multiply[depth_buff_multiply>255]=255
    #depth_buff_multiply = max_pool(depth_buff_multiply, 3)            
    depth_buff_multiply = depth_buff_multiply.astype(np.uint8)
    #depth_buff_multiply = cv2.medianBlur(depth_buff_multiply, 7)  
    #depth_buff_multiply = cv2.resize(depth_buff_multiply, (640,480))   
    cv2.imshow("depth_buff_multiply",depth_buff_multiply)
    return depth_buff_multiply           

if __name__ == "__main__":
    
    cam = ac.ArducamCamera()
    if cam.init(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        print("Failed to start camera")
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)
    #cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    #cv2.setMouseCallback("preview",on_mouse)
        
    video_buffer_started = False    
    while True:
        frame = cam.requestFrame(200)
        if frame != None:
            depth_buf = frame.getDepthData()
                                    
            amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)

            depth_buff_multiply = depth_buf
            #img_to_check = process_depth_image255(depth_buff_multiply)
            depth_image255 = process_depth_image255(depth_buff_multiply)
            #checked_img = check_flat(img_to_check, 3)
            #checked_img = cv2.medianBlur(checked_img, 3)  
            #cv2.imshow("checked_img",checked_img)            
                        
            #diagMedian=[]
            #diagMedian.append(np.diagonal(depth_buff_multiply))
            #diagMedian.append(np.fliplr(depth_buff_multiply).diagonal(1)) 
            #diagMedian = int(np.average(diagMedian))

            amplitude_buf*=(255/1024)
            amplitude_buf = np.clip(amplitude_buf, 0, 255)
            
            amplitude_frame = amplitude_buf.astype(np.uint8)
            

            result_image = process_frame(depth_buf,amplitude_buf)    
           
            #drone_rect = get_drone_rect_size(diagMedian, 180)
            #print(diagMedian)
            #cannyImg = cv2.Canny(result_image, 10, 50)
            dat = result_image
            #result_image = cv2.GaussianBlur(result_image, (5,5), 0)
            #result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
            #cv2.rectangle(cannyImg, (0, 0), (drone_rect +drone_rect, drone_rect +drone_rect), (255, 255, 255))
            #cv2.rectangle(result_image,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1)
            #cv2.rectangle(result_image,(followRect.start_x,followRect.start_y),(followRect.end_x,followRect.end_y),(255,255,255), 1)
            #print("select Rect distance:",np.mean(depth_buf[selectRect.start_x:selectRect.end_x,selectRect.start_y:selectRect.end_y]))
            #cv2.rectangle(result_image, (0, 0), (drone_rect +drone_rect, drone_rect +drone_rect), (255, 255, 255))
            #cv2.rectangle(amplitude_frame, (0, 0), (drone_rect +drone_rect, drone_rect +drone_rect), (255, 255, 255))
            

            if image_saving_started:
                saveImages(result_image, amplitude_frame, depth_image255)          
            
            # putting the FPS count on the frame
            if show_fps:
                cv2.putText(amplitude_frame, calcFpsString(), (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

            cv2.imshow("preview_amplitude",amplitude_frame)
            #cv2.imshow("preview",result_image)
            #cv2.imshow("canny", cannyImg)
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
