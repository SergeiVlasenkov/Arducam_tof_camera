import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac
import uuid
import os

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

        
def usage(argv0):
    print("Usage: python "+argv0+" [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")


if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.init(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        print("Failed to start camera")
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)
    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    #cv2.setMouseCallback("preview",on_mouse)
    filecount =0
    foldercreated = False
    while True:
        frame = cam.requestFrame(200)
        if frame != None:
            depth_buf = frame.getDepthData()
            amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)
            amplitude_buf*=(255/1024)
            amplitude_buf = np.clip(amplitude_buf, 0, 255)
            
            amplitude_frame = amplitude_buf.astype(np.uint8)
            cv2.imshow("preview_amplitude",amplitude_frame)

            result_image = process_frame(depth_buf,amplitude_buf)
            dat = result_image
            #result_image = cv2.GaussianBlur(result_image, (5,5), 0)
            result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
            #cv2.rectangle(result_image,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1)
            #cv2.rectangle(result_image,(followRect.start_x,followRect.start_y),(followRect.end_x,followRect.end_y),(255,255,255), 1)
            #print("select Rect distance:",np.mean(depth_buf[selectRect.start_x:selectRect.end_x,selectRect.start_y:selectRect.end_y]))
            cv2.imshow("preview",result_image)

            key = cv2.waitKey(1)
            if key == ord("w"):                                
                if foldercreated == False:
                    folderguid = uuid.uuid4() 
                    folderpath = "./" +  f'{folderguid}'
                    os.mkdir(folderpath)
                    foldercreated = True
                    folderpath += "/"
                                                        
                
                filecount += 1
                if filecount < 10:
                    filecountstr = "000" + f'{filecount}'
                elif (filecount > 9) and (filecount <100):
                    filecountstr = "00" + f'{filecount}'
                elif (filecount > 99) and (filecount <1000):
                    filecountstr = "0" + f'{filecount}'
                elif (filecount > 999):
                    filecountstr =  f'{filecount}'
                                
                guid = uuid.uuid4() 
                fileimagename = folderpath + filecountstr + "_" + "DepthImage" + "_" + f'{guid}' + ".png"
                filename = folderpath + filecountstr + "_" + "DepthImageData" + "_" + f'{guid}' + ".txt"
                depthfilename = folderpath + filecountstr + "_" + "DepthData" + "_"  + f'{guid}' + ".txt"
                amplitudeImageFileName = folderpath + filecountstr + "_" + "AmplitudeImage" + "_" + f'{guid}' + ".png"    
                cv2.imwrite(fileimagename,result_image)
                cv2.imwrite(amplitudeImageFileName,amplitude_frame)
                with open(filename, 'w') as f:
                    f.write(', '.join(str(item) for item in dat)+'\n')
                with open(depthfilename, 'w') as d:
                    d.write(', '.join(str(item) for item in depth_buf)+'\n')
                   
                print("Files saved")
            if key == ord("q"):
                exit_ = True
                cam.stop()
                sys.exit(0)
