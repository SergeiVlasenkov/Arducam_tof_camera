import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ArducamDepthCamera as ac
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import struct
import math
from std_msgs.msg import Header
import numpy as np
#from cv_bridge import CvBridge
import sensor_msgs.msg
from sensor_msgs.msg import Image
import cv2
#msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        if image_publisher_enabled:
            self.imagepublisher = self.create_publisher(Image, 'Image', 10)
        if depthimage_publisher_enabled:
            self.depthimagepublisher = self.create_publisher(Image, 'Depth_image', 10)
        if pointcloud_publisher_enabled:
            self.pointcloudpublisher = self.create_publisher(PointCloud2, 'Point_cloud2', 10)
        self.get_logger().info('Publishers created')
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def timer_callback(self):

        depth_buf, amplitude_buf = get_depth_amplitude_buf()
        depth_ptr = depth_buf
        amplitude_ptr =  amplitude_buf
        if pointcloud_publisher_enabled:
            self.publishpointcloud(depth_ptr,amplitude_ptr)
        if image_publisher_enabled:
            self.publishimage(amplitude_buf)
        if depthimage_publisher_enabled:
            self.publisdepthhimage(depth_buf,amplitude_buf)


    def publishpointcloud(self,depth_ptr,amplitude_ptr):
        pointcloudheader = Header()
        pointcloudheader.frame_id = "sensor_frame"
        point_cloud = get_points_depth_amplitude_buf(depth_ptr,amplitude_ptr)
        pointcloudmsg = create_cloud_xyz(pointcloudheader, point_cloud)
        pointcloudmsg.header.stamp = self.get_clock().now().to_msg() 
        self.pointcloudpublisher.publish(pointcloudmsg)
        #self.get_logger().info('Publishing: "%s"' % pointcloudmsg)
        
    def publisdepthhimage(self, depth_buf, amplitude_buf):
        image = process_depth_frame(depth_buf,amplitude_buf)
        Br = CvBridge()
        imgmsg  = Br.cv2_to_imgmsg(image)
        imgmsg.header.stamp = self.get_clock().now().to_msg()  
        imgmsg.header.frame_id = "sensor_frame"  
        self.depthimagepublisher.publish(imgmsg)
        #self.get_logger().info('Publishing: "%s"' % imgmsg)

    def publishimage(self, amplitude_buf):
        image = get_image_from_amplitude_buf(amplitude_buf)
        Br = CvBridge()
        imgmsg  = Br.cv2_to_imgmsg(image)
        imgmsg.header.stamp = self.get_clock().now().to_msg()  
        imgmsg.header.frame_id = "sensor_frame"  
        self.imagepublisher.publish(imgmsg)
        #self.get_logger().info('Publishing: "%s"' % imgmsg)


class CvBridgeError(TypeError):
    """This is the error raised by :class:`cv_bridge.CvBridge` methods when they fail."""

    pass


class CvBridge(object):
    """
    The CvBridge is an object that converts between OpenCV Images and ROS Image messages.
       .. doctest::
           :options: -ELLIPSIS, +NORMALIZE_WHITESPACE
           >>> import cv2
           >>> import numpy as np
           >>> from cv_bridge import CvBridge
           >>> br = CvBridge()
           >>> dtype, n_channels = br.encoding_as_cvtype2('8UC3')
           >>> im = np.ndarray(shape=(480, 640, n_channels), dtype=dtype)
           >>> msg = br.cv2_to_imgmsg(im)  # Convert the image to a message
           >>> im2 = br.imgmsg_to_cv2(msg) # Convert the message to a new image
           >>> # Convert the image to a compress message
           >>> cmprsmsg = br.cv2_to_compressed_imgmsg(im)
           >>> # Convert the compress message to a new image
           >>> im22 = br.compressed_imgmsg_to_cv2(msg)
           >>> cv2.imwrite("this_was_a_message_briefly.png", im2)
    """

    def __init__(self):
        self.cvtype_to_name = {}
        self.cvdepth_to_numpy_depth = {cv2.CV_8U: 'uint8', cv2.CV_8S: 'int8',
                                       cv2.CV_16U: 'uint16', cv2.CV_16S: 'int16',
                                       cv2.CV_32S: 'int32', cv2.CV_32F: 'float32',
                                       cv2.CV_64F: 'float64'}

        for t in ['8U', '8S', '16U', '16S', '32S', '32F', '64F']:
            for c in [1, 2, 3, 4]:
                nm = '%sC%d' % (t, c)
                self.cvtype_to_name[getattr(cv2, 'CV_%s' % nm)] = nm

        self.numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                                     'int16': '16S', 'int32': '32S', 'float32': '32F',
                                     'float64': '64F'}
        self.numpy_type_to_cvtype.update(dict((v, k) for (k, v) in self.numpy_type_to_cvtype.items()))

    def dtype_with_channels_to_cvtype2(self, dtype, n_channels):
        return '%sC%d' % (self.numpy_type_to_cvtype[dtype.name], n_channels)


    def encoding_to_dtype_with_channels(self, encoding):
        return self.cvtype2_to_dtype_with_channels(self.encoding_to_cvtype2(encoding))
       

    def cv2_to_compressed_imgmsg(self, cvim, dst_format='jpg'):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::CompressedImage message.
        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param dst_format:  The format of the image data, one of the following strings:
        http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html
        http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat
        * imread(const string& filename, int flags)
           * bmp, dib
           * jpeg, jpg, jpe
           * jp2
           * png
           * pbm, pgm, ppm
           * sr, ras
           * tiff, tif
        :rtype:           A sensor_msgs.msg.CompressedImage message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``format``
        This function returns a sensor_msgs::Image message on success,
        or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        cmprs_img_msg = sensor_msgs.msg.CompressedImage()
        cmprs_img_msg.format = dst_format
        ext_format = '.' + dst_format
        try:            
            cmprs_img_msg.data.frombytes(np.array(cv2.imencode(ext_format, cvim)[1]).tobytes())
        except RuntimeError as e:
            raise CvBridgeError(e)

        return cmprs_img_msg

    def cv2_to_imgmsg(self, cvim, encoding='passthrough', header = None):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::Image message.
        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param encoding:  The encoding of the image data, one of the following strings:
           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h
        :param header:    A std_msgs.msg.Header message
        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``
        If encoding is ``"passthrough"``, then the message has the same encoding as the image's
        OpenCV type. Otherwise desired_encoding must be one of the standard image encodings
        This function returns a sensor_msgs::Image message on success,
        or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        img_msg = sensor_msgs.msg.Image()
        #cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
        #cv2.imshow("preview", cvim)
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        if header is not None:
            img_msg.header = header
        if len(cvim.shape) < 3:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
        else:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, cvim.shape[2])
        if encoding == 'passthrough':
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding
            # Verify that the supplied encoding is compatible with the type of the OpenCV image
            if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
                raise CvBridgeError('encoding specified as %s, but image has incompatible type %s'
                                    % (encoding, cv_type))
        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data.frombytes(cvim.tobytes())
        img_msg.step = len(img_msg.data) // img_msg.height

        return img_msg

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()    
    

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

def get_depth_amplitude_buf():
    frame = cam.requestFrame(200)
    if frame != None:
        depth_buf = frame.getDepthData()
        amplitude_buf = frame.getAmplitudeData() 
        cam.releaseFrame(frame)
        return depth_buf, amplitude_buf
        


def get_points_depth_amplitude_buf(depth_ptr, amplitude_ptr):
    points = []
    for row_idx in range(179):   # Y Axis             
        for col_idx in range(239): # X Axis
            if (float(amplitude_ptr[row_idx,col_idx]) > 30):    # remove ring from camera lens
                zz = float(depth_ptr[row_idx,col_idx])

                xx = (((120 - col_idx+1)) / fx) * zz                    
                
                yy = ((90 - row_idx+1) / fy) * zz

                pt = [xx, yy, zz]

                points.append(pt)
            else:
                pt = [0, 0, 0]
                points.append(pt)
    return points
    
def get_image_from_amplitude_buf(amplitude_buf):                     
    amplitude_buf*=(255/1024)
    amplitude_buf = np.clip(amplitude_buf, 0, 255)
    resultimg = amplitude_buf.astype(np.uint8) 
    return resultimg
            

#global init variables :
MAX_DISTANCE = 4
fx = 240 / (2 * math.tan(0.5 * math.pi * 64.3 / 180));
fy = 180 / (2 * math.tan(0.5 * math.pi * 50.4 / 180));
a = 255
#global init camera
cam = ac.ArducamCamera()
if cam.init(ac.TOFConnect.CSI,0) != 0 :
    print("initialization failed")
if cam.start(ac.TOFOutput.DEPTH) != 0 :
    print("Failed to start camera")
cam.setControl(ac.TOFControl.RANG, MAX_DISTANCE)
pointcloud_publisher_enabled = False
image_publisher_enabled = True
depthimage_publisher_enabled = True

# function creates pointCloud2 from XYZ point array 
def create_cloud_xyz( header, points):

    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)

    ]
    return pc2.create_cloud(header,fields,points)

def process_depth_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    result_frame = cv2.applyColorMap(result_frame, cv2.COLORMAP_JET)
    return result_frame 

if __name__ == '__main__':
    main()

