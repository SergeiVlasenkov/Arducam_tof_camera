import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ArducamDepthCamera as ac
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import struct
import math
from std_msgs.msg import Header

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud2', 10)
        self.get_logger().info('Publisher created')
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def timer_callback(self):
        header = Header()
        header.frame_id = "sensor_frame"
        point_cloud = get_points_from_frame()        
        msg = create_cloud_xyz(header, point_cloud)
        msg.header.stamp = self.get_clock().now().to_msg()                                 
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.get_logger().info('Publishing: "%s"' % msg)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

def get_points_from_frame():
    frame = cam.requestFrame(200)
    points = []
    if frame != None:
        depth_ptr = frame.getDepthData()
        amplitude_ptr = frame.getAmplitudeData()            
        cam.releaseFrame(frame)
        for row_idx in range(179):                
            for col_idx in range(239):
                if (float(amplitude_ptr[row_idx,col_idx]) > 30):    # remove ring from camera lens
                    zz = float(depth_ptr[row_idx,col_idx])
                    #if zz < 0:
                        #zz = 0
                    xx = (((120 - col_idx+1)) / fx) * zz                    
                    #xx = (((col_idx)) / fx) * zz
                    #if xx < 0:
                        #xx = 0                    
                    yy = ((90 - row_idx+1) / fy) * zz
                    #yy = ((row_idx) / fy) * zz
                    #if yy < 0:
                        #yy=0
                    
                    #r = int(xx/4 * 255.0)
                    #g = int(yy/4 * 255.0)
                    #b = int(zz/4 * 255.0)
                    #rgb = 9436681
                    #rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    #pt = [xx, yy, zz, rgb]
                    pt = [xx, yy, zz]

                    points.append(pt)
                else:
                    pt = [0, 0, 0]
                    points.append(pt)
        return points
   

# init global variables :
MAX_DISTANCE = 4
fx = 240 / (2 * math.tan(0.5 * math.pi * 64.3 / 180));
fy = 180 / (2 * math.tan(0.5 * math.pi * 50.4 / 180));
a = 255    
cam = ac.ArducamCamera()
if cam.init(ac.TOFConnect.CSI,0) != 0 :
    print("initialization failed")
if cam.start(ac.TOFOutput.DEPTH) != 0 :
    print("Failed to start camera")
cam.setControl(ac.TOFControl.RANG, MAX_DISTANCE)

def create_cloud_xyz( header, points):

    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                #PointField('intensity', 12, PointField.FLOAT32, 1),
                #PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)
    ]
    return pc2.create_cloud(header,fields,points)




if __name__ == '__main__':
    main()
