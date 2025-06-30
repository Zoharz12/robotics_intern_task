import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as QOS       # uniform “best-effort”
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from ultralytics import YOLO
import sensor_msgs_py.point_cloud2 as pc2

class FusionMapper(Node):
  def __init__(self):
    super().__init__('fusion_mapper'); self.model, self.bridge = YOLO('yolov8n-seg.pt'), CvBridge()
    self.pc = self.rgb = self.depth = None
    self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_cb, QOS)
    self.create_subscription(Image, '/camera/color/image_raw',  self.rgb_cb,  QOS)
    self.create_subscription(Image, '/camera/depth/image_raw',  self.depth_cb, QOS)
    self.pub = self.create_publisher(OccupancyGrid, 'world_map', 1)

  def lidar_cb(self, msg):                                    # LiDAR, RGB, Depth callbacks functions
    pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
    self.pc = np.array(pts) if pts else np.empty((0,3))

  def rgb_cb(self,   msg): self.rgb  = self.bridge.imgmsg_to_cv2(msg, 'bgr8')   # convert RGB image to OpenCV format
  def depth_cb(self, msg):self.depth = self.bridge.imgmsg_to_cv2(msg); self.process() \
                            if self.pc is not None and self.rgb is not None else None

  def process(self):
    img = cv2.resize(self.rgb, (640,360))                     # resize image for improve FPS
    res = self.model(img, verbose=False)[0]
    grid = np.full((200,200), -1, np.int8)
    thr = 3000  # mm
    
    if self.depth is not None:                                # depth fusion → occupied/free
      d = cv2.resize(self.depth, (200,200), cv2.INTER_NEAREST)
      grid[d <  thr]  = 100; grid[d >= thr] = 0

    if len(self.pc):                                          # LiDAR (ground filtered)
      gx = ((self.pc[:,0]*10)+100).astype(int)
      gy = ((self.pc[:,1]*10)+100).astype(int)
      mask = (gx>=0)&(gx<200)&(gy>=0)&(gy<200)&(self.pc[:,2] > -0.2)
      grid[gy[mask], gx[mask]] = 100

    if res.masks is not None:                                 # segmentation masks → occupied
      for mk in res.masks.data.cpu():
        m = (mk>0.5).numpy().astype(np.uint8).squeeze()
        grid[cv2.resize(m,(200,200),cv2.INTER_NEAREST) == 1] = 100

    msg = OccupancyGrid()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.info.resolution = 0.1
    msg.info.width = msg.info.height = 200
    msg.info.origin = Pose()
    msg.info.origin.position.x = msg.info.origin.position.y = -10.0
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist(); self.pub.publish(msg)

def main(): rclpy.init(); rclpy.spin(FusionMapper()); rclpy.shutdown()
if __name__ == '__main__': main()
