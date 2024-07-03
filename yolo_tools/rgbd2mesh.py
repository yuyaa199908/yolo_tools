import rclpy
from rclpy.node import Node
import logging
# msgs
from realsense2_camera_msgs.msg import RGBD
# from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point,Pose, Vector3
from shape_msgs.msg import Mesh, MeshTriangle
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
# from mesh_msgs.msg import MeshGeometryStamped, MeshGeometry, MeshVertexColorsStamped, MeshVertexColors, MeshTriangleIndices

# add
from ultralytics import YOLO
import numpy as np
from cv_bridge import CvBridge
import cv2
import open3d as o3d
# from ctypes import * # convert float to uint32
from pypcd4 import PointCloud
from builtin_interfaces.msg import Time
import struct

class RGBD2MESH(Node):
    def __init__(self):
        super().__init__('RGBD2MESH')
        self.init_param()

        # qos_policy = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_image = self.create_subscription(
                            RGBD(), 
                            '/input_rgbd', 
                            self.CB_main,
                            10)
        
        self.pub_mesh =  self.create_publisher(Marker, '/output_mesh', 10)
        self.count_sub_rgbd = 0

        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=4.0 / 512.0,
            sdf_trunc=0.04,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.mesh = o3d.geometry.TriangleMesh()
        self.geom_added = False

    def CB_main(self, msg):
        input_rgb = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
        input_d = CvBridge().imgmsg_to_cv2(msg.depth, "passthrough") #.astype(np.int32)
        input_rgb = cv2.resize(input_rgb, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)
        input_d = cv2.resize(input_d, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)
        input_d[input_d >= 0.5 * 1000] = 0

        color = o3d.geometry.Image(input_rgb)
        depth = o3d.geometry.Image(input_d)#.astype(np.uint16))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
        
        fx_d = msg.rgb_camera_info.k[0] * self.fxy
        fy_d = msg.rgb_camera_info.k[4] * self.fxy
        cx_d = msg.rgb_camera_info.k[2] * self.fxy
        cy_d = msg.rgb_camera_info.k[5] * self.fxy
        height, width = input_d.shape
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width, height, fx_d,fy_d, cx_d, cy_d
        )

        self.volume.integrate(
            rgbd,
            pinhole_camera_intrinsic,
            np.array(([1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0])) #?
        )

        # https://www.open3d.org/docs/release/python_api/open3d.geometry.TriangleMesh.html
        mesh = self.volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        # o3d.visualization.draw_geometries([mesh])
        self.mesh = mesh
        if self.geom_added == False:
            self.vis.add_geometry(self.mesh)
            geom_added = True
        self.vis.update_geometry(self.mesh)
        self.vis.poll_events()
        self.vis.update_renderer()

        msg_marker = Marker()
        msg_marker.header.frame_id = self.frame_id
        now = self.get_clock().now()
        msg_marker.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        msg_marker.ns = "hoge"
        msg_marker.type = 11
        msg_marker.action = 0 #1
        msg_marker.pose = Pose()
        msg_marker.scale = Vector3(x=1.0 ,y=1.0,z=1.0)
        #[a b c] -> [c -a -b]
        for t in mesh.triangles:
            for i in t:
                msg_marker.points.append(Point(x=mesh.vertices[i][2], y= mesh.vertices[i][0]*-1, z=mesh.vertices[i][1]*-1))
        msg_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.pub_mesh.publish(msg_marker)

        # for idx, t in  enumerate(mesh.triangles):
        #     msg_marker = Marker()
        #     msg_marker.header.frame_id = self.frame_id
        #     now = self.get_clock().now()
        #     msg_marker.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        #     msg_marker.ns = "hoge"
        #     msg_marker.id = idx
        #     msg_marker.type = 11
        #     msg_marker.action = 0 #1
        #     msg_marker.pose = Pose()
        #     msg_marker.scale = Vector3(x=1.0 ,y=1.0,z=1.0)
        #     c = np.array([0,0,0])
        #     for i in t:
        #         msg_marker.points.append(Point(x=mesh.vertices[i][2], y= mesh.vertices[i][0]*-1, z=mesh.vertices[i][1]*-1))
        #         c = c + mesh.vertex_colors[i]
        #     #bgr->rgb
        #     msg_marker.color = ColorRGBA(r=c[2]/3, g=c[1]/3, b=c[0]/3, a=1.0)
        #     self.pub_mesh.publish(msg_marker)
        
        # self.volume.reset()

    def init_param(self):
        self.declare_parameter('image_scale', 1.0)
        self.declare_parameter('do_publish_image', False)
        self.declare_parameter('frame_id', "hoge")
        self.declare_parameter('yolo.model_path', "")
        self.declare_parameter('yolo.classes', [0])
        self.declare_parameter('pub_interval', 0.5)

        self.fxy = self.get_parameter("image_scale").get_parameter_value().double_value
        self.do_publish_image = self.get_parameter('do_publish_image').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        yolo_path = self.get_parameter('yolo.model_path').get_parameter_value().string_value
        self.yolo_classes = self.get_parameter('yolo.classes').get_parameter_value().integer_array_value
        # self.model = YOLO(yolo_path)
        self.pub_interval = self.get_parameter("pub_interval").get_parameter_value().double_value



def main():
    rclpy.init()
    node = RGBD2MESH()
    rclpy.spin(node)
    rclpy.shutdown()
    node.vis.destroy_window()
    del node.vis

if __name__ == '__main__':
    main()