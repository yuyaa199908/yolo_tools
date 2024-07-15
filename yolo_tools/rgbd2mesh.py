import rclpy
from rclpy.node import Node
import logging
# msgs
from realsense2_camera_msgs.msg import RGBD
# from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point,Pose, Vector3
from shape_msgs.msg import Mesh, MeshTriangle
from sensor_msgs.msg import PointCloud2, PointField, Image
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
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation

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
        self.pub_cloud =  self.create_publisher(PointCloud2, '/output_cloud', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.TSDF_voxel_length,
            sdf_trunc=self.TSDF_sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )

        if self.is_display_open3d:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.mesh = o3d.geometry.TriangleMesh()
            self.geom_added = False

    def CB_main(self, msg):
        rgbd = self.convert_msg_to_rgbd(msg)
        height, width = np.asarray(rgbd.depth).shape
        pinhole_camera_intrinsic = self.convert_msg_to_camera_intrinsic(msg,height,width)

        camera_pose = self.convert_tf_to_camera_pose()
        self.volume.integrate(
            rgbd,
            pinhole_camera_intrinsic,
            camera_pose
            ) 

        # https://www.open3d.org/docs/release/python_api/open3d.geometry.TriangleMesh.html
        """
        open3d.geometry.TriangleMesh
        頂点: p1, p2, ..., pn
        頂点インデックス： 1,2, ..., n
        mesh.vertices = [[p1_x, p1_y, p1_z], ... ,[pn_x, pn_y, pn_z]]
        mesh.vertices = [[p1_r, p1_g, p1_b], ... ,[pn_r, pn_g, pn_b]]

        三角形: t1, t2,...,tm 
        mesh.triangles = [[t1_1, t1_2, t1_3],...,[tm_1, tm_2, tm_3]]

        ex. t = [5,1,2]
        p5 *----* p1
             \ |
              \|
               * p2
        """

        if self.is_publish_cloud:
            pcd = self.volume.extract_voxel_point_cloud()
            msg_cloud = self.create_msg_cloud(pcd)
            self.pub_cloud.publish(msg_cloud)

        if self.is_publish_mesh:
            mesh = self.volume.extract_triangle_mesh()
            # mesh.compute_vertex_normals()   #?

            #TODO: 表示オブジェクトの更新がうまく行ってない？
            if self.is_display_open3d:
                self.mesh = mesh
                if self.geom_added == False:
                    self.vis.add_geometry(self.mesh)
                    self.geom_added = True
                self.vis.update_geometry(self.mesh)
                self.vis.poll_events()
                self.vis.update_renderer()

            msg_marker = self.create_msg_mesh(mesh)
            self.pub_mesh.publish(msg_marker)

    def create_msg_mesh(self, mesh):
        msg = Marker()
        msg.header.frame_id = self.frame_id_ground
        now = self.get_clock().now()
        msg.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        msg.ns = self.marker_ns
        msg.type = 11
        msg.action = 0 #1
        msg.pose = Pose()
        msg.scale = Vector3(x=1.0 ,y=1.0,z=1.0)
        
        # TODO: 座標変換
        #[a b c] -> [c -a -b]
        for t in mesh.triangles:
            for i in t:
                # msg.points.append(Point(x=mesh.vertices[i][2], y= mesh.vertices[i][0]*-1, z=mesh.vertices[i][1]*-1))
                msg.points.append(Point(x=mesh.vertices[i][0], y= mesh.vertices[i][1], z=mesh.vertices[i][2]))
        msg.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        return msg
        
    def create_msg_cloud(self,pcd):
        header = Header()
        now = self.get_clock().now()
        header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        header.frame_id = self.frame_id_ground
        rgb_float_array = self.convert_rgb_array_to_float(pcd.colors)
        arr = np.concatenate([np.asarray(pcd.points) ,rgb_float_array.reshape((-1,1))],1)
        pc = PointCloud.from_xyzrgb_points(arr) #PointCloud.from_points(arr, fields, types)
        msg = pc.to_msg(header)
        return msg

    def convert_msg_to_rgbd(self,msg):
        input_rgb = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
        input_d = CvBridge().imgmsg_to_cv2(msg.depth, "passthrough") #.astype(np.int32)
        input_rgb = cv2.resize(input_rgb, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)
        input_d = cv2.resize(input_d, dsize=None, fx=self.fxy, fy=self.fxy ,interpolation=cv2.INTER_NEAREST)
        color = o3d.geometry.Image(input_rgb)
        depth = o3d.geometry.Image(input_d)#.astype(np.uint16))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=self.depth_range_max, convert_rgb_to_intensity=False)
        return rgbd

    def convert_msg_to_camera_intrinsic(self, msg,h,w):
        fx_d = msg.rgb_camera_info.k[0] * self.fxy
        fy_d = msg.rgb_camera_info.k[4] * self.fxy
        cx_d = msg.rgb_camera_info.k[2] * self.fxy
        cy_d = msg.rgb_camera_info.k[5] * self.fxy
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            w, h, fx_d,fy_d, cx_d, cy_d
        )
        return pinhole_camera_intrinsic

    def convert_tf_to_camera_pose(self):
        camera_pose = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])
        
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id_ground, self.frame_id_depth, rclpy.time.Time())
            self.get_logger().info(f"Translation: {trans.transform.translation}")
            self.get_logger().info(f"Rotation: {trans.transform.rotation}")
            tx,ty,tz = trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z
            rx,ry,rz,rw = trans.transform.rotation.x,trans.transform.rotation.y ,trans.transform.rotation.z,trans.transform.rotation.w
            rot = Rotation.from_quat(np.array([rx,ry,rz,rw]))
            camera_pose[:3,:3] = rot.as_dcm()
            camera_pose[:3,3] = np.array([tx,ty,tz])
        except:
            pass
        return camera_pose

    def convert_rgb_array_to_float(self, rgb_array):
        """
        Convert an array of RGB values (0.0 to 1.0) to an array of single float32 RGB values.
        """
        rgb_float_array = np.apply_along_axis(lambda x: self.rgb_to_float(x[2], x[1], x[0]), 1, rgb_array)
        return rgb_float_array

    def rgb_to_float(self, r, g, b):
        """
        Convert separate R, G, B values (0.0 to 1.0) to a single float32 RGB value.
        """
        # Ensure the RGB values are in the range 0 to 255
        r = int(r * 255.0)
        g = int(g * 255.0)
        b = int(b * 255.0)
        
        # Combine the RGB values into a single 32-bit integer
        rgb_int = (r << 16) | (g << 8) | b
        
        # Pack this integer into a float32
        rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
        return rgb_float

    def init_param(self):
        self.declare_parameter('image_scale', 1.0)
        self.declare_parameter('frame_id_ground', "map")
        self.declare_parameter('frame_id_depth', "camera_depth_optical_frame")
        self.declare_parameter('depth_range_max', 4.0)
        self.declare_parameter('is_display_open3d', True)
        self.declare_parameter('marker_ns', "marker")
        self.declare_parameter('TSDF.voxel_length',4.0 / 512.0)
        self.declare_parameter('TSDF.sdf_trunc',0.04)
        
        self.fxy = self.get_parameter("image_scale").get_parameter_value().double_value
        self.frame_id_ground = self.get_parameter('frame_id_ground').get_parameter_value().string_value
        self.frame_id_depth = self.get_parameter('frame_id_depth').get_parameter_value().string_value
        self.depth_range_max = self.get_parameter('depth_range_max').get_parameter_value().double_value
        self.is_display_open3d = self.get_parameter('is_display_open3d').get_parameter_value().bool_value
        self.marker_ns = self.get_parameter('marker_ns').get_parameter_value().string_value
        self.TSDF_voxel_length = self.get_parameter('TSDF.voxel_length').get_parameter_value().double_value
        self.TSDF_sdf_trunc = self.get_parameter('TSDF.sdf_trunc').get_parameter_value().double_value

        self.is_publish_mesh = True
        self.is_publish_cloud = False

def main():
    rclpy.init()
    node = RGBD2MESH()
    rclpy.spin(node)
    rclpy.shutdown()
    node.vis.destroy_window()
    del node.vis

if __name__ == '__main__':
    main()