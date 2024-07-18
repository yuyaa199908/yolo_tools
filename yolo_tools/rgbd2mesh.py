import rclpy
from rclpy.node import Node
import logging
# msgs
from realsense2_camera_msgs.msg import RGBD
# from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Point,Pose, Vector3, PoseStamped
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
from collections import deque

class RGBD2MESH(Node):
    def __init__(self):
        super().__init__('RGBD2MESH')
        self.init_param()

        # qos_policy = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_image = self.create_subscription(RGBD(), '/input_rgbd', self.CB_main,10)
        self.sub_flag_move = self.create_subscription(
                            Bool(),
                            '/servo_state_move',
                            self.CB_update_state_move,
                            10)
        self.sub_flag_create_mesh = self.create_subscription(
                            Bool(),
                            '/op_create_mesh',
                            self.CB_update_op1,
                            10)
        self.sub_flag_save_mesh = self.create_subscription(
                            Bool(),
                            '/op_save_mesh',
                            self.CB_update_op2,
                            10)
        

        self.pub_mesh =  self.create_publisher(Marker, '/output_mesh', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.TSDF_voxel_length,
            sdf_trunc=self.TSDF_sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )

        self.cnt = 0
        self.is_move = True
        self.is_create_mesh = False
        self.is_save_mesh = False

        if self.is_display_open3d:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.mesh = o3d.geometry.TriangleMesh()
            self.geom_added = False

        self.last_camera_pose = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            self.cam_w, self.cam_h, self.cam_K[0],self.cam_K[4], self.cam_K[2], self.cam_K[5]
        )

    def CB_main(self, msg):
        if self.is_move == False:
            self.get_logger().info(f"\nwait for servo to stop\n")
            if self.is_save_mesh == True:
                mesh = self.create_mesh()

                o3d.io.write_triangle_mesh(self.output_mesh_path, mesh, write_ascii=True)

                self.is_save_mesh = False
                self.get_logger().info(f"\n ---saved --- \n")

        else:
            rgbd = self.convert_msg_to_rgbd(msg)
            # カメラパラメータは固定値を使う
            # height, width = np.asarray(rgbd.depth).shape
            # pinhole_camera_intrinsic = self.convert_msg_to_camera_intrinsic(msg,height,width)

            camera_pose = self.convert_tf_to_camera_pose()
            self.volume.integrate(
                rgbd,
                self.pinhole_camera_intrinsic,
                np.linalg.inv(camera_pose)
                ) 

            if self.is_publish_mesh:
                mesh = self.create_mesh()
            
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
                # self.volume.reset()

    def create_msg_mesh(self, mesh):
        msg = Marker()
        msg.header.frame_id =  self.frame_id_ground
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
            tx,ty,tz = trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z
            rx,ry,rz,rw = trans.transform.rotation.x,trans.transform.rotation.y ,trans.transform.rotation.z,trans.transform.rotation.w
            rot = Rotation.from_quat(np.array([rx,ry,rz,rw]))
            # self.get_logger().info(f"t: {tx,ty,tz}")
            # self.get_logger().info(f"Rot(euler): {rot.as_euler('xyz', degrees=True)}")
            camera_pose[:3,:3] = rot.as_matrix(),      
            camera_pose[:3,3] = np.array([tx,ty,tz])

            self.last_camera_pose = camera_pose # update camera_pose
            return camera_pose

        except:
            self.get_logger().error(f"Can't convert tf to camera pose !!!!!!!!!")
            return self.last_camera_pose    # tfからcamera_pose を取得できなかったら最新のカメラポーズを使う

    def CB_update_state_move(self, msg):
        self.is_move = msg.data

    def CB_update_op1(self, msg):
        self.is_create_mesh  = msg.data
    
    def CB_update_op2(self, msg):
        self.is_save_mesh  = msg.data

    def init_param(self):
        self.declare_parameter('frame_id_ground', "map")
        self.declare_parameter('frame_id_depth', "camera_color_optical_frame")
        self.declare_parameter('depth_range_max', 4.0)
        self.declare_parameter('is_display_open3d', True)
        self.declare_parameter('is_publish_mesh', True)
        self.declare_parameter('marker_ns', "marker")
        self.declare_parameter('TSDF.voxel_length',4.0 / 512.0)
        self.declare_parameter('TSDF.sdf_trunc',0.04)
        self.declare_parameter('camera.width',640)
        self.declare_parameter('camera.height',260)
        self.declare_parameter('camera.K', 
                               [618.33599854,   0.0,         311.00698853,   
                                0.0,         618.52191162,   239.00808716,
                                0.0,           0.0,           1.0        ])
        self.declare_parameter('output_mesh_path', "./hoge.ply")

        self.frame_id_ground = self.get_parameter('frame_id_ground').get_parameter_value().string_value
        self.frame_id_depth = self.get_parameter('frame_id_depth').get_parameter_value().string_value
        self.depth_range_max = self.get_parameter('depth_range_max').get_parameter_value().double_value
        self.is_display_open3d = self.get_parameter('is_display_open3d').get_parameter_value().bool_value
        self.is_publish_mesh = self.get_parameter('is_publish_mesh').get_parameter_value().bool_value
        self.marker_ns = self.get_parameter('marker_ns').get_parameter_value().string_value
        self.TSDF_voxel_length = self.get_parameter('TSDF.voxel_length').get_parameter_value().double_value
        self.TSDF_sdf_trunc = self.get_parameter('TSDF.sdf_trunc').get_parameter_value().double_value
        self.cam_w = self.get_parameter('camera.width').get_parameter_value().integer_value
        self.cam_h = self.get_parameter('camera.height').get_parameter_value().integer_value
        self.cam_K = self.get_parameter('camera.K').get_parameter_value().double_array_value
        self.output_mesh_path = self.get_parameter('output_mesh_path').get_parameter_value().string_value

    def create_mesh(self):
        """
        TODO:
        - メッシュ作成手法(TSDFでいいのか?)
        - 色の改良(https://www.open3d.org/docs/0.16.0/tutorial/pipelines/color_map_optimization.html)
            - メッシュは座標系さえあってればどんな手法でもあり
            - RGBDのリストと対応するカメラ姿勢を camera_trajectory として保存
            - どの画像を選ぶか？
                - 理想的には全包囲からまんべんなく画像がほしい  
                - カメラ姿勢から選択できるようなアルゴリズムが必要
                - これで来たらガウシアンスプラッティングの見栄えも良くなる
        """
        mesh = self.volume.extract_triangle_mesh()  # シンプルにメッシュ作るだけ
        mesh.compute_vertex_normals()   #?
        return mesh

def main():
    rclpy.init()
    node = RGBD2MESH()
    rclpy.spin(node)
    rclpy.shutdown()
    node.vis.destroy_window()
    del node.vis

if __name__ == '__main__':
    main()