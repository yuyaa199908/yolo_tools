# note
<<<<<<< HEAD
- 遅延がすごいのでpclで実装するかrealsense-ros内で実装すべき
- setup.py だと --symlink-install がきかないらしいので config と launch を変えたらビルドし直すこと
=======
遅延がすごいのでpclで実装するかrealsense-ros内で実装すべき
>>>>>>> 50d57a5ed7fa079ccbcfd3e8cf3b26ebe36d7f97
# install
~~~
cd ~/ros2_ws/src
git clone https://github.com/yuyaa199908/yolo_tools
wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt
cd ~/ros2_ws
pip3 install ultralytics open3d pypcd4 opencv-contrib-python
colcon build --symlink-install --base-path ~/ros2_ws/src/yolo_tools/
~~~
<<<<<<< HEAD
'--symlink-install' で'~/ros2_ws/install/yolo_tools/share/yolo_tools/'にシンボリックリンクが生成されない場合は'python3-colcon-core'をインストール
~~~
sudo apt install python3-colcon-core
~~~

=======
>>>>>>> 50d57a5ed7fa079ccbcfd3e8cf3b26ebe36d7f97
# rgbd2cloud
~~~
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true
~~~
~~~
ros2 launch yolo_tools rgbd2cloud.launch.py
~~~
## config
<<<<<<< HEAD
### yolo.classes
- https://qiita.com/Nya-ku/items/ad07872260cfe8abd322

# rgbd2mesh
~~~
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true
~~~
~~~
ros2 launch yolo_tools rgbd2mesh.launch.py
~~~
## config
=======
>>>>>>> 50d57a5ed7fa079ccbcfd3e8cf3b26ebe36d7f97
