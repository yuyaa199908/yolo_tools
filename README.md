# note
- 遅延がすごいのでpclで実装するかrealsense-ros内で実装すべき
- setup.py だと --symlink-install がきかないらしいので config と launch を変えたらビルドし直すこと
# install
~~~
cd ~/ros2_ws/src
git clone https://github.com/yuyaa199908/yolo_tools
wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt
cd ~/ros2_ws
pip3 install ultralytics open3d pypcd4 opencv-contrib-python
colcon build --symlink-install --base-path ~/ros2_ws/src/yolo_tools/
~~~
# rgbd2cloud
~~~
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true
~~~
~~~
ros2 launch yolo_tools rgbd2cloud.launch.py
~~~
## config
### yolo.classes
- https://qiita.com/Nya-ku/items/ad07872260cfe8abd322
