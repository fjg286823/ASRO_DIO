20220122

1. 用DIO-RO离线跑kinect录的bag（凯乐）:
(1)运行脚本的命令：./ok_kaile_of.sh
(2)用的launch: /home/javens/Workspace/catkin_0115/src/ROGIO-ROS-threads/launch/kinect_kaile.launch
(3)用的config: /home/javens/Workspace/catkin_0115/src/ROGIO-ROS-threads/config/Kinect_kaile_config.yaml
(4)用的rviz文件:
(5)Topic类型: raw/raw
(6)IMU数据类型: 没有normalized，不需要x9.8


2. 用DIO-RO在线跑（前端用kinect）（实验室）:
(1)运行脚本的命令：./ok_lab.sh
(2)用的launch: /home/javens/Workspace/catkin_0115/src/ROGIO-ROS-threads/launch/kinect_lab.launch
(3)用的config: /home/javens/Workspace/catkin_0115/src/ROGIO-ROS-threads/config/Kinect_lab_config.yaml
(4)用的rviz文件: 
(5)Topic类型: compressed/compressedDepth
(6)IMU数据类型: 没有normalized，不需要x9.8


3. 用DIO-RO离线跑kinect录的bag（实验室）:
(1)运行脚本的命令：./ok_lab_of.sh
(2)用的launch: /home/javens/Workspace/catkin_0115/src/ROGIO-ROS-threads/launch/kinect_lab.launch
(3)用的config: /home/javens/Workspace/catkin_0115/src/ROGIO-ROS-threads/config/Kinect_lab_config.yaml
(4)用的rviz文件: 
(5)Topic类型: compressed/compressedDepth
(6)IMU数据类型: 没有normalized，不需要x9.8


4. 用DIO-RO跑VCU-RVI数据集的bag
(1)运行脚本的命令: ./ok.sh
(2)用的launch: 
(3)用的config: 
(4)用的rviz文件:
(5)Topic类型: 
(6)IMU数据类型: 


***注意：
如果image topic要选择接收raw或者compressed，在源码的
/home/javens/Workspace/catkin_0115/src/ROGIO-ROS-threads/src/main.cpp
的line 486 处进行修改。

