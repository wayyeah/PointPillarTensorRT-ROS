## PointPillarTensorRT-ROS
PointPillarTensorRT-ROS is an implementation of the PointPillar object detection algorithm in a TensorRT-optimized version, with visualization support using ROS neotic.

# Installation
To install PointPillarTensorRT-ROS, follow these steps:
```
sudo apt-get install ros-noetic-jsk-rviz-plugins
sudo apt-get install git-lfs && git lfs install
git clone https://github.com/wayyeah/PointPillarTensorRT-ROS.git
cd PointPillarTensorRT-ROS
bash compile.sh
```
If your TensorRT is 8.4.x please do this
```
rm -rf /src/pointpillar.onnx
mv /src/pointpillar/model/pointpillar.onnx /src/pointpillar.onnx
```
# Usage
1、start the ROS core:
```
roscore
```
2、play the kitti bag:
```
rosbag play kitti_2011_09_26_drive_0009_synced.bag
```
download link: https://pan.baidu.com/s/14lB2Djw6iiivfuhaINgkyA?pwd=asr8  key: asr8

3、run 
```
rosrun pointpillar pointpillar
```
4、start rviz and add topic 
```
rviz
```
<img width="556" alt="3" src="https://github.com/wayyeah/PointPillarTensorRT-ROS/assets/53206282/1b2d7259-2960-4279-8625-03c74d1409e9">
![4](https://github.com/wayyeah/PointPillarTensorRT-ROS/assets/53206282/fba096f5-dbd0-4583-a9b7-39567eab6c32)
![5](https://github.com/wayyeah/PointPillarTensorRT-ROS/assets/53206282/ffa8366d-307e-42a3-88ef-ef9b8e182f13)

# Result
![6](https://github.com/wayyeah/PointPillarTensorRT-ROS/assets/53206282/007c62e0-e007-4311-8a82-9542637d01d0)



