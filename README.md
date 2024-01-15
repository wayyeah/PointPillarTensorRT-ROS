## PointPillarTensorRT-ROS
PointPillarTensorRT-ROS is an implementation of the PointPillar object detection algorithm in a TensorRT-optimized version, with visualization support using ROS neotic.

# Installation
To install PointPillarTensorRT-ROS, follow these steps:
```
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
![5](https://github.com/wayyeah/PointPillarTensorRT-ROS/assets/53206282/a4f2fa32-b752-4600-8f77-a96d127fd87a)
![4](https://github.com/wayyeah/PointPillarTensorRT-ROS/assets/53206282/a19cdbc1-e678-498c-84bf-4327f9351efb)
# Result
![Uploading 6.png…]()
