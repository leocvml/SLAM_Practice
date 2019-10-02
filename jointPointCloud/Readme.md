1. use RGBD camera capture RGB and Depth image
2. get extrinsic (pose) parameters ( Quaterniond and translation matrix )
3. normalization pixel coordinate system[u v 1] * depth => camera coordinate system 
4. camera coordinate system * extrinsic (pose) => JointPointCloud
![image](https://github.com/leocvml/SLAM_Practice/blob/master/jointPointCloud/JointCloudPoint.png)
