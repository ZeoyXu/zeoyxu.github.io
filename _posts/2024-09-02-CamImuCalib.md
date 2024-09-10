---
title: 相机IMU（联合）标定
date: 2024-09-10 15:59:00 +0800
categories: [SLAM]
tags: [相机, IMU, 标定]
toc: true 
comments: false
math: true
---
# 设备
IMU: STIM300

相机: FLIR BFS-U3-123S6M-C
# IMU标定
使用imu_utils
## 安装依赖
```shell
sudo apt-get install libdw-dev
```
### 安装abseil-cpp（Ceres依赖于此）
```shell
git clone https://github.com/abseil/abseil-cpp
cd abseil-cpp
mkdir bulid
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j8
make install   
```
### 安装Ceres(code_imu依赖于此)
依赖：
```shell
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
sudo apt-get install libsuitesparse-dev
```
安装1.14版本
```shell
git clone https://ceres-solver.googlesource.com/ceres-solver
mkdir build
cd build
cmake ..
make
sudo make install
```
## 编译imu_utils
imu_utils下载地址为：https://github.com/gaowenliang/imu_utils
code_utils下载地址为：https://github.com/gaowenliang/code_utils

**不要同时把imu_utils和code_utils一起放到src下进行编译！！！**由于imu_utils依赖code_utils，所以先把code_utils放在工作空间的src下面，进行编译。然后再将imu_utils放到src下面，再编译。
```shell
mkdir -p imu_utils/src
cd imu_utils/src
git clone https://github.com/gaowenliang/code_utils.git
cd ..
catkin_make
```
不出意外会报一堆错，修改如下：

**CMakeLists.txt文件下：**

修改
```cmake
set(CMAKE_CXX_FLAGS "-std=c++11")
```
为

```cmake
set(CMAKE_CXX_FLAGS "-std=c++14")
```

**sumpixel_test.cpp文件下：**

修改
```cpp
#include "backward.hpp"
```
为
```cpp
#include “code_utils/backward.hpp”
```
添加头文件：

```cpp
#include"opencv2/imgcodecs/legacy/constants_c.h"
```

修改所有OpenCV函数中的
```cpp
CV_MINMAX
```
为
```cpp
NORM_MINMAX
```
**mat_io_test.cpp文件下：**
修改所有的
```cpp
CV_LOAD_IMAGE_UNCHANGED
```
为
```cpp
cv::IMREAD_UNCHANGED
```
再次编译，大功告成！！！

code_utils编译成功以后，再把imu_utils放到工作空间的src下面，进行编译。
```shell
cd imu_utils/src
git clone https://github.com/gaowenliang/code_utils.git
cd ..
catkin_make
```
不出意外，还会报错！

**CMakeLists.txt文件下：**

修改
```cpp
set(CMAKE_CXX_FLAGS "-std=c++11")
```
为
```cpp
set(CMAKE_CXX_FLAGS "-std=c++14")
```

**imu_an.cpp文件下：**

添加头文件：
```cpp
#include <fstream>
```
再次编译，大功告成！！！

## 采集IMU数据
首先给ttyUSB口读写权限：
```shell
sudo chmod 777 /dev/ttyUSB0
```
下载STIM300驱动的ROS文件driver_stim300

编译后启动：
```shell
roslaunch driver_stim300 stim300_driver.launch
```
数据发布在/imu/data_raw话题中，显示数据：
```shell
rostopic echo "/imu/data_raw"
```
录制imu数据包，全程保持imu静止不动，imu上电运行10分钟后开始录制2小时:
```shell
rosbag record -O imu.bag /imu/data_raw
```
录制完成后，在imu_utils/src/imu_utils/launch/下找到xsense.launch,修改imu话题名和max_time_min，两小时就设为120
```yaml
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/imu/data_raw"/>
        <param name="imu_name" type="string" value= "xsens"/>
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <param name="max_time_min" type="int" value= "120"/>
        <param name="max_cluster" type="int" value= "100"/>
    </node>
</launch>
```
启动roslaunch：
```shell
source  devel/setup.bash
roslaunch imu_utils xsense.launch 
```
以200倍速度播放bag：
```
rosbag play -r200  imu.bag
```
标定结果存在imu_utils/src/imu_utils/data/下xsense_imu_param.yaml：
```yaml
%YAML:1.0
---
type: IMU
name: xsens
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 6.2250202132721748e-04
      gyr_w: 6.4414378129696065e-06
   x-axis:
      gyr_n: 7.1141832406509606e-04
      gyr_w: 4.7528548657982937e-06
   y-axis:
      gyr_n: 5.2261518175758471e-04
      gyr_w: 8.6560577587899026e-06
   z-axis:
      gyr_n: 6.3347255815897146e-04
      gyr_w: 5.9154008143206231e-06
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 2.2259852935722958e-02
      acc_w: 3.8719984514737604e-04
   x-axis:
      acc_n: 2.2648432844017852e-02
      acc_w: 4.1400359180647977e-04
   y-axis:
      acc_n: 2.1637273363403108e-02
      acc_w: 3.8028695298288312e-04
   z-axis:
      acc_n: 2.2493852599747906e-02
      acc_w: 3.6730899065276517e-04
```
# 相机标定
## 安装Kalibr(ROS)
安装依赖：
```shell
pip install pyx
sudo apt-get install python-pyx
pip install Cpython
pip install setuptools==41.0.1
pip install wxPython==4.0.7 -i https://pypi.tuna.tsinghua.edu.cn/simple

```
其余依赖缺啥装。
```shell
mkdir -p kalibr/src
cd kalibr/src
git clone https://github.com/ethz-asl/kalibr.git
cd ..
catkin_make
```
生成标定板PDF文件：
```shell
rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.03 --tspace 0.3
```
根据生成的标定板参数编写april.yaml文件：
```yaml
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.02           #size of apriltag, edge to edge [m]
tagSpacing: 0.3          #ratio of space between tags to tagSize
codeOffset: 0            #code offset for the first tag in the aprilboard
```
改变相机帧率至4Hz(官方要求)，运行相机，并录制bag
```shell
rosbag record /camera/image_raw -O cam_calib.bag
```
录制过程中，先绕相机的各自三个轴，每个轴旋转三次，然后再沿着相机的三个轴，每个轴平移三次，之后再随机运动，最好120s以上，运动期间要保证相机基本能一直看到标定板的全部信息。
kalibr_calibrate_cameras启动：
```shell
rosrun kalibr kalibr_calibrate_cameras --target april.yaml --bag cam_calib.bag --models pinhole-radtan --topics /camera/image_raw --bag-from-to 1 137 --show-extraction --approx-sync 0.1
```
--target 自己标定板yaml文件的路径

--bag 刚刚录制的cam_calib.bag

--models pinhole-radtan 摄像头的相机模型和畸变模型(有几个相机写几遍)

--topics /camera/image_raw话题名称

--bag-from-to 1 137 表示处理bag中1-137秒的数据

--show-extraction 显示检测特征点的过程

--approx-sync 0.01 相机时间同步的容许误差（单目似乎不需要）

**报错：**ImportError: /lib/aarch64-linux-gnu/libgomp.so.1: cannot allocate memory in static TLS block or /home/xzy/.local/lib/python3.8/site-packages/igraph/../igraph.libs/libgomp-d22c30c5.so.1.0.0
将下面语句加入bashrc(直接export也行)：
```shell
$ export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1:/home/xzy/.local/lib/python3.8/site-packages/igraph/../igraph.libs/libgomp-d22c30c5.so.1.0.0
```
Finally we get cam_calib-camchain.yaml, cam_calib-results-cam.txt, cam_calib-report-cam.pdf.
# 相机IMU联合标定
## 帧率设置
kalibr推荐标定帧率为相机20Hz，IMU200Hz。本人所用的FLIR相机可直接在roslaunch文件里设置帧率；IMU帧率通过topic_tools里的throttle工具修改：
```shell
rosrun topic_tools throttle messages /imu/data_raw 200.0 /imu_200
```
(理论上应设置成200.0，但实际我设置成300，录制的rosbag为200帧)
## 修改IMU标定格式
上面得到的IMU标定文件和kalibr所需的格式不符，修改前：
```yaml
%YAML:1.0
---
type: IMU
name: xsens
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 6.2250202132721748e-04
      gyr_w: 6.4414378129696065e-06
   x-axis:
      gyr_n: 7.1141832406509606e-04
      gyr_w: 4.7528548657982937e-06
   y-axis:
      gyr_n: 5.2261518175758471e-04
      gyr_w: 8.6560577587899026e-06
   z-axis:
      gyr_n: 6.3347255815897146e-04
      gyr_w: 5.9154008143206231e-06
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 2.2259852935722958e-02
      acc_w: 3.8719984514737604e-04
   x-axis:
      acc_n: 2.2648432844017852e-02
      acc_w: 4.1400359180647977e-04
   y-axis:
      acc_n: 2.1637273363403108e-02
      acc_w: 3.8028695298288312e-04
   z-axis:
      acc_n: 2.2493852599747906e-02
      acc_w: 3.6730899065276517e-04
```
修改后：
```yaml
#Accelerometers
accelerometer_noise_density: 2.2259852935722958e-02   #Noise density (continuous-time)
accelerometer_random_walk: 3.8719984514737604e-04   #Bias random walk

#Gyroscopes
gyroscope_noise_density: 6.2250202132721748e-04   #Noise density (continuous-time)
gyroscope_random_walk: 6.4414378129696065e-06   #Bias random walk

rostopic: /imu_200       #the IMU ROS topic
update_rate: 200.0      #Hz (for discretization of the values above)
```
## 录制rosbag并标定
录制方式同相机标定，充分激励相机和IMU，最好时长大于2min。
```shell
rosbag record -O camImu.bag /camera/camera_raw /imu_200
```

```shell
rosrun kalibr kalibr_calibrate_imu_camera --target ./april.yaml  --cam ./cam_calib-camchain.yaml --imu ./IMUcalib.yaml --bag ./camImu.bag  --bag-from-to 1 111 --show-extraction
```
--target：标定板参数信息;

--cam：Camera的标定文件;

--imu：IMU的标定文件;

--bag：录制的Camera-IMU数据包;

--bag-from-to：起始和终止时间，单位是秒;

等待一段比较长的时间后(由于我像素比较大，等了6h)会得到标定结果。


