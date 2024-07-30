---
title: FLIR相机配置与使用
date: 2024-07-30 16:40:00 +0800
categories: [SLAM]
tags: [相机, ROS]
toc: true 
comments: false
math: true

---
# Spinnaker SDK配置
arm linux下安装官方手册：https://www.flir.cn/support-center/iis/machine-vision/application-note/understanding-usbfs-on-linux/

下载地址（需要登陆）：https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-download/spinnaker-sdk--download-files/

选择Spinnaker 3.2.0.62 for Ubuntu 20.04 (May 31, 2024) 64-bit-ARM

安装依赖
```shell
sudo apt-get install libraw1394-11 libusb-1.0-0
```
Untar安装包：
```shell
tar xvfz spinnaker-3.2.0.62-arm64-pkg.20.04.tar.gz
```
运行安装脚本：
```shell
cd spinnaker-3.2.0.62-arm64/
sudo sh install_spinnaker_arm.sh
```
按照脚本的说明操作。 这将安装所有 Spinanaker 库、示例代码、示例应用和文档。 
此外，该脚本将提示您配置 udev，以便设备可由特定用户使用。 如果选择配置设备，该脚本将更改节点权限，具体做法是覆盖默认 Ubuntu 权限，并向用户提供对设备节点的完整读取和写入权限。(这一步跳过未配置)

打开SpinView，USB Interface 0 显示 No Devices Detect...，说明设备没连接成功，是USB权限问题，修改ruls解决：
```shell
cd /etc/udev/rules.d
sudo gedit 40-flir-spinnaker.rules
```
将40-flir-spinnaker.rules中的
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10",GROUP="flirimaging"
```
改为：
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="4000", MODE:="0777",GROUP="flirimaging"
```
重新插拔，即可显示。

# ROS运行
参考官方资料：https://www.flir.eu/support-center/iis/machine-vision/application-note/using-ros-with-spinnaker/

中文网上的spinnaker_sdk_camera_driver太古老，不适用于OpenCV4.
配置要求：Ubuntu20.04，ROS-noetic（虽然github仓库中写明需要ROS2，但是亲测ROS1也可）

安装依赖：
```shell
sudo apt install ros-noetic-camera-info-manager ros-noetic-diagnostic-updater ros-noetic-dynamic-reconfigure ros-noetic-image-exposure-msgs ros-noetic-image-transport ros-noetic-nodelet ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-wfov-camera-msgs
```
ROS Workspace：
```shell
mkdir -p catkin_ws_spinnaker/src
cd catkin_ws_spinnaker/src
git clone -b noetic-devel https://github.com/ros-drivers/flir_camera_driver.git
cd ..
catkin_make
```
可将setup.bash写入变量：
```shell
echo "source ~/catkin_ws_spinnaker/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
启动相机流传输图像：
```shell
roslaunch spinnaker_camera_driver camera.launch
```
启动时报错：Error: Spinnaker: Not enough available memory to allocate buffers for streaming.
原因是默认分配内存太小(16)，将其增大为1000后重新启动：
```shell
sudo gedit /sys/module/usbcore/parameters/usbfs_memory_mb
```
更改设置：
```shell
rosrun rqt_reconfigure rqt_reconfigure
```
显示图像：
```shell
rqt_image_view
```
![](/assets/img/FLIRInstructions.png)


