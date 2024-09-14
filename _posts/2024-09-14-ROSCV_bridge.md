---
title: 使用cv_bridge将ROS图像消息转换为OpenCV图像时错误
date: 2024-09-14 11:55:00 +0800
categories: [ROS]
tags: [ROS, OpenCV, SLAM]
toc: true 
comments: false
math: true
---
# 问题复现
自己写一段代码实现功能：从/camera/image_raw中读取4K视频流，压缩成720p，再以同样帧率发布/camera/image_720p（其实有现成的工具）： 
```cpp
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

// 全局变量
image_transport::Publisher pub;

Mat FLIRImage;
Mat resized_image;

void myCVCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        FLIRImage = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::resize(FLIRImage, resized_image, cv::Size(1280, 720));
        ROS_INFO("size of resized image is %d, %d", resized_image.cols, resized_image.rows);
        sensor_msgs::ImagePtr cam720p = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_image).toImageMsg();
        pub.publish(cam720p);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "myCam_node");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber cv_sub = it.subscribe("/camera/image_raw", 1, myCVCallback);

    pub = it.advertise("/camera/image_720p", 1);

    ros::spin();
    
    return 0;
}
```
程序编译无误，运行时报错：
```shell
terminate called after throwing an instance of 'cv::Exception' what():  OpenCV(4.8.1) /home/xzy/opencv-4.8.1/modules/core/src/matrix.cpp:246: error: (-215:Assertion failed) s >= 0 in function 'setSize'
```
# 原因
项目中用到的cv_bridge，会调用ROS自带的OpenCV中的库（OpenCV 4.2.0），如果此时项目中引用了自己安装的OpenCV（OpenCV 4.8.1）会与ros自带的OpenCV中的cv_bridge冲突。编译时也有警告：
```shell
/usr/bin/ld: warning: libopencv_imgcodecs.so.4.2, needed by /opt/ros/noetic/lib/libcv_bridge.so, may conflict with libopencv_imgcodecs.so.408
/usr/bin/ld: warning: libopencv_core.so.4.2, needed by /opt/ros/noetic/lib/libcv_bridge.so, may conflict with libopencv_core.so.408
/usr/bin/ld: warning: libopencv_features2d.so.4.2, needed by /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.2.0, may conflict with libopencv_features2d.so.408
/usr/bin/ld: warning: libopencv_imgproc.so.408, needed by /usr/local/lib/libopencv_imgcodecs.so.4.8.1, may conflict with libopencv_imgproc.so.4.2
```
# 解决方案
## 方案一（不推荐）
修改/opt/ros/noetic/share/cv_bridge/cmake文件夹中的cv_bridgeConfig.cmake，ctrl+f 搜索set(libraries 会看到cv_bridge所需要的库，这里面都是 .4.2.0的版本，将发生冲突的库连着路径改为自己的版本4.8.1。

但是这个方案过于繁琐，要改这么多：
```cmake
set(libraries "cv_bridge;/usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_video.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_face.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_text.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.2.0;/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.2.0")
```
## 方案二
重新下载cv_bridege，并在编译前制定自己的OpenCV版本。
```shell
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv
git checkout noetic
```
将其中的cv_bridge文件夹单独复制出来，修改CMakeLists.txt中find_package(OpenCV 4 REQUIRED)为find_package(OpenCV 4.8.1 REQUIRED)。
编译：
```shell
mkdir build
cd build
cmake ..
make
sudo make install
```
最后在项目中CMakeLists.txt中find_package前面添加
```cmake
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
```
大功告成！！！