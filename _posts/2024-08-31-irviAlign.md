---
title: 红外-可见光对齐方法（失败）
date: 2025-08-31 22:00:00 +0800
categories: [图像处理]
tags: [红外, 图像处理]
toc: true 
comments: false
math: true
---
# 问题提出
当使用红外(ir)和可见光(vi)图像融合时，红外和可见光拍摄图像匹配困难，故想提出一种可以实时运行的红外和可见光图像对齐算法。
# 思路
大部分可见光相机拍摄范围及像素都高于红外图像，故可以简单固定两个相机，保证红外图像位于可见光图像内。此时，可见光可以经过缩放h，水平和竖直位置像素的裁减对齐a,b转换为与红外图像匹配得上的图像。
对两幅图像进行去噪,边缘提取,二值化，得到骨干网络。
两个图像中相同部分骨干网络应该是差不多的，因此可以最小化两幅图像的像素差值优化a,b,h（实际就是vi->ir的转化矩阵）得到准确的匹配图像。
# 问题
在使用Canny算子提取vi和ir的边缘后，发现由于红外和可见光图像的模态差别，两幅图像提取的骨干网络重点完全不同，因此不可能匹配上。（也许可以用神经网络提取骨干使得侧重点一致？也悬！）
# Code
```cpp
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;

// 图像边缘提取
// threshold1和threshold2表示边缘提取强度
cv::Mat edgeDetect(cv::Mat img, int threshold1, int threshold2) {
    //转换灰度图
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);  
    cv::blur(img_gray, img_gray, cv::Size(3, 3));
    cv::Mat img_edge;
    cv::Canny(img_gray, img_edge, threshold1, threshold2);
    return img_edge;
}

int main(int argc, char **argv) {
    cv::Mat img_ir = cv::imread("/home/xzy/IVMatchinng/infrared.jpg");
    cv::Mat img_vi = cv::imread("/home/xzy/IVMatchinng/vis.jpg");
    if(img_ir.empty() or img_vi.empty()) {
        cout << "Can't load image!!!" << endl;
    }
    cv::Mat irEdge = edgeDetect(img_ir, 10, 30); 
    cv::Mat viEdge = edgeDetect(img_vi, 50, 180);
    cv::imshow("红外边缘", irEdge);
    cv::imshow("可见光边缘", viEdge);
    cv::waitKey(0);
}
```

```cmake
cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_FLAGS "-std=c++11")
project(IVMatching)

# 查找OpenCV库
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "config: ${OpenCV_DIR}")
message(STATUS "version: ${OpenCV_VERSION}")
message(STATUS "libraries: ${OpenCV_LIBS}")
message(STATUS "include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(IVMatching matching.cpp)

# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加链接
target_link_libraries(IVMatching ${OpenCV_LIBS})
```
