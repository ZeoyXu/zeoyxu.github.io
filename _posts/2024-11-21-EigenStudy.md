---
title: Eigen学习
date: 2024-11-21 10:48:00 +0800
categories: [SLAM]
tags: [SLAM, Eigen, Sophus]
toc: true 
comments: false
math: true
---

# 常用函数
## 旋转矩阵
```cpp
// Eigen::AngleAxisd(angle, axis)表示绕axis轴旋转angle角度
Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitX());
```

## 四元数
### 从旋转矩阵创建四元数
```cpp
Eigen::Quaterniond q = Eigen::Quaterniond(R);
```
### 直接创建四元数
```cpp
Eigen::Quaterniond q(1, 2, 3, 4);
std::cout << "q= " << q_new.coeffs() << std::endl;
```
此处1,2,3,4分别为w,x,y,z但是q储存、计算的顺序是x,y,z,w

## 李群
### 创建李群
```cpp
// 将R和w转化成李群形式
Sophus::SO3<double> SO3_R(R);
// 等价于
Sophus::SO3d SO3_R(R);
```
### 旋转矩阵扰动
```cpp
// w为小扰动
Eigen::Vector3d w(0.01, 0.02, 0.03);
Sophus::SO3<double> SO3_R_new = SO3_R * Sophus::SO3d::exp(w);
// 转换回旋转矩阵
std::cout << "R NEW = " << SO3_R_new.matrix() << std::endl;
```

# 四元数和李代数更新对比
对于小扰动，直接用四元数计算和李群计算，差别不大。
```cpp
#include <iostream>
#include <Eigen/Dense>
#include "sophus/so3.hpp"

int main()
{
    // 初始化
    Eigen::Matrix3d R;
    Eigen::Matrix3d R_new;
    Eigen::Quaterniond q;
    Eigen::Quaterniond q_new;
    Eigen::Vector3d w(0.01, 0.02, 0.03);

    R = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitX());
    std::cout << "R = " << std::endl << R << std::endl;

    q = Eigen::Quaterniond(R);
    std::cout << "q = " << std::endl << q.coeffs() << std::endl;

    // 将R和w转化成李群形式
    Sophus::SO3<double> SO3_R(R);
    Sophus::SO3<double> SO3_w = Sophus::SO3d::exp(w);

    Sophus::SO3<double> SO3_R_new = SO3_R * SO3_w;
    R_new = SO3_R_new.matrix();
    std::cout << "R NEW = " << std::endl << R_new << std::endl;

    Eigen::Quaterniond q_w(1, 0.5*w(0), 0.5*w(1),0.5*w(2));
    q_new = q * q_w;
    std::cout << "q NEW = " << std::endl << q_new.coeffs() << std::endl;

    // 把扰动后的R转换成四元数
    Eigen::Quaterniond q_R_new(R_new);
    std::cout << "q_R_new = " << std::endl << q_R_new.coeffs() << std::endl;

    //把扰动后的q转换成R
    Eigen::Matrix3d R_q_new = q_new.toRotationMatrix();
    std::cout << "R_q_new = " << std::endl << R_q_new << std::endl;

    return 0;
}
```

```CMakeLists
cmake_minimum_required(VERSION 3.5)

project(MyStudy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")

# 包含eigen
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Sophus_INCLDE_DIR})

add_executable(q_R_compare q_R_compare.cpp)
```
