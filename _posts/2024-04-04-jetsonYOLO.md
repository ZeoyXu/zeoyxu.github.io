---
title: Jetson运行YOLOv8
date: 2024-04-04 11:53:00 +0800
categories: [YOLO]
tags: [yolo, jetson, linux]
toc: true 
comments: false
math: true

---

开发板型号：Jetson Orin NX

系统版本：ubuntu 20.04

本文不深究YOLO算法原理，仅仅展示如何在jetson边缘计算板利用YOLOv8训练分类模型。

# pytorch安装

安装依赖：

```shell
sudo apt-get install libopenblas-base libopenmpi-dev
sudo apt-get install libjpeg-dev zlib1g-dev
```

查看jetpack版本：

```shell
sudo apt-cache show nvidia-jetpack
```

下载对应jetpack版本的pytorch：https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

安装Cpython CUDA torch：

```shell
pip3 install Cython
pip3 install pycuda --user  # 可不用pip3 install numpy torch-1.14.0-cp38-cp38m-linux_aarch64.whl
```

检验是否安装成功：

```python
python3 
import torch
```

找到pytorch对应版本的torchvision：https://github.com/pytorch/vision

安装torchvision：

```shell
pip3 install pillowgit clone --branch  v0.16.0 https://github.com/pytorch/vision torchvision
```

编译：

```shell
cd torchvision
python3 setup.py install --user
```

# YOLOv8环境

安装ultralytics库（包含了YOLOv8的依赖）：

```shell
pip install ultralytics
```

下载yolov8

```git
git clone https://github.com/ultralytics/ultralytics.git
```

```python
import cv2
from ultralytics import YOLO
# 导入 YOLOv8 模型# 官网下载的训练好的模型
model = YOLO('./yolov8n.pt')
# 打开视频文件, 自己拍频屏的一段视频
video_path = "./1848114474.mp4"
cap = cv2.VideoCapture(video_path)
while cap.isOpened():    
    success, frame = cap.read()    
    if success:        
        results = model(frame)        
        annotated_frame = results[0].plot()        
        cv2.imshow("YOLOv8 Inference", annotated_frame)        
        if cv2.waitKey(1) & 0xFF == ord("q"):            
            break    
        else:        
            break
    cap.release()cv2.destroyAllWindows()
```

结果：

![](/assets/img/jetsonYOLO1.png)

![](/assets/img/jetsonYOLO2.png)

# 训练自己的数据集

在ultralytics/ultralytics目录下创建datasets文件夹，将数据集分为test、train、valid三个文件。

**划分数据集：**

```python
import os  # 用于处理文件路径、创建目录等操作
import random  # 用于生成随机数种子、打乱列表等操作
import shutil  # 用于生成随机数种子、打乱列表等操作

# 设置随机数种子
random.seed(123)

# 定义文件夹路径（需要按照自己文件目录修改）
image_dir = 'path/to/images'  # 原始图像所在的子目录
label_dir = 'path/to/lables'  # 原始标签所在的子目录
output_dir = 'yolo_dataset'  # 处理后的数据集输出目录

# 定义训练集、验证集和测试集比例（根据自己的需求修改，可改可不改）
train_ratio = 0.7  # 训练集比例
valid_ratio = 0.15  # 验证集比例
test_ratio = 0.15  # 测试集比例

# 获取所有图像文件和标签文件的文件名（不包括文件扩展名）
image_filenames = [os.path.splitext(f)[0] for f in os.listdir(image_dir)]  # 提取所有图像文件的文件名列表
label_filenames = [os.path.splitext(f)[0] for f in os.listdir(label_dir)]  # 提取所有标签文件的文件名列表

# 随机打乱文件名列表
random.shuffle(image_filenames)  # 打乱图像文件的文件名列表

# 计算训练集、验证集和测试集的数量
total_count = len(image_filenames)  # 总文件数
train_count = int(total_count * train_ratio)  # 训练集文件数
valid_count = int(total_count * valid_ratio)  # 验证集文件数
test_count = total_count - train_count - valid_count  # 测试集文件数

# 定义输出文件夹路径
train_image_dir = os.path.join(output_dir, 'train', 'images')  # 训练集图像输出目录
train_label_dir = os.path.join(output_dir, 'train', 'labels')  # 训练集标签输出目录
valid_image_dir = os.path.join(output_dir, 'valid', 'images')  # 验证集图像输出目录
valid_label_dir = os.path.join(output_dir, 'valid', 'labels')  # 验证集标签输出目录
test_image_dir = os.path.join(output_dir, 'test', 'images')  # 测试集图像输出目录
test_label_dir = os.path.join(output_dir, 'test', 'labels')  # 测试集标签输出目录

# 创建输出文件夹
os.makedirs(train_image_dir, exist_ok=True)  # 创建训练集图像输出目录
os.makedirs(train_label_dir, exist_ok=True)  # 创建训练集标签输出目录
os.makedirs(valid_image_dir, exist_ok=True)  # 创建验证集图像输出目录
os.makedirs(valid_label_dir, exist_ok=True)  # 创建验证集标签输出目录
os.makedirs(test_image_dir, exist_ok=True)  # 创建测试集图像输出目录
os.makedirs(test_label_dir, exist_ok=True)  # 创建测试集标签输出目录

# 将图像和标签文件划分到不同的数据集中
for i, filename in enumerate(image_filenames):
    # 如果文件数量小于训练数据集大小，则将文件复制到训练数据集目录中
    if i < train_count:
        output_image_dir = train_image_dir
        output_label_dir = train_label_dir
    # 如果文件数量小于训练数据集大小+验证数据集大小，则将文件复制到验证数据集目录中
    elif i < train_count + valid_count:
        output_image_dir = valid_image_dir
        output_label_dir = valid_label_dir
    # 否则，将文件复制到测试数据集目录中
    else:
        output_image_dir = test_image_dir
        output_label_dir = test_label_dir

    # 复制图像文件（注意是否为jpg txt保持一致）
    src_image_path = os.path.join(image_dir, filename + '.jpg')  # 获取图像文件的源路径
    dst_image_path = os.path.join(output_image_dir, filename + '.jpg')  # 获取图像文件的目标路径
    shutil.copy(src_image_path, dst_image_path)  # 复制图像文件到目标路径

    # 复制标签文件
    src_label_path = os.path.join(label_dir, filename + '.txt')  # 获取标签文件的源路径
    dst_label_path = os.path.join(output_label_dir, filename + '.txt')  # 获取标签文件的目标路径
    shutil.copy(src_label_path, dst_label_path)  # 复制标签文件到目标路径
```

在datasets文件夹下建立boxIdentify.yaml文件(名称自取，注意yaml书写格式)

```yaml
# boxIdentify
train: /home/nvidia/ultralytics/ultralytics/datasets/train
val: /home/nvidia/ultralytics/ultralytics/datasets/valid
test: /home/nvidia/ultralytics/ultralytics/datasets/test

# number of classes
nc: 3

# Classes
names:
    0: l
    1: m
    2: s
```

在cfg/models/v8目录下，修改yolov8.yaml文件，修改类别数nc: 3(numbers of classes)

下载预训练模型(这次用了最小的yolov8n.pt模型)

从预训练模型开始训练：

```shell
yolo task=detect mode=train model=.../yolov8n.pt epochs=100 batch=1 data=.../datasets/boxIdentify.yaml
```

![](/assets/img/jetsonYOLO3.png)

训练完在.../runs/detect/train/weights文件下生成best.pt和last.pt权重

模型导出：

```shell
yolo task=detect mode=export model=.../best.pt
```
