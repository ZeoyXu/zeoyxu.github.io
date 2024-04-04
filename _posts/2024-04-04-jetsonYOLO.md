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

```Plain
sudo apt-get install libopenblas-base libopenmpi-devsudo apt-get install libjpeg-dev zlib1g-dev
```

查看jetpack版本：

```Plain
sudo apt-cache show nvidia-jetpack
```

下载对应jetpack版本的pytorch：https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

安装Cpython CUDA torch：

```Plain
pip3 install Cythonpip3 install pycuda --user  # 可不用pip3 install numpy torch-1.14.0-cp38-cp38m-linux_aarch64.whl
```

检验是否安装成功：

```Plain
python3import torch
```

找到pytorch对应版本的torchvision：https://github.com/pytorch/vision

安装torchvision：

```Plain
pip3 install pillowgit clone --branch  v0.16.0 https://github.com/pytorch/vision torchvision
```

编译：

```Plain
cd torchvisionpython3 setup.py install --user
```

# YOLOv8环境

安装ultralytics库（包含了YOLOv8的依赖）：

```Plain
pip install ultralytics
```

下载yolov8

```Plain
git clone https://github.com/ultralytics/ultralytics.git
```

```Python
import cv2from ultralytics import YOLO# 导入 YOLOv8 模型# 官网下载的训练好的模型model = YOLO('./yolov8n.pt')# 打开视频文件# 自己拍频屏的一段视频video_path = "./1848114474.mp4"cap = cv2.VideoCapture(video_path)while cap.isOpened():    success, frame = cap.read()    if success:        results = model(frame)        annotated_frame = results[0].plot()        cv2.imshow("YOLOv8 Inference", annotated_frame)        if cv2.waitKey(1) & 0xFF == ord("q"):            break    else:        breakcap.release()cv2.destroyAllWindows()
```

结果：

![](/assets/img/jetsonYOLO1.png)

![](/assets/img/jetsonYOLO2.png)

# 训练自己的数据集

在ultralytics/ultralytics目录下创建datasets文件夹，将数据集分为test、train、valid三个文件。

在datasets文件夹下建立boxIdentify.yaml文件(名称自取，注意yaml书写格式)

```Plain
# boxIdentifytrain: ...ultralytics\ultralytics\datasets\trainval: ...ultralytics\ultralytics\datasets\validtest: ...ultralytics\ultralytics\datasets\test# Number of classesnc: 3# Classesnames:    0: l    1: m    2: s
```

在cfg/models/v8目录下，修改yolov8.yaml文件，修改类别数nc: 3(numbers of classes)

下载预训练模型(这次用了最小的yolov8n.pt模型)

从预训练模型开始训练：

```Plain
yolo task=detect mode=train model=.../yolov8n.pt epochs=100 batch=1 data=.../datasets/boxIdentify,yaml
```

![](/assets/img/jetsonYOLO3.png)

训练完在.../runs/detect/train/weights文件下生成best.pt和last.pt权重

模型导出：

```Plain
yolo task=detect mode=export model=.../best.pt
```
