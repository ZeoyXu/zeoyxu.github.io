---
title: sudo python和python环境不同导致的"No Module"
date: 2024-07-22 16:00:00 +0800
categories: [Linux]
tags: [Linux, Python]
toc: true 
comments: false
math: true

---

# 问题复现
在jetson AGX Orin中通过whl安装了jetson版本的pytorch2.1，在torchvision 0.16文件夹中运行安装命令：
```python
python setup.py install
```
报错，权限不够。于是加上权限：
```python
sudo python setup.py install
```
却提示No Module named "torch"。

# 原因
sudo python和python的路径不同，简单测试：

**python**:

```python
import sys
print(sys.path)
```
输出：
```shell
['', 
'/usr/lib/python38.zip', 
'/usr/lib/python3.8', 
'/usr/lib/python3.8/lib-dynload', 
'/home/xzy/.local/lib/python3.8/site-packages', 
'/usr/local/lib/python3.8/dist-packages', 
'/usr/lib/python3/dist-packages', 
'/usr/lib/python3.8/dist-packages']
```

**sudo python**:
```python
import sys
print(sys.path)
```
输出：
```shell
['', 
'/usr/lib/python38.zip', 
'/usr/lib/python3.8', 
'/usr/lib/python3.8/lib-dynload', 
'/usr/local/lib/python3.8/dist-packages',
'/usr/lib/python3/dist-packages', 
'/usr/lib/python3.8/dist-packages']
```
# 解决
将python中的库通过链接的方式配置到sudo python底下。
在sudo python的/usr/local/lib/python3.8/dist-packages路径下新建路径：
```shell
sudo gedit xzy_py39_path.pth
```
将上面python的路径的路径加入其中
```
/usr/lib/python38.zip
/usr/lib/python3.8
/usr/lib/python3.8/lib-dynload
/home/xzy/.local/lib/python3.8/site-packages
/usr/local/lib/python3.8/dist-packages
/usr/lib/python3/dist-packages
/usr/lib/python3.8/dist-packages
```
再次导入，大功告成！！！