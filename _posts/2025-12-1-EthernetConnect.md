---
title: jetson开发板网口连接
date: 2025-12-12 12:00:00 +0800
categories: [Linux]
tags: [Linux, 工具]
toc: true 
comments: true
math: true
---

# 开发板串口连接
开发板拿到手，如果没有显示屏，可以通过串口连接电脑，再通过mobaXterm新建serial设置好波特率打开。

# 开发板网口连接
```
ip a
```
插上网口和拔出网口分别运行上面命令，可以对比出网口名称（我的jetson orin nano为eth0）

设置静态ip：
```
sudo nmcli connection add type ethernet ifname eth0 con-name eth0-static ipv4.addresses 192.168.122.10/24 ipv4.method manual
sudo nmcli connection up eth0-static
```
如果 eth0 之前有连接，需要先删除：
```
sudo nmcli connection delete eth0
```
启用静态配置：
```
sudo nmcli connection up eth0-static
```
# PC端网络配置
控制面板->网络和Internet->查看网络状态和任务->以太网->属性->IPv4

设置ip地址192.168.122.5，子网掩码255.255.255.0，默认网关无。

