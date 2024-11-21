---
title: VS Code C++配置
date: 2024-11-21 13:48:00 +0800
categories: [C++]
tags: [C++, VS Code]
toc: true 
comments: false
math: true
---

# 路径配置
终端运行
```shell
gcc -v -E -x c++ -
```
将#include <...> search starts here:下面的路径粘贴到c_cpp_properties.json：
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/include/c++/9/**",
                "/usr/include/x86_64-linux-gnu/c++/9/**",
                "/usr/include/c++/9/backward/**",
                "/usr/lib/gcc/x86_64-linux-gnu/9/include/**",
                "/usr/local/include/**",
                "/usr/include/x86_64-linux-gnu/**",
                "/usr/include/**"
               
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

# 一键cmake
tasks.json里配置：
``` json
{   
    "version": "2.0.0",
    "options": {
        "cwd": "${workspaceFolder}/build"
    },
    "tasks": [
        {
            "type": "shell",
            "label": "cmake",//第一个动作，执行cmake
            "command": "cmake",
            "args": [
                ".."
            ]
        },
        {
            "label": "make",//第二个动作，执行make
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "command": "make",
            "args": [
                

            ]
        },
        {
            "label": "Build",//把上面的两个动作绑定，命名为Build
			"dependsOrder": "sequence", 
            "dependsOn":[
                "cmake",
                "make"
            ]
        }
    ]

}
```
之后直接Ctrl+Shift+B就可以直接生成可执行文件！

# 调试配置
launch.json中配置：
``` json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) 启动",
            "type": "cppdbg",
            "request": "launch",
            //只需要改两处
            //第一处，在这里指定你的可执行文件的路径和名称
            "program": "${workspaceFolder}/build/q_R_compare",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${fileDirname}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "preLaunchTask": "Build",//第二处，绑定刚才的task
            "setupCommands": [
                {
                    "description": "为 gdb 启用整齐打印",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```
之后可以F5愉快断点调试！