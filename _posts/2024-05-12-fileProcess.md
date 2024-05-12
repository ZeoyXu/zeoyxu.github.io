---
title: 文件处理操作
date: 2024-05-12 17:00:00 +0800
categories: [技巧类]
tags: [文件处理, Python]
toc: true 
comments: false
math: true

---

# 常见文件处理操作

## 重命名文件

将文件从1到n命名

```python
import os


path = '/home/nvidia/SAAYOLO/datasets_new/data_raw/part8'
files = os.listdir(path)

n = 0
for i, file in enumerate(files):
    NewName = os.path.join(path, str(n + 1) + ".jpg")
    OldName = os.path.join(path, file)
    os.rename(OldName, NewName)
    n += 1
```

## 复制并且重命名文件

```python
import os
import shutil


path = '/home/nvidia/SAAYOLO/datasets/labels'
files = os.listdir(path)

for i, file in enumerate(files):
    OldName = os.path.join(path, file)
    NewName1 = os.path.join(path, 'gsnoise' + file)
    shutil.copy(OldName, NewName1)

    NewName2 = os.path.join(path, 'spnoise' + file)
    shutil.copy(OldName, NewName2)

    NewName3 = os.path.join(path, 'randnoise' + file)
    shutil.copy(OldName, NewName3)
    
```

## 将某一类文件移动到另一文件夹

```python
import os
import shutil


seq_dir = '/home/nvidia/SAAYOLO/datasets_new/data02_aug'
target_dir = '/home/nvidia/SAAYOLO/datasets_new/data02_aug/images'

files = os.listdir(seq_dir)

for f, file in enumerate(files):
    # 判断文件后缀
    if os.path.splitext(file)[-1] == '.jpg':
        src = os.path.join(seq_dir, file)
        dst = os.path.join(target_dir, file)      
        shutil.move(src, dst)
```

# txt内容操作

将txt文件的每一行第1个字符替换成0

```python

```
