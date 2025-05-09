---
title: 卡尔曼滤波推导（概率角度）
date: 2025-04-22 15:30:00 +0800
categories: [卡尔曼滤波]
tags: [状态估计, 卡尔曼滤波, 概率论]
toc: true 
comments: true
math: true
---

# 问题引入
机器人状态估计问题本质就是由机器人**之前的状态**、**观测数据**、**控制命令**来求当前的状态。然而对系统的建模有**过程噪声**$w$，观测数据有**观测噪声**$v$.需要从噪声中得到最优估计，因此该方法被称为“滤波”。

# 理论知识
## 正态（高斯）分布
高维高斯分布的概率密度函数为：

$$
p(\mathbf{x} \mid \mathbf{\mu}, \mathbf{\Sigma}) = \frac{1}{(2\pi)^{n/2} |\mathbf{\Sigma}|^{1/2}} \exp \left( -\frac{1}{2} (\mathbf{x} - \mathbf{\mu})^\top \mathbf{\Sigma}^{-1} (\mathbf{x} - \mathbf{\mu}) \right)
$$

## 贝叶斯公式

$$
p(\mathbf{x}, \mathbf{y}) = p(\mathbf{x} \mid \mathbf{y}) p(\mathbf{y}) = p(\mathbf{y} \mid \mathbf{x}) p(\mathbf{x})
$$

多条件下的贝叶斯公式：

$$
    \begin{aligned}
    P(\mathbf{x}|\mathbf{y},\mathbf{z}) 
    &= \frac{p(\mathbf{x},\mathbf{y},\mathbf{z})}{p(\mathbf{y},\mathbf{z})} \\
    &= \frac{p(\mathbf{y}|\mathbf{x},\mathbf{z})p(\mathbf{x},\mathbf{z})}{p(\mathbf{y}|\mathbf{z})p(\mathbf{z})} \\
    &= \frac{p(\mathbf{y}|\mathbf{x},\mathbf{z})p(\mathbf{x}|\mathbf{z})p(\mathbf{z})}{p(\mathbf{y}|\mathbf{z})p(\mathbf{z})} \\
    &= \frac{p(\mathbf{y}|\mathbf{x},\mathbf{z})p(\mathbf{x}|\mathbf{z})}{p(\mathbf{y}|\mathbf{z})}
    \end{aligned}
$$

## woodbury公式

$$
(\mathbf{A} + \mathbf{U}\mathbf{C}\mathbf{V})^{-1} = \mathbf{A}^{-1} - \mathbf{A}^{-1}\mathbf{U}(\mathbf{C}^{-1} + \mathbf{V}\mathbf{A}^{-1}\mathbf{U})^{-1}\mathbf{V}\mathbf{A}^{-1}
$$

观察公式，可以发现这个公式的优点是保证 $\mathbf{U}$ 和 $\mathbf{V}$ 不变的基础上，让公式变成只含有 $\mathbf{A}^{-1}$ 和 $\mathbf{C}^{-1}$。

证明过程如下：

令 $\mathbf{M= A + UCV} $

因为：A可逆，我们有：

$$
    \mathbf{MA^{-1} = I + UCV A^{-1}}
$$

两边右乘 $\mathbf{U}$ ,有

$$
    \mathbf{MA^{-1}U = U + UCV A^{-1}U}
$$

提出 $\mathbf{UC}$ 项，有

$$
    \mathbf{MA^{-1}U = UC(C^{-1} + VA^{-1}U)}
$$

因为有 $\mathbf{C^{-1} + VA^{-1}U}$ 可逆

$$
    \mathbf{MA^{-1}U(C^{-1} + VA^{-1}U)^{-1} = UC}
$$

把这里的 $\mathbf{UC}$ 配成$\mathbf{M} $的形式，有

$$
    \mathbf{MA^{-1}U(C^{-1} + VA^{-1}U)^{-1}V + A = UCV + A = M}
$$

所以有

$$
    \mathbf{M = MA^{-1}U(C^{-1} + VA^{-1}U)^{-1}V + A}
$$

$$
    \mathbf{I - A^{-1}U(C^{-1} + VA^{-1}U)^{-1}V = M^{-1}A}
$$

所以，

$$
    \mathbf{(A + UCV)^{-1} = A^{-1} - A^{-1}U(C^{-1} + VA^{-1}U)^{-1}V}
$$


# 假设
卡尔曼滤波依赖如下假设：
1. 马尔可夫性：当前观测只与当前状态有关，当前状态只与上一时刻状态和当前控制命令有关；
2. 所有状态和噪声都服从正态分布；
3. 运动、观测方程线性（非线性可以用EKF）。

# 卡尔曼滤波推导
对于线性系统的状态估计，我们可做如下建模：

$$
\begin{cases}
    \mathbf{x}_k = \mathbf{A}_k \mathbf{x}_{k-1} + \mathbf{B}_k \mathbf{u}_k + \mathbf{w}_k \\ 
    \mathbf{z}_k = \mathbf{C}_k \mathbf{x}_k + \mathbf{v}_k
\end{cases}
$$

其中，$\mathbf{x}_k$为$k$时刻的系统状态，$\mathbf{u}_k$ 为控制量，$\mathbf{z}_k$ 为观测量，$\mathbf{A}_k$、$\mathbf{B}_k$、$\mathbf{C}_k$ 分别为当前时刻状态转移矩阵、控制矩阵、测量矩阵。$\mathbf{w}_k \sim N(\mathbf{0},\mathbf{Q})$、$\mathbf{v}_k \sim N(\mathbf{0},\mathbf{R})$ 分别为过程噪声和观测噪声。

$\hat{\mathbf{x}}_k$ 表示状态**后验**，$\bar{\mathbf{x}}$ 表示状态**先验**，他们的协方差分别为 $\hat{\mathbf{P}}_k$ 、$\bar{\mathbf{P}}_k$。

由于噪声服从正态分布，概率先验有：

$$
\begin{cases} 
    \bar{\mathbf{x}}_k = \mathbf{A}_k \hat{\mathbf{x}}_{k-1} + \mathbf{B}_k \mathbf{u}_k, \\ 
    \bar{\mathbf{P}}_k = \mathbf{A}_k \hat{\mathbf{P}}_{k-1} \mathbf{A}^T_k + \mathbf{Q}
\end{cases}
$$

我们的需求是通过 $k-1$ 时刻状态、$k$ 时刻观测和控制求 $k$ 时刻状态，写成概率的形式即

$$
P \left( \mathbf{x}_{k} \, \middle| \, \mathbf{z}_{k}, \mathbf{x}_{k-1}, \mathbf{u}_{k} \right)
$$

利用贝叶斯公式：

$$
\begin{aligned}
    P \left( \mathbf{x}_k \mid \mathbf{z}_k, \mathbf{x}_{k-1}, \mathbf{u}_k \right) 
    &= \dfrac{P\left(\mathbf{z}_k, \mathbf{x}_k, \mathbf{x}_{k-1}, \mathbf{u}_k\right)}{P\left(\mathbf{z}_k, \mathbf{x}_{k-1}, \mathbf{u}_k\right)} \\[10pt]
    &= \dfrac{P\left(\mathbf{z}_k \mid \mathbf{x}_k, \mathbf{x}_{k-1}, \mathbf{u}_k\right) P\left(\mathbf{x}_k, \mathbf{x}_{k-1}, \mathbf{u}_k\right)}{P\left(\mathbf{z}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k\right) P\left(\mathbf{x}_{k-1}, \mathbf{u}_k\right)} \\[10pt]
    &= \dfrac{P\left(\mathbf{z}_k \mid \mathbf{x}_k\right) P\left(\mathbf{x}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k\right) P\left(\mathbf{x}_{k-1}, \mathbf{u}_k\right)}{P\left(\mathbf{z}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k\right) P\left(\mathbf{x}_{k-1}, \mathbf{u}_k\right)} \\[10pt]
    &= \dfrac{P\left(\mathbf{z}_k \mid \mathbf{x}_k, \mathbf{x}_{k-1}, \mathbf{u}_k\right)P\left(\mathbf{x}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k\right)}{P\left(\mathbf{z}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k\right)}
\end{aligned}
$$

由假设1可知观测 $\mathbf{z}_k$ 只与当前状态 $\mathbf{x}_k$ 有关，因此有

$$
\begin{cases}
    P\left(\mathbf{z}_k \mid \mathbf{x}_k, \mathbf{x}_{k-1}, \mathbf{u}_k\right)=P\left(\mathbf{z}_k \mid \mathbf{x}_k\right)\\
    P\left(\mathbf{z}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k\right)=P\left(\mathbf{z}_k\right)
\end{cases}
$$

$P\left(\mathbf{z}_k\right)$ 为定值，不妨设为1，因此有

$$
    P \left( \mathbf{x}_k \mid \mathbf{z}_k, \mathbf{x}_{k-1}, \mathbf{u}_k \right)=P\left(\mathbf{z}_k \mid \mathbf{x}_k\right)P\left(\mathbf{x}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k\right)
$$

这个公式也是贝叶斯估计的公式。

根据假设2，状态服从正态分布，记 $k$ 时刻状态 $\mathbf{x}_k$ 服从以下分布：

$$
\mathbf{x}_k \sim N(\hat{\mathbf{x}}_k, \hat{\mathbf{P}}_k)
$$

根据上面推导的贝叶斯估计公式，有

$$
N(\hat{\mathbf{x}}_k, \hat{\mathbf{P}}_k)=N(\mathbf{C}_k\mathbf{x}_k, \mathbf{R})N(\bar{\mathbf{x}}_k, \bar{\mathbf{P}}_k)
$$

分别代入高斯分布的概率密度函数（要注意观测概率为 $\mathbf{z}_k \sim N(\mathbf{C}_k\mathbf{x}_k, \mathbf{R})$ ）

$$
p(\mathbf{x} \mid \mathbf{\mu}, \mathbf{\Sigma}) = \frac{1}{(2\pi)^{n/2} |\mathbf{\Sigma}|^{1/2}} \exp \left( -\frac{1}{2} (\mathbf{x} - \mathbf{\mu})^T \mathbf{\Sigma}^{-1} (\mathbf{x} - \mathbf{\mu}) \right)
$$

忽视前面系数，只看指数部分的次方有

$$
(\mathbf{x}_k - \hat{\mathbf{x}}_k)^T \hat{\mathbf{P}}_k^{-1} (\mathbf{x}_k - \hat{\mathbf{x}}_k) = 
(\mathbf{z}_k - \mathbf{C}_k \mathbf{x}_k)^T \mathbf{R}^{-1} (\mathbf{z}_k - \mathbf{C}_k \mathbf{x}_k) + 
(\mathbf{x}_k - \bar{\mathbf{x}}_k)^T \bar{\mathbf{P}}_k^{-1} (\mathbf{x}_k - \bar{\mathbf{x}}_k)
$$

这个就叫公式base吧。展开公式base并分别比较一次、二次项系数。对于二次项 ${\mathbf{x}_k}^T\mathbf{x}_k$ 系数有

$$
\hat{\mathbf{P}}_k^{-1} = \mathbf{C}_k^T \mathbf{R}^{-1} \mathbf{C}_k + \bar{\mathbf{P}}_k^{-1}
$$

左右同乘 $\hat{\mathbf{P}}_k$ ，有

$$
\mathbf{I} = \hat{\mathbf{P}}_k \mathbf{C}_k^T \mathbf{Q}^{-1} \mathbf{C}_k + \hat{\mathbf{P}}_k \bar{\mathbf{P}}_k^{-1}
$$

这是个比较重要的中间公式，后面还会用，就取个名字吧，叫公式importance。可以推导出 $\mathbf{\hat{P}}_k$ 的解为

$$
\mathbf{\hat{P}}_k=(\mathbf{\bar{P}}_k^{-1}+\mathbf{C}_k^T\mathbf{R}^{-1}\mathbf{C}_k)^{-1}
$$

理论上用这种形式就可以直接更新状态的协方差矩阵了，但是对 $\mathbf{\bar{P}}_k$ 求逆维度很大，且在状态空间中求逆会放大误差。因此，需要将 $\mathbf{\bar{P}_k}^{-1}$ 转化为 $\mathbf{\bar{P}_k}$。有Woodbury公式，有

$$
\begin{aligned}
    \mathbf{\hat{P}_k}&=\mathbf{\bar{P}_k}-\bar{\mathbf{P}}_k \mathbf{C}_k^T (\mathbf{C}_k \bar{\mathbf{P}}_k \mathbf{C}_k^T + \mathbf{R})^{-1}\mathbf{C}_k \bar{\mathbf{P}}_k \\
    &=(\mathbf{I} - \bar{\mathbf{P}}_k \mathbf{C}_k^T (\mathbf{C}_k \bar{\mathbf{P}}_k \mathbf{C}_k^T + \mathbf{R})^{-1}\mathbf{C}_k )\mathbf{\bar{P}_k}
\end{aligned}
$$

定义**卡尔曼增益**

$$
\mathbf{K} = \bar{\mathbf{P}}_k \mathbf{C}_k^T (\mathbf{C}_k \bar{\mathbf{P}}_k \mathbf{C}_k^T + \mathbf{R})^{-1}
$$

有

$$
\hat{\mathbf{P}}_k = (\mathbf{I} - \mathbf{K} \mathbf{C}_k) \bar{\mathbf{P}}_k
$$

至此，我们求出了 $k$ 时刻状态 $\mathbf{x}_k$ 的协方差矩阵 $\mathbf{\hat{P}}_k$ 的更新公式，在推导 $\mathbf{x}_k$ 的更新公式之前，我们先给出卡尔曼增益的另一种形式。

由公式importance我们还可以推出

$$
\hat{\mathbf{P}}_k = (\mathbf{I} - \hat{\mathbf{P}}_k \mathbf{C}_k^T \mathbf{R}^{-1} \mathbf{C}_k) \bar{\mathbf{P}}_k
$$

与上面 $\mathbf{\hat{P}}_k$ 的更新公式对比，可以给出卡尔曼增益的另一种形式

$$
\mathbf{K} = \hat{\mathbf{P}}_k \mathbf{C}_k^T \mathbf{R}^{-1}
$$

下面推导 $\mathbf{x}_k$ 的更新公式。展开公式base，并比较一次项系数 $\mathbf{x}_k$ ，有

$$
-2 \hat{\mathbf{x}}_k^T \hat{\mathbf{P}}_k^{-1} \mathbf{x}_k = 
-2 \mathbf{z}_k^T \mathbf{R}^{-1} \mathbf{C}_k \mathbf{x}_k - 
2 \bar{\mathbf{x}}_k^T \bar{\mathbf{P}}_k^{-1} \mathbf{x}_k
$$

消去系数并转置，得

$$
\hat{\mathbf{P}}_k^{-1} \hat{\mathbf{x}}_k = \mathbf{C}_k^T \mathbf{R}^{-1} \mathbf{z}_k + \bar{\mathbf{P}}_k^{-1} \bar{\mathbf{x}}_k
$$

（由于 $\hat{\mathbf{P}}_k$ 正定，因此对称，因此转置为本身）

两边同时左乘 $\hat{\mathbf{P}}_k$ ，有

$$
\hat{\mathbf{x}}_k = \hat{\mathbf{P}}_k \mathbf{C}_k^T \mathbf{R}^{-1} \mathbf{z}_k + \hat{\mathbf{P}}_k \bar{\mathbf{P}}_k^{-1} \bar{\mathbf{x}}_k
$$

代入 $\hat{\mathbf{P}}_k$ 和卡尔曼增益 $\mathbf{K}$ 的另一种形式有

$$
\hat{\mathbf{x}}_k = \bar{\mathbf{x}}_k + \mathbf{K} (\mathbf{z}_k - \mathbf{C}_k \bar{\mathbf{x}}_k)\\
$$

至此我们推导了 状态量 $\mathbf{x}_k$ 和其协方差矩阵 $\mathbf{\hat{P}}_k$ 的更新公式。

# 卡尔曼滤波最终公式
预测：

$$
\begin{aligned}
    \bar{\mathbf{x}}_k &= \mathbf{A}_k \hat{\mathbf{x}}_{k-1} + \mathbf{u}_k, \\
    \bar{\mathbf{P}}_k &= \mathbf{A}_k \hat{\mathbf{P}}_{k-1} \mathbf{A}_k^T + \mathbf{Q}
\end{aligned}
$$

卡尔曼增益：

$$
\mathbf{K} = \bar{\mathbf{P}}_k \mathbf{C}_k^T (\mathbf{C}_k \bar{\mathbf{P}}_k \mathbf{C}_k^T + \mathbf{R})^{-1}
$$

更新：

$$
\begin{aligned}
\hat{\mathbf{x}}_k &= \bar{\mathbf{x}}_k + \mathbf{K} (\mathbf{z}_k - \mathbf{C}_k \bar{\mathbf{x}}_k), \\
\hat{\mathbf{P}}_k &= (\mathbf{I} - \mathbf{K} \mathbf{C}_k) \bar{\mathbf{P}}_k.
\end{aligned}
$$