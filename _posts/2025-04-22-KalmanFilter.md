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
p(\bm{x} \mid \bm{\mu}, \bm{\Sigma}) = \frac{1}{(2\pi)^{n/2} |\bm{\Sigma}|^{1/2}} \exp \left( -\frac{1}{2} (\bm{x} - \bm{\mu})^\top \bm{\Sigma}^{-1} (\bm{x} - \bm{\mu}) \right)
$$
## 贝叶斯公式
$$
p(\bm{x}, \bm{y}) = p(\bm{x} \mid \bm{y}) p(\bm{y}) = p(\bm{y} \mid \bm{x}) p(\bm{x})
$$
多条件下的贝叶斯公式：
$$
    \begin{aligned}
    P(\bm{x}|\bm{y},\bm{z}) 
    &= \frac{p(\bm{x},\bm{y},\bm{z})}{p(\bm{y},\bm{z})} \\
    &= \frac{p(\bm{y}|\bm{x},\bm{z})p(\bm{x},\bm{z})}{p(\bm{y}|\bm{z})p(\bm{z})} \\
    &= \frac{p(\bm{y}|\bm{x},\bm{z})p(\bm{x}|\bm{z})p(\bm{z})}{p(\bm{y}|\bm{z})p(\bm{z})} \\
    &= \frac{p(\bm{y}|\bm{x},\bm{z})p(\bm{x}|\bm{z})}{p(\bm{y}|\bm{z})}
    \end{aligned}
$$

## woodbury公式
\[
(\bm{A} + \bm{U}\bm{C}\bm{V})^{-1} = \bm{A}^{-1} - \bm{A}^{-1}\bm{U}(\bm{C}^{-1} + \bm{V}\bm{A}^{-1}\bm{U})^{-1}\bm{V}\bm{A}^{-1}
\]
观察公式，可以发现这个公式的优点是保证 $\bm{U}$ 和 $\bm{V}$ 不变的基础上，让公式变成只含有 $\bm{A}^{-1}$ 和 $\bm{C}^{-1}$。

证明过程如下：

令 $\bm{M= A + UCV} $
因为：A可逆，我们有：
$$
    \bm{MA^{-1} = I + UCV A^{-1}}
$$
两边右乘 $\bm{U}$ ,有
$$
    \bm{MA^{-1}U = U + UCV A^{-1}U}
$$
提出 $\bm{UC}$ 项，有
$$
    \bm{MA^{-1}U = UC(C^{-1} + VA^{-1}U)}
$$
因为有 $\bm{C^{-1} + VA^{-1}U}$ 可逆
$$
    \bm{MA^{-1}U(C^{-1} + VA^{-1}U)^{-1} = UC}
$$
把这里的 $\bm{UC}$ 配成$\bm{M} $的形式，有
$$
    \bm{MA^{-1}U(C^{-1} + VA^{-1}U)^{-1}V + A = UCV + A = M}
$$
所以有
$$
    \bm{M = MA^{-1}U(C^{-1} + VA^{-1}U)^{-1}V + A}
$$
$$
    \bm{I - A^{-1}U(C^{-1} + VA^{-1}U)^{-1}V = M^{-1}A}
$$
所以，
$$
    \bm{(A + UCV)^{-1} = A^{-1} - A^{-1}U(C^{-1} + VA^{-1}U)^{-1}V}
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
    \bm{x}_k = \bm{A}_k \bm{x}_{k-1} + \bm{B}_k \bm{u}_k + \bm{w}_k \\ 
    \bm{z}_k = \bm{C}_k \bm{x}_k + \bm{v}_k
\end{cases}
$$
其中，$\bm{x}_k$为$k$时刻的系统状态，$\bm{u}_k$ 为控制量，$\bm{z}_k$ 为观测量，$\bm{A}_k$、$\bm{B}_k$、$\bm{C}_k$ 分别为当前时刻状态转移矩阵、控制矩阵、测量矩阵。$\bm{w}_k \sim N(\bm{0},\bm{Q})$、$\bm{v}_k \sim N(\bm{0},\bm{R})$ 分别为过程噪声和观测噪声。

$\hat{\bm{x}}_k$ 表示状态**后验**，$\bar{\bm{x}}$ 表示状态**先验**，他们的协方差分别为 $\hat{\bm{P}}_k$ 、$\bar{\bm{P}}_k$。

由于噪声服从正态分布，概率先验有：
$$
\begin{cases} 
    \bar{\bm{x}}_k = \bm{A}_k \hat{\bm{x}}_{k-1} + \bm{B}_k \bm{u}_k, \\ 
    \bar{\bm{P}}_k = \bm{A}_k \hat{\bm{P}}_{k-1} \bm{A}^T_k + \bm{Q}
\end{cases}
$$

我们的需求是通过 $k-1$ 时刻状态、$k$ 时刻观测和控制求 $k$ 时刻状态，写成概率的形式即
$$
P \left( \bm{x}_{k} \, \middle| \, \bm{z}_{k}, \bm{x}_{k-1}, \bm{u}_{k} \right)
$$
利用贝叶斯公式：
$$
\begin{aligned}
    P \left( \bm{x}_k \mid \bm{z}_k, \bm{x}_{k-1}, \bm{u}_k \right) 
    &= \dfrac{P\left(\bm{z}_k, \bm{x}_k, \bm{x}_{k-1}, \bm{u}_k\right)}{P\left(\bm{z}_k, \bm{x}_{k-1}, \bm{u}_k\right)} \\[10pt]
    &= \dfrac{P\left(\bm{z}_k \mid \bm{x}_k, \bm{x}_{k-1}, \bm{u}_k\right) P\left(\bm{x}_k, \bm{x}_{k-1}, \bm{u}_k\right)}{P\left(\bm{z}_k \mid \bm{x}_{k-1}, \bm{u}_k\right) P\left(\bm{x}_{k-1}, \bm{u}_k\right)} \\[10pt]
    &= \dfrac{P\left(\bm{z}_k \mid \bm{x}_k\right) P\left(\bm{x}_k \mid \bm{x}_{k-1}, \bm{u}_k\right) P\left(\bm{x}_{k-1}, \bm{u}_k\right)}{P\left(\bm{z}_k \mid \bm{x}_{k-1}, \bm{u}_k\right) P\left(\bm{x}_{k-1}, \bm{u}_k\right)} \\[10pt]
    &= \dfrac{P\left(\bm{z}_k \mid \bm{x}_k, \bm{x}_{k-1}, \bm{u}_k\right)P\left(\bm{x}_k \mid \bm{x}_{k-1}, \bm{u}_k\right)}{P\left(\bm{z}_k \mid \bm{x}_{k-1}, \bm{u}_k\right)}
\end{aligned}
$$
由假设1可知观测 $\bm{z}_k$ 只与当前状态 $\bm{x}_k$ 有关，因此有
$$
\begin{cases}
    P\left(\bm{z}_k \mid \bm{x}_k, \bm{x}_{k-1}, \bm{u}_k\right)=P\left(\bm{z}_k \mid \bm{x}_k\right)\\
    P\left(\bm{z}_k \mid \bm{x}_{k-1}, \bm{u}_k\right)=P\left(\bm{z}_k\right)
\end{cases}
$$
$P\left(\bm{z}_k\right)$ 为定值，不妨设为1，因此有
$$
    P \left( \bm{x}_k \mid \bm{z}_k, \bm{x}_{k-1}, \bm{u}_k \right)=P\left(\bm{z}_k \mid \bm{x}_k\right)P\left(\bm{x}_k \mid \bm{x}_{k-1}, \bm{u}_k\right)
$$
这个公式也是贝叶斯估计的公式。

根据假设2，状态服从正态分布，记 $k$ 时刻状态 $\bm{x}_k$ 服从以下分布：
$$
\bm{x}_k \sim N(\hat{\bm{x}}_k, \hat{\bm{P}}_k)
$$
根据上面推导的贝叶斯估计公式，有
$$
N(\hat{\bm{x}}_k, \hat{\bm{P}}_k)=N(\bm{C}_k\bm{x}_k, \bm{R})N(\bar{\bm{x}}_k, \bar{\bm{P}}_k)
$$
分别代入高斯分布的概率密度函数（要注意观测概率为 $\bm{z}_k \sim N(\bm{C}_k\bm{x}_k, \bm{R})$ ）
$$
p(\bm{x} \mid \bm{\mu}, \bm{\Sigma}) = \frac{1}{(2\pi)^{n/2} |\bm{\Sigma}|^{1/2}} \exp \left( -\frac{1}{2} (\bm{x} - \bm{\mu})^T \bm{\Sigma}^{-1} (\bm{x} - \bm{\mu}) \right)
$$
忽视前面系数，只看指数部分的次方有
$$
(\bm{x}_k - \hat{\bm{x}}_k)^T \hat{\bm{P}}_k^{-1} (\bm{x}_k - \hat{\bm{x}}_k) = 
(\bm{z}_k - \bm{C}_k \bm{x}_k)^T \bm{R}^{-1} (\bm{z}_k - \bm{C}_k \bm{x}_k) + 
(\bm{x}_k - \bar{\bm{x}}_k)^T \bar{\bm{P}}_k^{-1} (\bm{x}_k - \bar{\bm{x}}_k)
$$
这个就叫公式base吧。展开公式base并分别比较一次、二次项系数。对于二次项 ${\bm{x}_k}^T\bm{x}_k$ 系数有
$$
\hat{\bm{P}}_k^{-1} = \bm{C}_k^T \bm{R}^{-1} \bm{C}_k + \bar{\bm{P}}_k^{-1}
$$
左右同乘 $\hat{\bm{P}}_k$ ，有
$$
\bm{I} = \hat{\bm{P}}_k \bm{C}_k^T \bm{Q}^{-1} \bm{C}_k + \hat{\bm{P}}_k \bar{\bm{P}}_k^{-1}
$$
这是个比较重要的中间公式，后面还会用，就取个名字吧，叫公式importance。可以推导出 $\bm{\hat{P}}_k$的解为
$$
\bm{\hat{P}}_k=(\bm{\bar{P}}_k^{-1}+\bm{C}_k^T\bm{R}^{-1}\bm{C}_k)^{-1}
$$
理论上用这种形式就可以直接更新状态的协方差矩阵了，但是对 $\bm{\bar{P}}_k$ 求逆维度很大，且在状态空间中求逆会放大误差。因此，需要将 $\bm{\bar{P}_k}^{-1}$ 转化为 $\bm{\bar{P}_k}$。有Woodbury公式，有
$$
\begin{aligned}
    \bm{\hat{P}_k}&=\bm{\bar{P}_k}-\bar{\bm{P}}_k \bm{C}_k^T (\bm{C}_k \bar{\bm{P}}_k \bm{C}_k^T + \bm{R})^{-1} \\
    &=\bm{\hat{P}_k}(\bm{I} - \bm{C}_k^T (\bm{C}_k \bar{\bm{P}}_k \bm{C}_k^T + \bm{R})^{-1})
\end{aligned}
$$
定义**卡尔曼增益**
$$
\bm{K} = \bar{\bm{P}}_k \bm{C}_k^T (\bm{C}_k \bar{\bm{P}}_k \bm{C}_k^T + \bm{R})^{-1}
$$
有
$$
\hat{\bm{P}}_k = (\bm{I} - \bm{K} \bm{C}_k) \bar{\bm{P}}_k
$$
至此，我们求出了 $k$ 时刻状态 $\bm{x}_k$ 的协方差矩阵 $\bm{\hat{P}}_k$ 的更新公式，在推导 $\bm{x}_k$ 的更新公式之前，我们先给出卡尔曼增益的另一种形式。
由公式importance我们还可以推出
$$
\hat{\bm{P}}_k = (\bm{I} - \hat{\bm{P}}_k \bm{C}_k^T \bm{R}^{-1} \bm{C}_k) \bar{\bm{P}}_k
$$
与上面 $\bm{\hat{P}}_k$ 的更新公式对比，可以给出卡尔曼增益的另一种形式
$$
\bm{K} = \hat{\bm{P}}_k \bm{C}_k^T \bm{R}^{-1}
$$

下面推导 $\bm{x}_k$ 的更新公式。展开公式base，并比较一次项系数 $\bm{x}_k$ ，有
$$
-2 \hat{\bm{x}}_k^T \hat{\bm{P}}_k^{-1} \bm{x}_k = 
-2 \bm{z}_k^T \bm{R}^{-1} \bm{C}_k \bm{x}_k - 
2 \bar{\bm{x}}_k^T \bar{\bm{P}}_k^{-1} \bm{x}_k
$$
消去系数并转置，得
$$
\hat{\bm{P}}_k^{-1} \hat{\bm{x}}_k = \bm{C}_k^T \bm{R}^{-1} \bm{z}_k + \bar{\bm{P}}_k^{-1} \bar{\bm{x}}_k
$$
（由于 $\hat{\bm{P}}_k$ 正定，因此对称，因此转置为本身）
两边同时左乘 $\hat{\bm{P}}_k$ ，有
$$
\hat{\bm{x}}_k = \hat{\bm{P}}_k \bm{C}_k^T \bm{R}^{-1} \bm{z}_k + \hat{\bm{P}}_k \bar{\bm{P}}_k^{-1} \bar{\bm{x}}_k
$$
代入 $\hat{\bm{P}}_k$ 和卡尔曼增益 $\bm{K}$ 的另一种形式有
$$
\hat{\bm{x}}_k = \bar{\bm{x}}_k + \bm{K} (\bm{z}_k - \bm{C}_k \bar{\bm{x}}_k)\\
$$
至此我们推导了 状态量 $\bm{x}_k$ 和其协方差矩阵 $\bm{\hat{P}}_k$ 的更新公式。

# 卡尔曼滤波最终公式
预测：
$$
\begin{aligned}
    \bar{\bm{x}}_k &= \bm{A}_k \hat{\bm{x}}_{k-1} + \bm{u}_k, \\
    \bar{\bm{P}}_k &= \bm{A}_k \hat{\bm{P}}_{k-1} \bm{A}_k^T + \bm{Q}
\end{aligned}
$$
卡尔曼增益：
$$
\bm{K} = \bar{\bm{P}}_k \bm{C}_k^T (\bm{C}_k \bar{\bm{P}}_k \bm{C}_k^T + \bm{R})^{-1}
$$
更新：
$$
\begin{aligned}
\hat{\bm{x}}_k &= \bar{\bm{x}}_k + \bm{K} (\bm{z}_k - \bm{C}_k \bar{\bm{x}}_k), \\
\hat{\bm{P}}_k &= (\bm{I} - \bm{K} \bm{C}_k) \bar{\bm{P}}_k.
\end{aligned}
$$