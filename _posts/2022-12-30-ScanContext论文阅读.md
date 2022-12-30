---
typora-copy-images-to: image
---

# Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map

扫描上下文：3D点云地图中用于位姿识别的中心空间描述符

# 摘要

本文提出了一种基于非直方图的雷达扫描全局描述符：Scan Context。


# 简介

提出新的空间描述符Scan Context用于匹配算法，主要针对户外使用单一的3D扫描来进行识别。我们的表示法将整个点云编码为一个矩阵。该表示法描述了以自身为中心的一个2.5D信息。，主要贡献点为：

高效的bin编码功能。本方法不需要统计一个bin中的点数，而是提出了一个更高效的bin编码函数来进行位置识别。这种编码对点云密度和法线具有不变性。

保存点云的内部结构，如图所示，矩阵的每个元素值仅由属于该bin的点决定。

有效的两相匹配算法

与其他最先进的空间描述符进行验证

# Scan Context用于位置识别

## A. Scan Context

将一个扫描点云以相同的间隔分为方位角和径向的容器（极坐标划分），扫描的中心作为全局关键点，因此我们把Scan Context称为以自身为中心的描述符。$N_s$和$N_r$是扇区和环的个数。令雷达的最大感知距离为$L_{max}$，则环之间的间距为$\frac{L_{max}}{N_r}$，扇区的角度等于$\frac{2\pi}{N_s}$。在这篇文章里，我们使用$N_s= 60$和$N_r= 20$。

![image-20220825144009873](E:\学习\mynote\文档\image\ScanContext论文阅读\image-20220825144009873.png)

因此，构造Scan Context的第一个步骤是将整个点云分为独立的点云。如上图所示。$P_{ij}$是属于第$i$环第$j$扇区域的点的集合。标志$[N_s]$等于$\{1,2,...,N_{s-1},N_s\}$。因此，数学上，这个划分可以表述为
$$
P = \bigcup_{i\in[N_r], j\in[N_s]}P_{ij}\tag{1}
$$
Scan Context弥补了远点的稀疏性导致的信息量不足，并将附近的动态对象视为稀疏噪声。

在对点云进行分区后，为每个区域分配一个实数值：
$$
P_{ij}\rightarrow \mathbb{R}\tag{2}
$$
这里使用最大高度。因此每个区域的编码函数为：
$$
\phi(P_{ij})=\max_{p\in P_{ij}} z(p)\tag{3}
$$
上式中，$z(·)$函数返回点p的z轴高度。对空的区域赋予0值。如下图所示，Scan Context中的蓝色意味着其对应的空间要么是空闲的，要么由于遮挡没有被观察到。

通过上述步骤，将一个Scan Context $I$ 最终可以表达为一个$N_r\times N_s$的矩阵：
$$
I=(a_{ij})\in\mathbb{R}^{N_r\times N_s},a_{ij}=\phi(P_{ij})\tag{4}
$$
为了实现相对于平移的鲁棒识别，我们通过根位移（root shifting）增强了Scan Context。这样可以从原始扫描中获得在轻微运动扰动下的各种Scan Context。在重访过程中，单个扫描上下文可能对平移运动下的扫描中心位置敏感。例如，当重新访问不同通道中的相同位置时，可能不会保留扫描上下文的行顺序。为了克服这种情况，我们根据通道间隔将原始点云转换为$N_{trans}$邻居（本文中使用的$N_{trans} = 8$），并将这些根移位点云获得的扫描上下文一起存储。我们假设即使在实际移动的位置也会得到一个类似的点云，这是有效的，除了一些情况，如交叉接入点，一个新的空间突然出现。

## B. 两个Scan Context之间的相似分数

给定一对Scan Context，我们需要一个距离来度量两个Scan Context的相似性。设$I^q$和$I^c$是分别从查询点云和候选点云中获取的Scan Context。按列的方式进行比较，也就是说距离是具有相同索引的列之间的距离之和。用一个余弦距离来计算两个列向量在相同序号处的距离$c_j^q$和$c_j^c$。最后，把总和除以列数$N_s$来归一化。因此，距离函数为：
$$
d(I^q,I^c)=\frac{1}{N_s}\sum_{j=1}^{N_s}\left( 1-\frac{c_j^q\cdot c_j^c}{\|c_j^q\|\|c_j^c\|} \right)\tag{5}
$$

> 余弦距离：用两向量夹角的余弦值来衡量两向量的相似度。
>
> 这里将Scan context每一列都当作一个向量，$\frac{c_j^q\cdot c_j^c}{\|c_j^q\|\|c_j^c\|}=\frac{\|c_j^q\|\|c_j^c\|\cos(\theta)}{\|c_j^q\|\|c_j^c\|}=\cos(\theta)$

如下图所示，激光雷达坐标发生变化，则列的顺序可能会不同。

![image-20220831170821120](E:\学习\mynote\文档\image\ScanContext论文阅读\image-20220831170821120.png)

因此，计算所有可能的列位移下的Scan Context距离，并找到最小距离。令$I_n^c$为原始Scan Context位移了$n$列后形成的新的Scan Context。确定达到最佳匹配的列数和距离。
$$
D(I^q,I^c)=\min_{n\in[N_s]}d(I^q,I_n^c),\tag{6}
$$

$$
n^*=\mathop{argmin}_{n\in[N_s]}d(I^q,I_n^c)\tag{7}
$$

这里这个额外的位移信息可以作为一个很好的初值，用于进一步的定位细化。

## C. 两阶段搜索方法

有三种方法可以用于位置识别的搜索：两两相似度评分、最近邻搜索和稀疏优化。本文将两两评分和最近邻搜索分层融合，以获得合理搜索时间。

定义环值（ring key），是从Scan Context中提取的旋转不变描述符。将Scan Context中的每一行r，通过环编码函数$\psi$编码为一个实数。向量$k$为以传感器为圆心的每个环的环值构成的向量，如下图所示。

![image-20220831173119146](E:\学习\mynote\文档\image\ScanContext论文阅读\image-20220831173119146.png)

因此，环值是一个$N_r$维向量：
$$
k=(\psi(r_1),...,\psi(r_{N_r})),\ where\ \psi:r_i\rightarrow\mathbb{R}\tag{8}
$$
环编码函数$\psi$是用$L_0$范数表示的占用率。
$$
\psi(r_i)=\frac{\|r_i\|_0}{N_s}\tag{9}
$$
由于占用率独立于视点，因此环值实现了旋转不变性。

用向量k构造KD-tree，用于查询相似的环值。查找得到若干相似的环值的k，比较Scan context。
$$
c^*=\mathop{argmin}_{c_k\in C}D(I^q,I^{c_k}),\ s.t\ D<\tau\tag{10}
$$
上式中，C是从KD-tree中查找得到的一组序号，$\tau$是给定的接受阈值。$c^*$是被判断维出现回环的索引。

![image-20220831184056239](E:\学习\mynote\文档\image\ScanContext论文阅读\image-20220831184056239.png)

# 实验

