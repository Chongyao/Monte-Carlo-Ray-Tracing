# Ray Tracing算法　说明文档
##	结果展示
### VeachMIS
![VeachMIS](https://raw.githubusercontent.com/Chongyao/Monte-Carlo-Ray-Tracing/master/res/3.png)
### Treasure in Room
![room](https://raw.githubusercontent.com/Chongyao/Monte-Carlo-Ray-Tracing/master/res/1.png)
### Coffee
![cup](https://raw.githubusercontent.com/Chongyao/Monte-Carlo-Ray-Tracing/master/res/2.png)
## 开发环境
### 系统信息
系统 | 信息
------------ | ------------- 
操作系统 | Linux Mint 19 Cinnamon 
处理器 | Intel© Core™ i7-4790 CPU @ 3.60GHz × 4
内存　|　32GB
显卡　| GeForce GTX 750
编译器　| gcc 7.3
编译工具 |　Cmake 3.10.2
### 项目依赖库

矩阵库：Eigen3

并行库：Openmp(verison:4.5)

具体查看根目录下的CMakeLists.txt
## 编译运行
在满足依赖的情况下，进入根目录，开启终端运行：
```
$ mkdir build & cd build & cmake .. & make -k
```
使用Release模式编译
### 运行
开启终端进入build/bin/目录，并运行./MC
```bash
$ cd res
$ ../build/bin/MC ../scenes/Scene01/room.obj ../test/cam_room.txt
```
## 数据结构
### aabb类和tri_aabb类
这两个类用于构建bounding box。 aabb类用于给某一个kd_tree建立bound_box,tri_aabb用于给某一个面片存bounding box,并且计算它的平面参数。
### Scene类
读取场景中物体的位置、材料参数等。
### kd_tree_tris类
针对空间的三角形构建kd_tree，在实现中对每一个物体构建一个kd tree。
### Ray类
表示光线，并且内部集成了与kd_tree的求交算法。







