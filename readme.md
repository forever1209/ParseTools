# 标定参数解析工具使用教程
## 一：编译工程
### 1.1 编译前准备
安装Cython
```shell
pip install Cython -i https://pypi.tuna.tsinghua.edu.cn/simple
```
### 1.2 编译指令
```shell
bash build.sh -b 12
```
如果报错python3.8找不到则需要在容器外编译或者选择x86编译容器编译（有python3.8的容器）
## 二：自动生成
### 2.1 aws配置
```shell
aws configure 
AWS Access Key ID [None]:输入Access Key
AWS Secret Access Key [None]:输入Secret key
```
### 2.2 根据模型发布路径、pkl路径、参数路径运行程序
以如下指令为例子：
```shell
bash Bin/scripts/update.sh  -v car602 -d   /home/liulei/deva/config/car602 -m  lane-det/share/20240527hs/ -p camera-perceptron/resources/calib/fovs_intrinsic_k_dict_tmp.pkl -t HS
```