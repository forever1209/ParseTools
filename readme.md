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

