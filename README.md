# 简介
运行于小米CyberDog，基于Nvidia [Argus](https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html)和[Ros2](https://www.ros.org/)的相机程序。

该程序使用Argus API提供的接口来操控MIPI相机硬件并实时捕获图像，使用ROS2提供的接口来管理相机节点，为外部模块提供交互接口。

# 编译
编译本模块需要依赖若干外部软件包，编译前需按照下列命令安装：

## 依赖项
### 1.nvidia-l4t-jetson-multimedia-api
```console
sudo apt-get install nvidia-l4t-jetson-multimedia-api
```
### 2.cuda-toolkit
```console
sudo apt-get install cuda-toolkit-10-2
```
### 3.libavformat-dev
```console
sudo apt-get install libavformat-dev
```

# 测试程序
基于相机api的相机测试程序，可以用来测试相机是否正常，亦可以作为camera api使用方式参考。

## 编译
```console
colcon build --merge-install --package-up-to camera_test
```

## 运行
```console
./build/camera_test/camera_test cam_id width height rgb/bgr
```
