# 配置ROS

本次实验配置在**Ubuntu 16.04 64位**环境下进行,安装**ROS Kinetic**.

### 配置Ubuntu源

进入系统设置`System Settings`->`Software & Updates`,勾选 `restricted`, `universe` 和`multiverse`.  

![ros_fig1](../image/ros_fig1.png)  

### 配置source.list

让ubuntu允许来自packages.ros.org的软件包.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 配置秘钥

```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
```

### 安装

ROS有许多不同的库以及工具,这里选择安装`Desktop-Full`

```bash
sudo apt-get update # 更新Debian package的索引,确保索引是最新的
sudo apt-get install ros-kinetic-desktop-full
```

### 初始化rosdep

使用ROS之前,需要出示话`rosdep`, 当我们在编译或者运行某些ROS的组件时,`rosdep`能够使得系统依赖的安装更为容易.  

```bash
sudo rosdep init
rosdep update
```

### 环境配置

每次启动shell时都把ROS的环境变量加入到会话中

```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 安装rosinstall工具

```bash
sudo apt-get install python-rosinstall
```



**参考链接:**

- [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)