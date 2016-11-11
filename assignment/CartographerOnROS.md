# ROS下配置cartographer



## 配置ROS

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



---

## 配置cartographer

本次ROS下安装cartographer是在Ubuntu16.04 64位下进行的,在此之前已经安装好了**ROS Kinetic**.

### 编译和安装

谷歌官方推荐使用工具[wstool](http://wiki.ros.org/wstool)和[rosdep](http://wiki.ros.org/rosdep),编译工具推荐使用[Ninja](https://ninja-build.org/).

1. 安装依赖

   安装`wstool`和`rosdep`  以及`ninja`编译工具

   ```bash
   sudo apt-get update
   sudo apt-get install -y python-wstool python-rosdep ninja-build
   ```

2. 创建`catkin_ws`工作目录(这里是在`home`路径下,其他路径也可以):

   ```bash
   mkdir catkin_ws
   cd catkin_ws
   wstool init src
   ```

3. 配置各种依赖项

   ```bash
   # Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
   wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
   wstool update -t src

   # Install deb dependencies.
   rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
   ```

4. 编译安装

   ```bash
   catkin_make_isolated --install --use-ninja # 在运行时因为需要从谷歌下载代码,因此运行这一步需要翻墙环境.
   source install_isolated/setup.bash
   source devel_isolated/setup.bash
   rospack profile # 如果运行时出现其他提示,可以选择再运行一次, 加上参数--zombie-only
   ```

### 运行demo

安装好cartographer后就可以下载谷歌官方提供的测试数据(包括2D和3D)来运行了.

1. 下载2D数据:

   ```bash
   wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
   ```

2. 运行2D数据:

   ```bash
   roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
   ```

3. 运行结果:  

   ![cartographer_fig1](../image/cartographer_fig1.png)  

4. 下载3D数据:  

   ```bash
   wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/cartographer_3d_deutsches_museum.bag
   ```

5. 运行3D数据:

   ```bash
   roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/cartographer_3d_deutsches_museum.bag
   ```

6. 3D数据运行结果(由于电脑配置不行,因此跑的时间不长):  

   俯视图:  

   ![cartographer_fig5](../image/cartographer_fig5.png)  

   3d视角:  

   ![cartographer_fig6](../image/cartographer_fig6.png)  


---

### 关于Ubuntu下翻墙

本次实验配置在最后的`catkin_make_isolated`编译时需要从谷歌下载代码,在墙内环境下如果不翻墙的话会出下**fail**.这里使用shadowsocks翻墙.  

1. 安装**shadowsocks**,这里安装**qt5**客户端,能够自动转**http(s)**代理.

   ```bash
   sudo add-apt-repository ppa:hzwhuang/ss-qt5
   sudo apt-get update
   sudo apt-get install shadowsocks-qt5
   ```

2. 配置**shadowsocks**客户端:

   搜索`ss-qt5`,打开图形客户端，添加配置，示例如下,注意本地服务器类型需要选择`HTTP(S)`.  

    ![cartographer_fig3](../image/cartographer_fig3.png)  

3. 配置系统级代理,打开`系统设置`->`网络`,设置如下:   

   ![cartographer_fig4](../image/cartographer_fig4.png)  

> **注意**: 配置完代理后,终端要重启后代理才会生效.



**参考链接:**

- [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- [Cartographer ROS Integration(谷歌官方安装文档)](https://google-cartographer-ros.readthedocs.io/en/latest/)
- [shadowsock-qt5文档](https://github.com/shadowsocks/shadowsocks-qt5/wiki)  
