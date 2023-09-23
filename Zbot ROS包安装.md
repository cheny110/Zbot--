# Zbot3 ROS包安装

**前提条件： 安装完ROS Noetic 相关基础包**

*安装路径以通用安装目录 **/opt/ros/** 为例，该目录在安装**ros-noetic-desktop-full**后会生成*

1.将如下工作空间源文件的压缩文件复制到/opt/ros目录下并解压

- catkin_ws.zip
- astra_ws.zip
- zbot3_ws.zip

```Bash
#替换path为压缩包所在路径
sudo cp /[path]/catkin_ws.zip /opt/ros/
sudo cp /[path]/astra_ws.zip /opt/ros/
sudo cp /[path]/zbot3_ws.zip /opt/ros/

cd /opt/ros
unzip -qr ./catkin_ws.zip
unzip -qr ./astra_ws.zip
unzip -qr ./zbot3_ws.zip
```

2.依次编译各工作空间并更新工作空间环境变量

```Bash
cd /opt/ros/catkin_ws
catkin_make
source ./devel/setup.bash
cd ../astra_ws
catkin_make
source ./devel/setup.bash
cd ../zbot3_ws
catkin_make
echo "source /opt/ros/zbot3_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

3.设置串口别名和访问权限

在Linux系统中，外接设备都要设置访问权限才能读写里面的数据。此外，对于接入的USB设备，都会按照接入顺序命名。比如接入两个USB设备，会按接入顺序命名为 /dev/ttyUSB0 和 /dev/ttyUSB1。 如何下次两个设备的接入顺序变了，则设备名也会跟着改变。这不利于我们的程序运行，我们也不想每次运行前都要修改以下读写权限。按照如下方法为Zbot3所使用到的USB设备设置别名和读写权限

```Bash
#替换【path】为规则文件所在目录 规则文件扩展名为rules
sudo cp /[path]/*.rules /etc/udev.rules.d/
sudo service udev reload
sudo service udev restart
```

**重启电脑或重新插拔相关USB设备使得配置生效**