 - 创建工作空间
  
ros的工作空间可以创建在任何有权限的地方，此处我们已 /opt/ros/目录为例子，工作空间命名通常为 [空间名]_ws。(ws为workspace 简写)。创建工作空间就是创建对应的文件目录结构，最简单的为工作空间目录和存放功能包的src子目录。

```bash
cd /opt/ros #切换到目录
mkdir -p ./zbot_append_ws/src #在ros目录下创建 /zbot_append_ws/src层级目录
cd ./zbot_append_ws/src
catkin_init_workspace #初始化工作空间
cd ../ #返回上层目录

#创建zbot3_multinav功能包，依赖包为rospy roscpp std_msgs nav_msgs actionlib movebase_msgs
catkin_create_pkg zbot3_multinav rospy roscpp std_msgs nav_msgs actionlib movebase_msgs
#进行一次编译，导入必要文件

```