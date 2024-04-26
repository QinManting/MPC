# UAV-px4ctrl
## Overview
该仓库为 nROS-lab 2023届硕士生参加2023RMUA比赛所用MPC控制器代码。

### File Layouts
**controller**：包含MPC控制器代码代码  
**pseudo_path**：用于将多项式轨迹转换为一系列离散的路点  
**quadrotor_msgs**：用于定义MPC向px4ctrl发送的控制指令的消息类型

## Installation
### Dependencies
- [ROS-Melodic](http://wiki.ros.org)
- [Eigen](https://eigen.tuxfamily.org/)版本3.4.0,否则cpp文件编译不通过
- [osqp](https://osqp.org/)
- [osqp-Eigen] （要与osqp版本相对应，参考osqp v0.8.0对应osqp-eigen v0.6.3）

### Building
To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone ...
        cd ../
        catkin_make

## Run
**启动pseudo_path**  

        roslaunch pseudo_path pseudo_path.launch  

**启动controller**  
    
        roslaunch controller mpc_controller_sim.launch    

## Topic

### controller topic
**Sub**  
`/next_base_path` : 从psedo_path包接受控制器所要跟踪的离散轨迹点.  
`/mavros/local_position/odom` : 接受飞机的里程计信息.  
`/reference/polynomial` : 从planning中接收多项式轨迹，目前没有用到.  

**Pub**  
`/solution` : mpc每次优化求解后得到的轨迹.  
`/ref` : 输入给mpc的参考轨迹.  
`/history_path` : 无人机实际走过的历史轨迹.  
`/planning/pos_cmd` : 向px4ctrl发送控制指令.  
`/mpc_recv` : 发送bool类型消息,表示mpc是否在向飞控发送控制指令,确保mpc控制器和px4ctrl控制器仅有一个在向飞控发送控制指令.  

### pseudo_path topic  
**Sub**  
`/reference/polynomial` : 从planning中接收多项式轨迹.  
**Pub**  
`/next_base_path` : 从psedo_path包接受控制器所要跟踪的离散轨迹点.  

