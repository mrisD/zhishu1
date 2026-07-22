# guohua_ws  


###启动mtc抓取程序  
ros2 launch zhishu_robot_mtc_pick_place mtc_demos.launch.py  


ros2 launch zhishu_robot_hardware driver.launch.py slave_position:=2

ros2 topic pub --once /drive3/relative_deg std_msgs/msg/Float64 "{data: 90.0}"


igh主站(x86)-------------------------------------  
install igh master (x86 & arm64) 
cd /opt  
sudo git clone https://gitlab.com/etherlab.org/ethercat.git  
sudo chown -R $USER:$USER ethercat  
cd ethercat  
git checkout stable-1.6  

./bootstrap  
sudo apt install -y autoconf automake libtool  

./configure --prefix=/usr/local/etherlab --disable-8139too --disable-eoe --enable-generic 

make all modules  
sudo make modules_install install  
sudo depmod  

sudo ln -sf /usr/local/etherlab/bin/ethercat /usr/bin/ethercat  
sudo ln -sf /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat  

sudo mkdir -p /etc/sysconfig  
sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat  




配置 EtherCAT 网卡-------------------------------------  
ip -br link  
找出连接 EtherCAT 电机驱动器的那个网口 MAC，例如：    
cat /sys/class/net/enp0s31f6/address    
格式如下
enp0s31f6 f8:75:a4:c6:f9:e2   
enP1p1s0 a0:28:33:80:56:21 左1口
enx00e04c476648 00:e0:4c:47:66:48 USB

sudo vim /etc/sysconfig/ethercat  
找到这两行，改成你的实际 MAC：  
  MASTER0_DEVICE="enp0s31f6"（左手臂网卡）  
  MASTER1_DEVICE="enp0s31f5"（右手臂网卡）     
  DEVICE_MODULES="generic"  

上面方法如果不生效使用下面的方法：  
配置 IgH 使用 enp0s31f6  
执行：  
sudo sed -i 's|^MASTER0_DEVICE=.*|MASTER0_DEVICE="enp0s31f6"|' /usr/local/etherlab/etc/sysconfig/ethercat  
sudo sed -i 's|^MASTER1_DEVICE=.*|MASTER1_DEVICE="enp0s31f5"|' /usr/local/etherlab/etc/sysconfig/ethercat  
sudo sed -i 's|^DEVICE_MODULES=.*|DEVICE_MODULES="generic"|' /usr/local/etherlab/etc/sysconfig/ethercat  

确认配置：   
grep -nE '^[[:space:]]*(MASTER[0-9]+_(DEVICE|BACKUP)|DEVICE_MODULES)=' \
  /usr/local/etherlab/etc/sysconfig/ethercat
应该看到：  
MASTER0_DEVICE="enP1p1s0"  
MASTER1_DEVICE="enP1p1s1"  
DEVICE_MODULES="generic"  


添加 udev 权限规则  
sudo vim /etc/udev/rules.d/99-EtherCAT.rules  
写入：  
KERNEL=="EtherCAT[0-9]*", MODE="0666"  
然后刷新：  
sudo udevadm control --reload-rules  
sudo udevadm trigger  

启动 IgH EtherCAT Master  
sudo /etc/init.d/ethercat start  
检查状态：  
sudo /etc/init.d/ethercat status  
ls /dev/EtherCAT*  

运行调试  
实时权限修复  
sudo groupadd realtime  
sudo usermod -aG realtime $USER  
然后创建实时权限配置（注意去掉后面的空格）：  
sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null <<EOF  
@realtime soft rtprio 99  
@realtime hard rtprio 99  
@realtime soft memlock unlimited  
@realtime hard memlock unlimited  
EOF  


查询ethercat状态  
watch -n 0.2 "ethercat slaves"  
查询ethercat日志  
sudo dmesg -C
sudo dmesg -wT | grep -Ei "EtherCAT|Link state|TIMED OUT|UNMATCHED|SKIPPED|Synchronization"  

更换DDS  
sudo apt install ros-humble-rmw-cyclonedds-cpp  
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  
gedit ~/.bashrc  
  在调用ros2前加上  
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
保存退出重新打开终端  
查询版本 echo $RMW_IMPLEMENTATION

安装依赖  
首先安装rosdep工具  
wget http://fishros.com/install -O fishros && . fishros  
  [3]:一键安装:rosdep(小鱼的rosdepc,又快又好用)  
安装完毕后执行
  rosdep update


在src同级目录下执行 
cd ~/guohua_ws  
 
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble --skip-keys "gazebo_ros ros_gz ros_gz_sim gz_ros2_control gz_ros2_control_demos"  



Gazobo仿真相关配置
export IGN_GAZEBO_RESOURCE_PATH=/home/zhul/online/guohua_ws/src/zhishu_robot_gazebo/models

echo "alias robot='bash /home/zhul/online/guohua_ws/src/zhishu_robot_bringup/scripts/zhishu_robot_gazebo.sh'" >> ~/.bashrc && source ~/.bashrc


性能测试  
stress-ng --cpu 0 --io 4 --vm 2 --vm-bytes 512M --timeout 120s  
sudo cyclictest -S -p95 -m -i1000 -l100000 -q  
时实性检测  
cat /sys/kernel/realtime 