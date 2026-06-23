# guohua_ws  


###启动mtc抓取程序  
ros2 launch zhishu_robot_mtc_pick_place mtc_demos.launch.py  


ros2 launch zhishu_robot_hardware driver.launch.py slave_position:=2

ros2 topic pub --once /drive3/relative_deg std_msgs/msg/Float64 "{data: 90.0}"


igh主站-------------------------------------  
install igh master  
cd /opt  
sudo git clone https://gitlab.com/etherlab.org/ethercat.git  
sudo chown -R $USER:$USER ethercat  
cd ethercat  
git checkout stable-1.6  

./bootstrap  
sudo apt install -y autoconf automake libtool  

./configure --prefix=/usr/local/etherlab \  
  --disable-8139too \  
  --disable-eoe \  
  --enable-generic  

make all modules  
sudo make modules_install install  
sudo depmod  

sudo ln -sf /usr/local/etherlab/bin/ethercat /usr/bin/ethercat  
sudo ln -sf /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat  

sudo mkdir -p /etc/sysconfig  
sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat  

配置 EtherCAT 网卡  
ip -br link  
找出连接 EtherCAT 电机驱动器的那个网口 MAC，例如：    
cat /sys/class/net/enP1p1s0/address    
enp0s31f6 f8:75:a4:c6:f9:e2   

sudo vim /etc/sysconfig/ethercat  
找到这两行，改成你的实际 MAC：  
  MASTER0_DEVICE="enP1p1s0"  
  DEVICE_MODULES="generic"  
上面方法失败了使用下面的：  
配置 IgH 使用 enP1p1s0  
执行：  
sudo sed -i 's|^MASTER0_DEVICE=.*|MASTER0_DEVICE="f8:75:a4:c6:f9:e2"|' /usr/local/etherlab/etc/sysconfig/ethercat  
sudo sed -i 's|^DEVICE_MODULES=.*|DEVICE_MODULES="generic"|' /usr/local/etherlab/etc/sysconfig/ethercat  
确认配置：  
grep -E "MASTER0_DEVICE|DEVICE_MODULES" /usr/local/etherlab/etc/sysconfig/ethercat  
应该看到：  
MASTER0_DEVICE="enP1p1s0"  
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