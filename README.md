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
cat /sys/class/net/enp0s31f6/address    
enp0s31f6 f8:75:a4:c6:f9:e2   

sudo vim /etc/sysconfig/ethercat  
找到这两行，改成你的实际 MAC：  
  MASTER0_DEVICE="enp0s31f6"  
  DEVICE_MODULES="generic"  
上面方法失败了使用下面的：  
配置 IgH 使用 enp0s31f6  
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

运行调试  
实时权限修复  
sudo groupadd realtime  
sudo usermod -aG realtime $USER  
然后创建实时权限配置：  
sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null <<EOF  
@realtime soft rtprio 99  
@realtime hard rtprio 99  
@realtime soft memlock unlimited  
@realtime hard memlock unlimited  
EOF  

假设实际网卡是 enp0s31f6，执行：  

sudo nmcli dev set enp0s31f6 managed no  
sudo ip link set enp0s31f6 up  

然后关掉网卡省电和 offload：  

sudo ethtool --set-eee enp0s31f6 eee off  

sudo ethtool -K enp0s31f6  
  tso off  
  gso off  
  gro off  
  lro off  
如果支持固定速率，设置为 100M 全双工：
sudo ethtool -s enp0s31f6 speed 100 duplex full autoneg off

然后再看：

sudo dmesg -wT | grep -Ei "EtherCAT|Link state|TIMED OUT|UNMATCHED|SKIPPED|Synchronization"

如果还出现：

Link state of ecm0 changed to DOWN

优先换网线、换 EtherCAT 首站端口、换电脑网卡，尤其不要用 USB 网卡。

查询ethercat状态  
watch -n 0.2 "ethercat slaves"  
查询ethercat日志  
sudo dmesg -wT | grep -Ei "EtherCAT|Link state|TIMED OUT|UNMATCHED|SKIPPED|Synchronization"  

