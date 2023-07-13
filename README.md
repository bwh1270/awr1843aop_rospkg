# awr1843aop

※ This package is customized for the Ti mmwave awr1843aop module based on radar-lab/ti_mmwave_rospkg


1. Connect the device(radar sensor) to the Computer or Onboard computer which is operated in Ubuntu.

 
2. Set ttyusb speed to desired speed (baudrate)
```bash
 $ stty speed 115200 < /dev/ttyUSB0    # user port (config port)
 $ stty speed 921600 < /dev/ttyUSB1    # data port
 ```


3. Git clone in your workspace/src/ because these are the packages.
```bash
$ git clone https://github.com/bwh1270/awr1843aop.git
$ git clone https://github.com/wjwwood/serial.git
```

※ Customed message: radarPoints.msg
```bash
std_msgs/Header header
std_msgs/UInt16 total_points
std_msgs/UInt16MultiArray point_idx
geometry_msgs/Point[] points
std_msgs/Float32MultiArray velocity
std_msgs/FLoat32MultiArray snr
```
