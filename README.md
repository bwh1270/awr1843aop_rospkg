# awr1843aop
- - -

1. Connect the device(radar sensor) to the pc which is operated in Ubuntu.
2. Set ttyusb speed to desired speed (baudrate)
   <pre>
     <code>
       $ stty speed 115200 < /dev/ttyUSB0    # user port (config port)
       $ stty speed 921600 < /dev/ttyUSB1    # data port
     </code>
   </pre>
3. Git clone in your workspace/src/ because these are the packages.
  <pre>
    <code>
      $ git clone https://github.com/bwh1270/awr1843aop.git
      $ git clone https://github.com/wjwwood/serial.git
    </code>
  </pre>
