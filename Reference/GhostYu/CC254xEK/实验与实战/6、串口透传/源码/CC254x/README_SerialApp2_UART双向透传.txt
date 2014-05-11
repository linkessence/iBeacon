SerialApp2_UART双向透传.rar
	UART双向透传程序
注意，请讲该压缩包解压得到的三个文件夹复制到协议栈源码的project/ble目录下，并且替换原先文件。
另外主机和从机的uart设置均为：57600,8,N,1

该串口透传程序是基于SimpleBLECentral主机和SimpleBLEPeripheral从机（协议栈自带的主从机demo）的基础上开发。
在进行串口收发之前，请根据ble入门指南中的操作将主机与从机通过五向按键连接，连接后便可通过uart收发数据

当主机与从机通过五向按键连接之后即可完成双向串口透传
1、主机到从机
uart数据转入--》SimpleBLECentral主机 ------蓝牙协议----- SimpleBLEPeripheral从机--》uart数据转出

2、从机到主机
uart数据转入--》SimpleBLEPeripheral从机 ------蓝牙协议----- SimpleBLECentral主机--》uart数据转出
