# ROS2进程内（intra-process）快速数据包录制

包括一个录制节点（recorder_node）和一个数据包格式转换节点（converter_node）。

* 录制节点（recorder_node）：将消息录制成自定义的二进制数据包，需要使用ros2 component启动。

* 转换节点（converter_node）：将自定义二进制数据包转换成标准的ROS2数据包。

## 节点参数

* 录制节点（recorder_node）：

    * output: 保存的自定义数据包文件名

    * topics: 一个字符串列表，表示所有需要录制的消息名

    * dummy: 布尔变量，当为true时，不进行数据包文件写入，通常用于调试（默认值：false）

    * quiet: 布尔变量，当为true时，不打印消息的实时录制帧率（默认值：false）

* 转换节点（converter_node）不使用ros param传递参数，而是使用命令行参数。

    * 使用方法：`converter_node <custom bag file> <standard ros2 bag file>`

    * 第一个参数`<custom bag file>`为录制节点生成的数据包文件路径
    
    * 第二个参数`<standard ros2 bag file>`为输出的标准ROS2数据包文件路径
