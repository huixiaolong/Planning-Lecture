安装依赖库:
`pip install open3d`

启动所有节点:
`roslaunch lecture2 task.launch`

打开终端发目标点：
`rostopic pub /goal geometry_msgs/Point -1 -- 8.0 3.0 0.0`

