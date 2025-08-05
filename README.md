安装依赖库:
`pip install open3d`

启动所有节点:
`roslaunch lecture2 task.launch`

打开终端发目标点：
`rostopic pub /goal geometry_msgs/Point -1 -- 8.0 3.0 0.0`

增加包路径：打开~/.bashrc，增加`export PYTHONPATH=$PYTHONPATH:/home/$USER/catkin_ws/src/lecture2/scripts`

