# xian_dj_robotgo1_ws

## 一、环境设置
- 鲁班猫-ros-neotic开发[镜像](https://pan.baidu.com/s/1BsL4arp2vJuptlet9uc3Ew),提取码: ugg2
- 载入镜像
    `sudo docker load -i xxx/sixrss-v1.tar`
- 容器 启动指令：
    ```
    # 若lucat断电重启，需要再鲁班猫本地上打开终端，输入如下指令：
    sudo docker stop sixrss
    sudo docker rm sixrss
    xhost +local:root

    sudo docker run --name sixrss -itd --privileged -p 2003:22 -p 3241:3241 --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /home/cat/tangle:/root/code --restart=always ros-noetic-ros-base-joy:sixrss-v1
    ```

- 进入容器：
    ```
    sudo docker exec -it sixrss /bin/bash
    udo docker exec -it sixrss /bin/bash -c "cd /root && exec /bin/bash" # enter /root directory directly
    ```
## 二、运行
### 2.1 编译
- `cd ~/code/xian_dj_robotgo1_ws`
- `catkin_make`
### 2.2 添加环境变量
- `source /root/code/xian_dj_robotgo1_ws/devel/setup.bash`


