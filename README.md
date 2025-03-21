# IsaacSim_ObjectDetection_learning

# 關於此專案
設計一個草莓採摘機器人，能在真實環境採摘成熟草莓，為此，我們在isaac sim 平台建置虛擬環境，訓練與控制機器人學會以yolo影像辨識的方式到達目的地，之後將訓練好的決策模型部屬到草莓採摘機器人上。

# 執行結果

https://github.com/user-attachments/assets/b995aa7b-7520-4615-a35b-f873a9496640

# 快速開始

## 基本環境
依照Isaac sim 官方建議，此專案在以下硬體與軟體環境執行

* Ubuntu 20.04

* Cuda 12.6

* Ndivia GPU Driver 560.35+


## 套件
此專案使用以下套件，相關安裝連結如下:

[1. Isaac sim 2023.1.1](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/distributions.html)

[2. Ros2 Humble](https://docs.ros.org/en/humble/Installation.html)

[3. NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)

[4. Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

[5. yoloV12](https://github.com/sunsmarterjie/yolov12)


# 程式執行
* 依照[Isaac Ros官方教學](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)，建置docker環境，並啟動Isaac Ros docker環境

    ```
    cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
    ./scripts/run_dev.sh
    ```

使用Standalone模式下建立Isaac草莓模擬採摘環境的啟動與動態建立Action Graph

* Source Ws與Ros2
    ```
    . ~/ros2_humble/install/local_setup.bash
    source ~/ros2_humble/install/setup.bash

    # 設定fastdds.xml來Set the environment variable
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/IsaacSim-ros_workspaces/humble_ws/fastdds.xml

    # Source Ws設定
    source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
    ```

* 啟動模擬環境並發布Ros2影像
    ```
    // 切換到isaac 的安裝目錄
    cd ~/.local/share/ov/pkg/isaac-sim-2023.1.1 

    # 使用isaac的python執行檔 執行Standalone程式
    ./python.sh ${path_to_script}load_usd.py
    ```

* 啟動Ros2 影像訂閱節點(TODO : 補充怎麼用Ros2建置pkg)

    ```
    cd ~/ros2_ws
    colcon build --packages-select image_publisher_pkg image_subscriber_pkg 
    source install/setup.bash
    ros2 run image_publisher_pkg image_publisher
    ```

* Rviz2觀察結果

    ```
    rviz2
    ```

# Reference
[Reference.md](Reference.md)
# Maintainers
[@Johnny-Hu-406](https://github.com/Johnny-Hu-406?tab=repositories)

# License
MIT [@Johnny-Hu-406](https://github.com/Johnny-Hu-406?tab=repositories)
