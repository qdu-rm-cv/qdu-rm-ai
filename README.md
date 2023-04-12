# 青岛大学 RoboMaster 视觉 人工智能 代码开源

***Developing.***

[Gitee](https://gitee.com/qdu-rm-2022/qdu-rm-ai)
[Github](https://github.com/qdu-rm-cv/qdu-rm-ai)

软件正处在开发初期，只完成了视觉的核心部分，其余部分正在开发中。

## 软件介绍

本开源软件为青岛大学未来战队机器人的视觉和人工智能的代码。参考了其他战队代码和各种开源机器人项目，从零编写而成。中心思想：

- 基于OpenCV的识别算法
- 基于行为树设计哨兵的AI
- 一个项目适配不同型号的机器人

这样做增加代码了的重用，减少了工作量。实现了通过DLA(深度学习加速器)加速MiniPC上模型的推断速度。利用行为树实现了可控的复杂行为。

## 图片展示

| ![YOLO识别效果](./assets/image/test_yolo.jpg?raw=true "YOLO识别效果") |
| :-------------------------------------------------------------------: |
|                            *YOLO识别效果*                             |

| ![装甲板匹配效果](./assets/README.assets/test_origin.png?raw=true "装甲板匹配效果") |
| :-------------------------------------------------------------------------: |
|                              *本算法识别效果*                               |

| ![TensorRT加速效果](./assets/image/compare.jpg?raw=true "TensorRT加速效果") |
| :-------------------------------------------------------------------------: |
|                          *TODO：TensorRT加速效果*                           |

## 依赖 & 环境

- 依赖
  - [OpenCV (4.5.0 - 4.5.4)](https://docs.opencv.org/4.5.4/d7/d9f/tutorial_linux_install.html)
  - [BehavoirTree.CPP (3.3.0 - 3.8.1)](https://github.com/BehaviorTree/BehaviorTree.CPP)
  - [MVS SDK from HIKROBOT](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)
  - [spdlog (1.9.2)](https://github.com/gabime/spdlog)
  - [Google Test (1.8.0 - 1.10.0)](https://github.com/google/googletest)
  - [oneTBB (only tested 2020.1)](https://github.com/oneapi-src/oneTBB) or `libtbb-dev`
  - [libusbp](https://github.com/pololu/libusbp)
  - [Eigen (highest 3.3.7)](https://eigen.tuxfamily.org/index.php?title=Main_Page)
  - [Ceres-Solver (only tested 2.1.0)](http://ceres-solver.org/)
  - 可选
    - [CUDA](https://developer.nvidia.com/cuda-downloads)
    - [TensorRT](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html)
    - [ninja-build](https://ninja-build.org/manual.html) or `ninja-build`

- 开发测试环境
  - Ubuntu
  - WSL2(不能使用工业相机)

## 使用说明

1. 安装依赖
    1. 根据上面链接安装相关依赖
    2. 安装完成后运行`sudo ldconfig`

2. 获得代码

    ```sh
    git clone --recursive https://github.com/qdu-rm-cv/qdu-rm-ai
    # or
    git clone --recursive https://gitee.com/qdu-rm-2022/qdu-rm-ai
    ```

    - 快速配置所需环境

        ```sh
        git clone https://github.com/qdu-rm-cv/environment
        cd environment
        sudo chmod 777 ./shell/*
        ./shell/env_dep_install.sh
        ./shell/code_dep_install.sh
        ```

3. 编译 & 安装

    ```sh
    cd qdu-rm-ai
    mkdir build
    cd build
    cmake ..
    make -j8 # 根据CPU选择合适的数字
    make install
    ```

4. 神经网络(可选)

    1. 准备

        ```sh
        # 安装本项目需要的Python模块。
        pip3 install -r qdu-rm-ai/requirements.txt

        # 安装YOLOv5需要的Python模块
        pip3 install -r qdu-rm-ai/third_party/yolov5/requirements.txt
        ```

    2. 训练

        ```sh
        # 以下脚本涉及相对路径，需要在此文件夹内运行。
        cd qdu-rm-ai/utils

        # 处理数据集
        python3 roco2x.py --dji-roco-dir=<path to DJI ROCO>

        # 训练导出模型
        sh ./train_vision.sh
        ```

5. 运行

    ```sh
    cd qdu-rm-ai/runtime
    # 根据应用选择程序
    auto-aim # sentry / radar ...
    ```

## 文件目录结构及文件用途说明

| 文件夹      | 内容       | 备注                                     |
| ----------- | ---------- | ---------------------------------------- |
| assets      | 资源       | 包含文档、效果展示、测试产物等相关的图片 |
| runtime     | 运行环境   | 包含运行所需文件，和运行过程产生的文件   |
| src         | 源代码     |
| third_party | 第三方软件 |
| utils       | 工具       | 脚本和文件                               |

| src内     | 内容   | 备注               |
| --------- | ------ | ------------------ |
| apps      | 应用   | 所有产生的应用程序 |
| behavior  | 行为库 | 基于行为树开发的AI |
| component | 组件库 | 各类算法和数据结构 |
| device    | 设备库 | 外接设备的抽象     |
| vision    | 视觉库 | 目标识别等代码     |

| vision内    | 内容       | 备注                                             |
| ----------- | ---------- | ------------------------------------------------ |
| classifier  | 分类器     | 装甲板分类器                                     |
| compensator | 弹道补偿器 | 三维解算、测距、弹道补偿                         |
| detector    | 探测器     | 装甲板探测、能量机关等的探测器                   |
| object      | 对象类     | 图像对象、物理对象和比赛中抽象出的各种基础对象类 |
| params      | 参数类     | 可视化调参的参数类                               |
| predictor   | 预测器     | 滤波器和集成的预测器                             |
| process     | 流程       | 对以上视觉处理模块的组合                         |

| apps内 | 内容     | 备注                               |
| ------ | -------- | ---------------------------------- |
| app    | 应用程序 | 包含哨兵程序、自瞄算法、雷达程序等 |
| demo   | 样例     | 演示用的例子                       |
| tests  | 测试代码 | 单元测试

## 系统介绍

### 软件流程图

| ![视觉程序框图](./assets/README.assets/视觉程序框图.png?raw=true "步兵嵌入式硬件框图") |
| :------------------------------------------------------------------------------------: |
|                                     *视觉程序框图*                                     |

### 行为树演示

| ![行为树演示](./assets/README.assets/行为树演示.png?raw=true "行为树演示") |
| :------------------------------------------------------------------------: |
|                                *行为树演示*                                |

## Roadmap

近期：

1. 修缮好测距模块和弹道补偿器模块

2. 引入神经网络
    1. 使用基于pytorch的yolov5算法，训练得到的权重和模型导出到ONNX格式。
    2. 在Jetson平台使用TensorRT运行导出的模型。
    3. 在NUC平台使用OpenVINO。
    4. 添加Int8运行

3. 完成各种滤波器进行平滑化处理，并完成预测器

4. 完成其余兵种任务
    1. 矿石识别部分代码
    2. 飞镖控制部分代码

远期：

1. 实现类似多级流水线的视觉算法流程。[参考文章](https://opencv.org/hybrid-cv-dl-pipelines-with-opencv-4-4-g-api/)(等待OpenCV完善架构)

2. 多线程进行生产者消费者调度

3. 优化传统视觉算法和参数增强程序鲁棒性
