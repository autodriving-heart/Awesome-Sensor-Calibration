## Awesome-Sensor-Calibration

本仓库由[公众号【自动驾驶之心】](https://mp.weixin.qq.com/s?__biz=Mzg2NzUxNTU1OA==&mid=2247542481&idx=1&sn=c6d8609491a128233c3c3b91d68d22a6&chksm=ceb80b18f9cf820e789efd75947633aec9d2f1e8b58c29e5051c05a64b21ae63c244d54886a1&token=11182364&lang=zh_CN#rd) 团队整理，欢迎关注，一览最前沿的技术分享！

自动驾驶之心是国内首个自动驾驶开发者社区！这里有最全面有效的自动驾驶与AI学习路线（感知/定位/融合）和自动驾驶与AI公司内推机会！

## 一、**传感器时间同步** | Time Synchronization

时间同步：

一、统一时钟源 

1、假设各传感器都有自己的内部时钟，但由于每个时钟的钟漂不一样，导致各时钟之间存在时间差，所以需要统一各传感器的时间源。 

2、一般采用GPS时间作为统一时间源，通过GPS给各个传感器提供基准时间，各传感器根据提供的基准时间校准各自的时钟时间。

 3、各传感器根据已经校准后的各自时间为各自独立采集的数据加上时间戳信息，来做到所有传感器时间戳同步。

 二、硬件同步 

由于每种传感器的采样频率不一致，如lidar通常为10Hz，camera通常为25/30Hz，不同传感器之间的数据传输还存在一定的延迟，那么

可以通过寻找相邻时间戳的方法找到最近邻帧，但如果两个时间戳相差较大，且传感器或障碍物又在运动，那么最终会得到较大的同步误

差。这个情况可以采用硬件同步触发的方法来缓解查找时间戳造成的误差现象。如何实现激光雷达和相机的时间同步呢？也就是在激光雷

达和相机的时间源都统一到GPS时间后，如何保证他们同步触发采集呢？是以PPS信号为触发信号，实现激光雷达和相机均在PPS信号上

升沿的时候采集数据，且打上各自时钟的时间戳。由于PPS的频率只有1Hz，所以通常需要一个设备把PPS信号转发为任意频率、但是跟

PPS信号同相位的方波，这样就可以控制相机的采集频率了 

三、软同步 

当两传感器的采集时刻不一致，但又想得到同一时刻下两传感器的结果，就可以使用软件同步的方式。目前常采用的软件同步方式为内插

外推法，此方法适用于传感器间帧率不存在倍数关系的情况，或者传感器有帧率不稳定的情况。主要利用两个传感器帧上的时间标签，计

算出时间差，然后通过包含有运动信息的目标帧与时间差结合，对帧中每一个目标的位置进行推算，推算出新的帧时各个目标的位置，并

于原有的两帧之间建立新的帧。



多传感器时间同步图解

[[link]](https://www.zhihu.com/question/363919550/answer/1982689792)

## **二、多相机标定** | Multi-Camera-Calibration

Multi-camera calibration using one or more calibration patterns

[[code]](https://github.com/oliver-batchelor/multical)

Calibration of Asynchronous Camera Networks: CALICO

Infrastructure-based Multi-Camera Calibration using Radial Projections.

Infrastructure-Based Calibration of a Multi-Camera Rig

Leveraging Image-based Localization for Infrastructure-based Calibration of a Multi-camera Rig.

Deep Learning for Camera Calibration and Beyond: A Survey

## 三、**相机激光雷达标定** | Camera-LIDAR-Calibration

An Extrinsic Calibration Tool for Radar, Camera and Lidar

Joint camera intrinsic and lidar-camera extrinsic calibration.2022

Unified Intrinsic and Extrinsic Camera and LiDAR Calibration under Uncertainties（ICRA2020） 

Automatic online calibration of cameras and lasers. RSS2013

Automatic targetless extrinsic calibration of a 3d lidar and camera by maximizing mutual information. AAAI2012.

Online camera-lidar calibration with sensor semantic information. ICRA2020

Crlf: Automatic calibration and refinement based on line feature for lidar and camera in road scenes.2021

3d lidar-camera extrinsic calibration using an arbitrary trihedron.2013

Automatic extrinsic calibration of vision and lidar by maximizing mutual information.2015

Automatic calibration of lidar and camera images using normalized mutual information.2012

Multi-FEAT: Multi-Feature Edge AlignmenT for Targetless Camera-LiDAR Calibration.2022

A Novel Calibration Method between a Camera and a 3D LiDAR with Infrared Images（ICRA2020）

Automatic extrinsic calibration between a camera and a 3D Lidar using 3D point and plane correspondences

Online LiDAR-Camera Extrinsic Parameters Self-checking

An optimization-based IMU-Lidar-Camera Co-calibration method

Online Multi Camera-IMU Calibration

Spatiotemporal Calibration of 3D mm-Wavelength Radar-Camera Pairs

General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox

[[code]](https://github.com/koide3/direct_visual_lidar_calibration)

## 四、**相机IMU标定** | Camera-IMU-Calibration

A kalman filter-based algorithm for IMU-camera calibration: Observability analysis and performance evaluation

Spatio-temporal initialization for IMU to camera registration

Optimization based IMU camera calibration

[[tool]](https://github.com/ethz-asl/kalibr)

An optimization-based IMU-Lidar-Camera Co-calibration method

Online Multi Camera-IMU Calibration

Spatiotemporal Calibration of 3D mm-Wavelength Radar-Camera Pairs

## 五、**相机毫米波雷达标定**  | Camera-Millimeter-Wave-Radar-Calibration

Integrating millimeter wave radar with a monocular vision sensor for on-road obstacle detection applications

Targetless Rotational Auto-Calibration of Radar and Camera for Intelligent Transportation Systems

A novel method of spatial calibration for camera and 2D radar based on registration

Radar and vision sensors calibration for outdoor 3D reconstruction

An optimization-based IMU-Lidar-Camera Co-calibration method

Online Multi Camera-IMU Calibration

Spatiotemporal Calibration of 3D mm-Wavelength Radar-Camera Pairs

## **六、毫米波与激光雷达标定** | Millimeter-Wave-and-LIDAR-Calibration

Extrinsic and Temporal Calibration of Automotive Radar and 3D LiDAR

## **七、在线标定方法** | Online-Calibration-Method

Online extrinsic calibration based on per-sensor ego-motion using dual quaternions. 2021

Online temporal calibration for monocular visual-inertial systems. IROS 2018

Rggnet: Tolerance aware lidarcamera  online calibration with geometric deep learning and generative model. 2020

[[code]](https://github.com/KleinYuan/RGGNet)

Online Camera-LiDAR Calibration with Sensor Semantic Information. ICRA 2020.

## **八、其它标定方法** | Other-Calibration-Methods

An Extrinsic Calibration Tool for Radar, Camera and Lidar

Extrinsic 6DoF Calibration of a Radar – LiDAR – Camera System Enhanced by Radar Cross Section Estimates Evaluation

激光雷达-IMU标定

[[tool]](https://github.com/ethz-asl/lidar_align)

Deepcalib: A deep learning approach for automatic intrinsic calibration of wide field-of-view cameras. 2018

[[code]](https://github.com/alexvbogdan/DeepCalib)

Calibration of fish eye camera using entrance pupil

Croon: Automatic multi-lidar calibration and refinement method in road scene. 2022

Extrinsic multi-sensor calibration for mobile robots using the gauss-helmert model. IROS 2017

A general approach to spatiotemporal calibration in multisensor systems. 2016

Unified temporal and spatial calibration for multi-sensor systems.2013.

Real-time temporal and rotational calibration of heterogeneous sensors using motion correlation analysis

Regnet: Multimodal sensor registration using deep neural networks. 2017

Calibnet: Geometrically supervised extrinsic calibration using 3d spatial transformer networks. IROS 2018

Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. ICRA 2016

Automatic online calibration of cameras and lasers

Extrinsic Calibration of an Eye-In-Hand 2D LiDAR Sensor in Unstructured Environments Using  ICP.

Efficient  Extrinsic  Calibration  of  Multi-Sensor  3D  LiDAR  Systems  forAutonomous  Vehicles  using  Static  Objects  Information

Target-free Extrinsic Calibration of a 3D-Lidar and an IMU

An optimization-based IMU/Lidar/Camera Cocalibration method

## **九、标定工具** | Calibration-Tools

OpenCalib

[[code]](https://github.com/PJLab-ADG/SensorsCalibration)

Autoware

[[code]](https://github.com/autowarefoundation/autoware)

ApolloCalib

[[code]](https://github.com/ApolloAuto/apollo/tree/master/modules/calibration)

kalibr

[[code]](https://github.com/ethz-asl/kalibr)

Autoware Calib

[[code]](https://github.com/autowarefoundation/autoware_ai_utilities/tree/master/autoware_camera_lidar_calibrator)

LivoxCalib

[[code]](https://github.com/Livox-SDK/livox_camera_lidar_calibration)

MatlabCalib

[[code]](https://ww2.mathworks.cn/help/lidar/ug/lidar-and-camera-calibration.html)

PJLabCalib

[[code]](https://github.com/PJLab-ADG/SensorsCalibration)

Teri4Calib

[[code]](https://github.com/tier4/CalibrationTools)

## 十、代码 | code

#### **Lidar-Camera标定**

使用3D-3D点对应进行LiDAR相机标定

[[code]](https://github.com/ankitdhall/lidar_camera_calibration)

相机-LiDAR标定手册

[[code\]](https://github.com/Livox-SDK/livox_camera_lidar_calibration)

ROS相机激光雷达标定包

[[code\]](https://github.com/heethesh/lidar_camera_calibration)

基于全景基础设施的多摄像机和3D激光雷达标定

[[code\]](https://github.com/alibaba/multiple-cameras-and-3D-LiDARs-extrinsic-calibration)

#### **Lidar-Radar标定**

[[code]](https://github.com/keenan-burnett/radar_to_lidar_calib)

#### **Lidar-Lidar标定**

双激光雷达联合标定

[[code]](https://github.com/BIT-MJY/Multiple_Lidar_Calibration)

T-FAC:多线激光雷达的无目标自动校准
[[code]](https://github.com/AlienCat-K/LiDAR-Automatic-Calibration)

#### **Camera-IMU标定**

[[code\]](https://github.com/ethz-asl/kalibr)