# [ Robot Arm ]

- 로봇암으로 물체를 구별하고, 원하는 물체를 잡고 옮기기
- 해당 로봇암에 대한 정보를 클라우드에 저장

# :information_desk_person: Infomation

| Project  | Robot Arm                                         |
| :------: | ------------------------------------------------- |
|   Team   | 김진호, 김진성, 김진세, 신지환                    |
| Hardware | raspberrypi zero                                  |
|   Lang   | C/C++                                             |
|  Skills  | MySQL on Google Cloud Platform <br> ROS <br> RTOS |

# Project Instructions

- [x] Create Github project

- [x] Decide robot arm hardware

  - [x] Robot Arm controller

  - [ ] Camera module

  - [ ] WiFi module for the camera

    - [x] nRF52840 Dongle
          Host 를 통해서만 전원 공급가능

    - [x] Particle Argon
          64MHz CPU clock

    - [x] ARM Cortex-M4F
          64MHz CPU clock

    - [x] ESP32-DOWD
          240MHz CPU clock

    - [x] Adafruit Board

    - [x] Raspberry Pi W
          1GHz CPU clock

  - [ ] Analyze the Robot Arm

- [ ] Decide project instructions

  - Project Conditions

    - Iot
      We must use Internet
      -> wifi module
      -> ESP32
      -> raspberrypi zero

    - Cloud Computing
      Sensor -> Gateway -> Cloud
      GCP

    - Big data & Data rangling

- [ ] Initilise project

  - [x] Create a new team Google account
  - [x] Sign up to ChatGPT
  - [x] Sign up to GCP
  - [x] Add bills

- [ ] Coding!

  - [ ] Research robot arm software

## 하드웨어 선정

#### Claw

- 집게부
- 미세 조절 가능 여부

**RM-X52 선택**

#### Camera

- 소형

#### Frame of Arm

- 프레임 판매처
- 서보모터 연결 가능여부

**RM-X52 선택**

#### Servo Motor

- 로봇암 관절

_XM430-W350-T (RM-X52)_

#### Stearing Gear

- 로봇팔 방향 설정

**RM-X52 선택**

#### Controller

고려사항

- wifi를 통해 Gateway와 연결
- 명령어 수신
- 센싱데이터 전송

  - 카메라 데이터 전송 속도

    - 최소 1GHz 이상

- 모터 연결 가능

  - 최소 4개 이상 (자유롭게 움직이기 위해)

- ESP32

  - wifi: O
  - cpu clock: 240MHz

- raspberrypi zero w

  - wifi: O
  - cpu clock: 1GHz

- STM32

##### OpenCR1.0

ARM Cortex-M7 MCU

#### U2D2 & U2D2 Power Hub Board

PC -> U2D2 -> Power Hub Board -> DYNAMIXEL

It can supply various kinds of external power supply (to DYNAMIXEL in RM-X52)

- Operating Voltage : 3.5V ~ 24.0V

##### Structure

- DYNAMIXEL Connectors
  3 & 4-pinds JST connectors are installed for communication
  (JST: electric company in japen)

- TTL/RS485 Test Points

  - TTL data line  
    Time To Live, How long data is valid and available before a computing system discards it.

  - RS485 D+, TIA-485(-A), EIA-485  
    The Standard that define the electrical characteristics of drivers and receivers for use in serial communications systems.
    (D-line signals can be easily identified via the test points)

  - Power Connectors  
    U2D2 PHB provides three different types of power connectors to supply a diverse voltage and current to DYNAMIXEL.

    1. SMPS DC Connector (2.5mm ID / 5.5mm OD, Center Positive)
    2. MOLEX Power connector
    3. PCB Screw Terminal Block Connector (Left: -, Right: +)

#### Daisy-chaining

> A method of propagating signals along a bus in which the devices are connected in series and the signal passed from one device to the next.

#### Cloud Gateway

- GCP
- DB연결

## Problems

로봇팔 부품을 하나하나 구매 하려고 하는데,

어떤 부품을 사야하는 지, 감이 안옴

# Day Log

## Research robot arm hardware (20240624)

- [x] Robot Arm Frame
  - [x] Dofbot
        RM-X52 선택으로 인해 취소
  - [x] RM-X52
        구매 예산 부족 -> 대여

## Initialise Robot Arm (20240625)

- install ubuntu 20.04 to jetson nano  
  https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image
  (Using Rufus)
- install ROS 2 Foxy Fitzroy on ubuntu
- install ROS Packages
- [x] micro 5pin cable

## Switch to ROS1 (20240628)

- Add Gripper open/close code
- To use Moveit library on openmanipulator-x

## 20240701

- Install & setup ROS moveit

  - [ ] launcher is stopped often.

- Connect camera to opencv
  - [ ] camera doesn't detected
    - camera is broken (buy another one)

## 20240702

#### Implement Object Detection
  카메라 -> 객체 좌표 -> ROS

  - OpenCV

    ```bash
    sudo apt install python3-pip
    python3 -m pip install opencv-python
    ```

  - YOLO  
    You Only Look Once, Object Detection

  - What's the Jetpack SDK  
     It's bundles all the Jetson platform software, including TensorRT, cuDNN, CUDA Toolkit, Vision Works, Streamer, and OpenCV.
    All built on top of L4T with LTS Linux kernel.  
    To leveraging the GPU on the Jetson Nano.

    - Install Jetpack SDK  
      `sudo apt install nvidia-jetpack`

- learn what is moveit and how to use it.

- teleop를 통해 robot-arm 움직이는 것 영상 찍기 -> 분석, 계획 플래닝

#### 라즈베리파이 활용

- 라즈베리파이 5 vs Jetson Nano (ARM CortexA57)  
  - CPU Frequency
    데이터 집약적 작업에 유용
    센서데이터 수집, 초기데이터 관리, 데이터 분석
    통신 노드, 웹 서버, 파일 서버, 모니터링 대시보드

    Jetson Nano: 1.43 GHz
    **승** raspberry pi 5: 2.4 GHz

  - GPU Frequency  
    GPU 가속이 필요한 작업에 유용
    컴퓨터 비전, 실시간 비디오 처리, 딥러닝 기반 분석
    GPU기반 알고리즘 사용

    **승** Jetson Nano: 921 MHz
    raspberry pi 5: 800 MHz

- 처리 분산