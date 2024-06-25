# robot-arm

- 로봇암으로 물체를 구별하고, 원하는 물체를 잡고 옮기기
- 해당 로봇암에 대한 정보를 클라우드에 저장

# Project Instructions

- [x] Create Github project

- [ ] Decide robot arm hardware

  - [ ] Research robot arm hardware (20240624)

    - [ ] Robot Arm Frame

      - [ ] Dofbot

      - [ ] RM-X52
            구매 예산 부족 -> 대여

    - [ ] WiFi module

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

#### Camera

- 소형

#### Frame of Arm

- 프레임 판매처
- 서보모터 연결 가능여부

#### Servo Motor

- 로봇암 관절

#### Stearing Gear

- 로봇팔 방향 설정

#### Controller

고려사항

- wifi를 통해 Gateway와 연결
- 센싱데이터 전송
- 명령어 수신

- 카메라 데이터 전송 속도

  - 최소 1GHz 이상

- 모터 연결 가능

  - 최소 4개 이상 (자유롭게 움직이기 위해)

- ESP32

  - wifi: O
  - cpu clock: 240MHz

- raspberrypi zero

  - wifi: O
  - cpu clock: 240MHz

- STM32

#### Cloud Gateway

- GCP
- DB연결

## Problems

로봇팔 부품을 하나하나 구매 하려고 하는데,

어떤 부품을 사야하는 지, 감이 안옴
