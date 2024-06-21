# robot-arm

- 로봇암으로 물체를 구별하고, 원하는 물체를 잡고 옮기기
- 해당 로봇암에 대한 정보를 클라우드에 저장

# Project Instructions

- [x] Create Github Project

- [ ] Decide Robot Arm Hardware

  - [ ] Research Robot Arm Hardware

    - [ ] 

- [ ] Decide Project Instructions

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

- [ ] Initilise Project
  - [x] Create a new team Google account
  - [x] Sign up to ChatGPT
  - [x] Sign up to GCP
  - [x] Add bills

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

- ESP32
  - wifi: O

- raspberrypi zero
- STM32

#### Cloud Gateway
- GCP
- DB연결

## Problems

로봇팔 부품을 하나하나 구매 하려고 하는데,

어떤 부품을 사야하는 지, 감이 안옴
