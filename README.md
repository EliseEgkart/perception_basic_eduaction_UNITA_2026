# UNITA 2026.01.02 2차 기본교육

본 교육은 지난주에 수행한 시뮬레이션 기반의 주행을 실제 테스트용 차량 플랫폼에 적용하기 전, 자율주행  차량의 인지부에 사용되는 주요한 센서인 카메라, 라이다, 초음파센서를 활용한 프로젝트를 수행하여 동아리 팀원 전체의 인지 기술의 기본 역량을 확보하는 데 있다.
> ros2에 대한 기본 

## 참여 인원 및 일시 & 장소

- **강의자**: 김형진  
- **교육 조교**: 이다빈, 윤제호  
- **교육 참여자**:  
  강민수, 김민서, 박근호, 성현영, 윤지윤, 이석빈,  
  이원종, 장동혁, 정가용, 정규민, 한주형  

- **교육 일시**: 2026년 01월 02일 10:00 ~ 17:00  
  (점심시간 제외, 총 6시간)  
- **교육 장소**: 공과대학 8호관 117호  

## 차량 전력 및 신호 계통 소개.
![mobility_power](img/project_presentation_power.png)

![mobility_signal](img/project_presentation_signal.png)

## ROS2 파일 계층 소개 및 pkg 생성 방법.

### ROS2 파일 시스템의 계층 구조
일반적으로
ROS2 파일시스템의 구조는 

### pkg 생성 방법
```sh
cd ~/본인의 워크스페이스/src

### python based package 생성방법
ros2 pkg create --build-type ament_python python_test_pkg

### cpp based package 생성방법
ros2 pkg create --build-type ament_cmake cpp_test_pkg
```

### 외부(github)에서 pkg 불러오기
- [usb_cam Repo](https://github.com/ros-drivers/usb_cam)
```sh
### usb_cam package install guide
sudo apt-get install ros-humble-usb-cam

cd ~/본인의 워크스페이스/src

git clone https://github.com/ros-drivers/usb_cam.git
```
- [rplidar_ros Repo](https://github.com/Slamtec/rplidar_ros)

```sh
### rplidar package install guide
cd ~/본인의 워크스페이스/src

git clone https://github.com/Slamtec/rplidar_ros.git

git branch # 해당 repo는 기본 값이 ros1, 즉 catkin build system을 사용하기에 사용불가.

git checkout ros2 # 해당 명령어로 branch를 ros2로 변경해줘야함.
```

## 0. 경쟁형 퀴즈를 통한 팀 배치
세부사항은 quiz.md 참고
[시뮬레이션 설명](./quiz.md/)
```
1팀 7점    2팀 0점    3팀 3점
4팀 6점    5팀 2좀    6팀 4점
```
- **firmware**: 김형진, 이석빈, 성현영, 박근호
- **perception1(camera)**: 이다빈, 정가용, 강민수, 장동혁, 한주형
- **perception2(ultrasonic, 2d lidar)**: 윤제호, 윤지윤, 정규민, 김민서, 이원종


## 인지 1팀 미션 : 카메라 기반 신호등 인지

### 팀 구성
- 실습조교: 이다빈  
- 팀원: 강민수, 장동혁, 정가용, 한주형  

### 미션 개요
본 미션의 목표는 **카메라 센서를 활용하여 실제 환경의 신호등을 인지하고**, 인지 결과를 **음성 출력 및 RViz 시각화**를 통해 확인하는 것이다.  
학교 앞 공대 인근 신호등을 대상으로 하며, 단순 영상 재생이 아닌 **직접 촬영 및 실시간 인지 여부 확인**을 필수로 한다.


### 구현 요구 사항

#### 1. 신호등 인지
- 카메라 센서를 사용하여 **적색 / 황색 / 녹색 신호등 인지**
- 실제 환경에서 촬영된 영상 또는 실시간 스트림 사용
- 인지 성공 여부를 명확히 판단할 수 있어야 함

#### 2. 음성 출력
- 신호등 인지 결과를 음성으로 출력
- 예시:
  - “적색 신호등이 인식되었습니다.”
  - “녹색 신호등이 인식되었습니다.”

#### 3. 실제 학교 앞 신호등에서 테스트 할 것.
- 추운 날씨에 밖에 나가서 팀원들과 직접 해볼 것.


## 인지 2팀 미션 : 초음파 센서 및 LiDAR 기반 물체 인지

### 팀 구성
- 실습조교: 윤제호  
- 팀원: 김민서, 이원종, 윤지윤, 정규민  

### 미션 개요
본 미션의 목표는 **초음파 센서와 LiDAR를 동시에 활용한 물체 인지 시스템 구축**이다.

주어진 **6개의 초음파 센서**와 **RPLiDAR C1**을 모두 사용하여  
물체 인지, 알림, 시각화를 수행한다.

### 구현 요구 사항

#### 1. 초음파 센서 기반 물체 인지
- 총 6개의 초음파 센서를 모두 사용
- 특정 초음파 센서에서 물체가 감지되면 음성 출력
- 음성 출력 예시:
  - “1번 초음파 센서 물체 감지입니다.”
  - “3번 초음파 센서 물체 감지입니다.”

#### 2. LiDAR 기반 물체 클러스터링
- RPLiDAR C1을 사용하여 주변 물체 인지
- LiDAR 데이터 기반 물체 클러스터링 수행
- 단순 거리 출력이 아닌 **물체 단위 인지**를 목표로 함

#### 3. RViz 시각화
- LiDAR 스캔 및 클러스터링 결과를 RViz에서 시각화
- 마커 기능 사용
- 물체 위치가 직관적으로 보이도록 표현