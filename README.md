# MACRO_GNC

**MACRO_GNC**는 자율운항선을 위한 **Guidance, Navigation, Control (GNC)** 시스템을 C++ 기반으로 구현한 프로젝트입니다.  
기존 Python 버전에 비해 **더 적은 CPU 및 메모리 사용량**으로 **고성능**의 실시간 처리를 목표로 합니다.

---

## 프로젝트 구성

### 디렉토리 구조

```bash
MACRO_GNC/
├── control/                # 제어 알고리즘 구현 예정
├── guidance/               # 유도 알고리즘 구현 예정
├── navigation/             # 항법 시스템 구현
│   ├── include/            # 헤더 파일
│   ├── launch/             # ROS2 launch 파일
│   ├── param/              # 파라미터 파일 (YAML)
│   ├── src/
│   │   ├── tool/           # 도구 함수 및 유틸리티
│   │   ├── use_gps/        # GPS + IMU 항법 시스템
│   │   ├── use_slam/       # SLAM + IMU 항법 시스템
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 주요 기능

### Navigation 시스템
1. use_gps:
- IMU + GPS 기반 항법 시스템
- gps_navigation.cpp에 구현

2. use_slam:
- IMU + SLAM 기반 항법 시스템
- odom_navigation.cpp에 구현

3. 파라미터 파일:
- navigation/param/navigation_params.yaml을 통해 설정 가능
- 예: 기준 GPS 좌표, 속도 필터 계수 등

## Dependencies
본 프로젝트는 ROS 2 Humble을 기반으로 하며, 다음 패키지를 필요로 합니다:

GeographicLib
- GPS 위도/경도를 UTM 좌표계로 변환

```
sudo apt-get install geographiclib-tools
sudo apt-get install ros-humble-geographic-*
```

## 향후 개발 예정
 - Guidance 알고리즘 (예: LOS, Path Following)

 - Control 시스템 (예: PID, MPC)

 - 다중 선박 확장 (스웜 제어)

 - 자율 접안 및 장애물 회피 기능


