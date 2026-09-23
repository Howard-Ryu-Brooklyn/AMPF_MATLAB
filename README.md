# Acyclic Minimally Persistent Formation Control

*측정치가 부족하고 움직임에 제약이 걸린 상황에서도, 편대는 목표 형태로 수렴해야 한다.*

3대 이상의 로봇이 서로 거리 측정만으로 목표 편대를 이루는 문제는 모든 로봇이 필요한 거리를
항상 측정할 수 있다는 전제 위에 서 있다. 하지만 실제 로봇에는 센서 시야각이 있고, 구동에도
제약이 있다 — 어떤 순간엔 필요한 상대거리를 아예 측정하지 못한다. 이 저장소는 그 전제가
깨졌을 때도 편대가 무너지지 않고 수렴한다는 것을, 이론(Lyapunov 안정도 해석)과 시뮬레이션
양쪽에서 다루는 GIST 석사학위논문의 연구 코드입니다.

## 접근

Acyclic minimally persistent 그래프 구조를 대상으로, 알고리즘을 다섯 단계로 쌓아 올렸습니다.
각 폴더가 그 단계 하나에 대응합니다.

| 단계 | 폴더 | 다루는 문제 |
|---|---|---|
| 1 | `1. Basic Theory` | 선행 연구 3편(적응 소스 위치추정, 거리 기반 추적, non-steepest descent 편대 제어) 재현·검증 |
| 2 | `2. Flexible Coordination Control Law` | 거리 기반 gradient descent 제어 법칙 확장 |
| 3 | `3. Mode Divide Algorithm` | 구동·측정 제약 상황별로 제어 모드를 전환하는 알고리즘 |
| 4 | `4. Localization` | 측정치 결손을 보완하는 적응 추정(estimation) 법칙 |
| 5 | `5. Total Algorithm` | 위 요소를 결합한 전체 알고리즘과 안정도 해석 |

즉 1은 기존 이론이 실제로 서 있는 토대인지 먼저 확인한 것이고, 2~4는 그 토대 위에서 부족한
조건(제약, 결손 측정치) 하나씩을 다룬 확장이며, 5에서 전체를 합쳐 Lyapunov 안정도를 증명합니다.

## 결과

<p align="center">
  <img src="docs/media/trajectory.jpg" width="420" alt="편대 수렴 궤적 (x-y 평면)">
  <img src="docs/media/distance_error.jpg" width="420" alt="상대거리 오차 수렴">
</p>

측정치 결손과 모드 전환이 섞인 시나리오에서도 편대가 목표 형태로 수렴하고, 상대거리 오차가
0으로 줄어드는 것을 확인했습니다. 더 자세한 그래프는 각 단계 폴더의 `Figure/`에 있습니다.

## 실기 검증

이론과 시뮬레이션에 그치지 않고, Turtlebot3와 Crazyflie로 구성된 이기종 로봇 팀으로 실기
검증까지 진행했습니다.

- **[AMPF_GC](https://github.com/Howard-Ryu-Brooklyn/AMPF_GC)** — ROS2 기반 편대 제어기 구현
  (follower 1/2 controller), ArUco·ZED 카메라 기반 상대 위치 추정
- **[AMPF_Jetson](https://github.com/Howard-Ryu-Brooklyn/AMPF_Jetson)** — Jetson Nano 온보드
  배포, Turtlebot3 구동 스택, LiDAR(LD08)·UWB 드라이버 연동

세 저장소가 이론 → 구현 → 실기 배포로 이어지는 하나의 프로젝트입니다.

## 실행

```matlab
% 예: 전체 알고리즘 시뮬레이션
cd 'Matlab/5. Total Algorithm'
main
```

각 단계 폴더의 `main.m`이 진입점이고, `Scripts/`에 시뮬레이션·플로팅 로직이, `Figure/`에
결과 그래프가 있습니다.

## 논문

`Master_Degree_Dissertation.pdf` — *Gradient-based Acyclic Minimally Persistent Formation
Control with Measurement Deficiency and Motion Constraints* (GIST 기계로봇공학부, 2024)

전체 이론 전개와 증명은 논문 원문을 참고해 주세요.
