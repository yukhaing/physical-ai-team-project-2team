# OMX-F MoveJ joint3 terminal trim 검증 기록 — 2026-08-05

## 목적

하드웨어 gain이나 torque 설정을 변경하지 않고 joint3의 하중 방향 deadband/hysteresis를 controller 내부의 bounded terminal trim으로 보상한다.

## 진단 결론

기존 6초 미세 staging 시험에서 joint3는 프로파일 마지막 1초와 settle 3초 동안 실제값 `0.377359 rad`에 정지했다. 목표 `0.334964 rad`를 계속 유지해도 움직이지 않았으므로 단순 시간 지연이 아니라 정지마찰/deadband 구간으로 판단했다.

진단 bag:

```text
/tmp/omx_joint3_diag_20260805_0232
```

## 구현

새 패치:

```text
docker/patches/cyclo_movej_s_curve_terminal_trim.patch
```

`docker/Dockerfile`은 기존 S-curve, feedback hold, bounded settle 다음에 terminal trim 패치를 자동 적용한다.

물리 설정:

```yaml
smooth_terminal_trim_enabled: true
smooth_terminal_trim_stall_velocity: 0.002
smooth_terminal_trim_stall_time: 0.5
smooth_terminal_trim_rate: 0.02
smooth_terminal_trim_max_offset: 0.06
smooth_terminal_trim_timeout: 5.0
smooth_terminal_trim_capture_tolerance: 0.01
```

동작은 S-curve 종료 후 joint3 오차가 `0.04 rad`보다 크고 속도가 `0.002 rad/s` 이하로 0.5초 유지될 때만 시작한다. 명목 목표 방향으로 reference offset을 제한적으로 증가시키며, 실제 joint3가 명목 목표 0.01rad 이내에 들거나 목표를 통과하면 즉시 feedback hold한다. 5초 또는 0.06rad 제한에 도달해도 feedback hold로 종료한다.

## staging 미세 보정 결과

```text
S-curve: 6.0초
trim 시작 joint3 오차: -0.0439 rad
trim capture joint3 오차: -0.0040 rad
trim offset: 0.0408 rad
최종 joint3 오차: 0.02092 rad (1.20도)
최종 TCP FK: X=0.18140, Y=-0.00219, Z=0.13806 m
```

애플리케이션이 controller trim 종료보다 먼저 완료를 선언하는 문제를 발견해 staging `minimum_completion_time`을 12초로 변경했다.

staging trim bag:

```text
/tmp/omx_joint3_trim_diag_20260805_0730
```

## 10초 XY approach 결과

목표:

```text
X=0.2096 m
Y=0.0979 m
```

Dry-run:

```text
계획 q=[0.42604, -0.36687, 0.14324, 1.02065, 0.00153]
계획 XY 오차=0.00 mm
계획 최종 Z=0.1682 m
경로 최저 Z=0.1335 m
pitch=45.67도
```

실물 결과:

```text
S-curve 시간: 10.0초
프로파일 종료 joint3 오차: -0.04084 rad
프로파일 종료 실제 XY 오차: 5.34 mm
trim 시작 joint3 오차: -0.0408 rad
trim capture joint3 오차: -0.0071 rad
trim offset: 0.0326 rad
최종 joint3 오차: -0.01936 rad (1.11도)
최종 최대 관절 오차: 0.03320 rad (1.90도, joint4)
최종 실제 XY 오차: 4.17 mm
최종 실제 FK Z: 0.16004 m
결과: COMPLETED
```

접근 노드는 `move_duration=10.0`, `minimum_completion_time=16.0`, `timeout=20.0`으로 변경했다. 완료 판정에 실제 `/joint_states` FK 기반 XY 5mm와 최종 Z 제한을 추가했다.

approach bag:

```text
/tmp/omx_xy_approach_trim_20260805_0740
```

## 종료 상태

```text
MoveJ controller: 1개
MoveL controller: 0개
/omx_movej_controller/movej publisher: 0개
joint1=0.4187768
joint2=-0.3635534
joint3=0.1626020
joint4=1.0538448
joint5=0.0015340
그리퍼: 열림
실제 joint FK: XY 오차=4.17 mm, Z=0.16004 m
```

## 주의

`/omx_movej_controller/current_pose`는 controller command state에서 계산되므로 feedback hold 후 실제 `/joint_states` FK와 차이가 날 수 있다. 정밀 완료 판정과 기록에는 실제 `/joint_states` 기반 FK를 사용한다.

단순 tolerance 확대는 하지 않는다. XY 5mm 제한과 bounded trim 제한을 유지한다.
