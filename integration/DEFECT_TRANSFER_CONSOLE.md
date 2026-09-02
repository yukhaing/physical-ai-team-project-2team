# OMX 불량 박스 이송 통합 관제

이 문서는 YOLO가 검출한 `defect` 박스를 OMX-F 로봇팔로 Beagle에 적재하고
불량 구역까지 이송하는 관제 절차를 설명합니다. `normal` 박스는 대상에서
제외됩니다.

## 동작 흐름

1. YOLO가 안정적인 불량 박스를 검출·선택합니다.
2. OMX가 박스를 집어 수령 위치의 Beagle에 적재합니다.
3. 관제가 TCP로 `box_placed`를 전송하면 Beagle이 불량 구역으로 이동합니다.
4. 작업자는 GUI의 `하역 완료`로 복귀를 요청합니다.
5. Beagle이 수령 위치로 복귀를 시작하면 다음 불량 박스 집기를 시작합니다.
6. 다음 박스의 최종 배치는 Beagle이 수령 위치에 도착해 `READY`가 된 뒤에만
   진행합니다.

## 초기 설정과 실행

```bash
cd omx/docker
./container.sh build
./container.sh start
./container.sh enter
```

컨테이너에서 ROS 워크스페이스를 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/omx_box_project_ws
colcon build --symlink-install
```

호스트에서 GUI 스택을 실행합니다.

```bash
cd omx/docker
BEAGLE_MODE=auto ./container.sh gui-up
```

`BEAGLE_MODE`는 같은 PC의 `local`, 상대 IP 자동 사용 `auto`, 별도 PC 지정
`remote`를 지원합니다. 상태 확인과 종료는 `./container.sh gui-status`,
`./container.sh gui-down`을 사용합니다.

## 실장비 확인

- 7점 homography 보정값을 적용한 뒤 집기 좌표를 재검증합니다.
- `config/console.yaml`의 `bypass_beagle`, 자동 진행·하역 옵션을 환경에 맞게
  확인합니다.
- GUI 정지와 비상정지는 소프트웨어 요청입니다. 위험 상황에서는 OMX와 Beagle의
  물리 E-stop을 사용합니다.
