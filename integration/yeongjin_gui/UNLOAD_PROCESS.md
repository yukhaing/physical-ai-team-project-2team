# 독립 하역 OMX 프로세스

이 프로세스는 메인 적재 OMX 및 통합 관제와 독립적으로 두 번째 OMX와
`/unload_camera/image_raw` 카메라만 사용한다.

## 안전한 최초 실행

호스트에서 장치 경로를 확인하고 패키지를 빌드한다.

```bash
ls -l /dev/serial/by-id/
ls -l /dev/video*
./scripts/unload_omx_process_container.sh build
```

먼저 카메라를 보정한다.

```bash
UNLOAD_VIDEO_DEVICE=/dev/video2 \
  ./scripts/unload_omx_process_container.sh calibration-up
```

브라우저에서 `http://127.0.0.1:8088/`을 열고 `START`를 누른 뒤 화면에 표시된
P1부터 P7까지 순서대로 클릭한다. 결과는
`integration/yeongjin_gui/runtime/calibration/unload_active.yaml`에 저장된다. 저장 후 화면의
독립된 지점을 클릭하고 그 지점의 실측 `link0 X/Y`를 입력해 `CHECK ERROR`로 오차를
확인한다. 현재 파일의 일반 homography 평균 오차는 10.50mm로 8mm 경고 기준을 넘으므로,
재보정 후 독립점 오차를 확인하기 전에는 실제 사이클을 활성화하지 않는다.

```bash
./scripts/unload_omx_process_container.sh calibration-down
```

## Beagle 트레이 자연 랜드마크

기본 하역 카메라는 YOLO나 인쇄 마커를 사용하지 않는다. 화면 하단의 검은색
Beagle 트레이 외곽을 검출하고, 티칭 당시 트레이 중심과 현재 중심의 차이로 도착
위치의 XY 오차를 보정한다. 트레이가 거의 정사각형이라 외곽 각도가 90도씩 바뀔
수 있으므로 현재 기본 모드에서는 회전 보정 없이 필요한 ±1cm 평행 이동만 반영한다.

트레이 외곽은 다음 상태를 유지한다.

- 카메라에서 트레이의 바깥 테두리가 잘리지 않아야 한다.
- 큰 검은 물체가 트레이 외곽과 맞닿지 않아야 한다.
- 박스는 트레이 내부의 고정 위치에 둔다.

ArUco ID 0 검출 기능과 인쇄 파일은 향후 회전 보정이 필요할 때 사용할 수 있도록
보조 옵션으로 남겨 두었지만 현재 작업에는 필요하지 않다.

## 하역 기준 위치 티칭

보정이 끝나면 Beagle을 정상 도착 위치에 두고 박스 하나를 트레이의 정상 위치에 올린다.
기존 하역·보정 세션을 모두 종료한 상태에서 다음 명령을 실행한다.

```bash
./scripts/unload_omx_process_container.sh build
./scripts/unload_omx_process_container.sh teach
```

티칭 화면은 자동 사이클 coordinator를 실행하지 않으며 키 입력이 있을 때만 움직인다.

- `G`: 트레이 기준을 잠근 뒤 vision parking에서 높은 staging을 거쳐 티칭 시작점으로 이동
- `B`: 티칭을 마친 팔을 카메라 시야 밖 vision parking으로 복귀
- `W/S`: TCP X축 ± 이동
- `A/D`: TCP Y축 ± 이동
- `1`, `5`, `0`: 각각 1mm, 5mm, 10mm 이동 단위
- `P`: 실제 `/unload_omx/joint_states` 기반 TCP와 Beagle 트레이 기준 XY 출력
- `V`: 현재 로봇 XY/joint5와 안정된 트레이 기준 XY를 함께 저장
- `X` 또는 `ESC`: 종료

`G`를 먼저 눌러 접근 높이로 이동한 뒤 그리퍼 중심을 박스 중심의 수직 위에 맞춘다.
그 다음 `P`로 좌표를 확인하고 `V`를 누른다. 저장 후에는 `B`로 vision parking에 복귀한 뒤 `X`로 종료한다.
저장 결과는
`integration/yeongjin_gui/runtime/calibration/unload_source_teach.yaml`에 기록된다.
이후 정상 운전의 목표는 다음 식으로 계산된다.

```text
목표 로봇 XY = 티칭 로봇 XY + (현재 트레이 XY - 티칭 당시 트레이 XY)
```

카메라는 Z를 결정하지 않는다. 접근 Z `0.19253m`와 파지 Z `0.15599m`는 기존에
검증된 적재 place 높이를 그대로 사용한다.

처음에는 반드시 dry-run으로 실행한다.

```bash
UNLOAD_OMX_PORT_NAME=/dev/serial/by-id/<하역_OMX_ID> \
UNLOAD_VIDEO_DEVICE=/dev/video2 \
UNLOAD_DRY_RUN=true \
  ./scripts/unload_omx_process_container.sh up

./scripts/unload_omx_process_container.sh status
./scripts/unload_omx_process_container.sh cycle
./scripts/unload_omx_process_container.sh logs
```

`cycle`은 최신 타깃이 1초 이내이고, 트레이 중심의 7개 샘플 위치 편차가 5mm 이하이며,
티칭 기준 이동 30mm 이내이고 로봇 도달 반경 안일 때만 계획을 만든다. 계획과
실제 설치 상태를 확인한 다음에만 `UNLOAD_DRY_RUN=false`로 다시 기동한다.

실제 사이클은 다음 순서다.

1. 시작 순간의 보정된 XY와 박스 각도를 잠근다.
2. 대기 자세에서 그리퍼를 열고 Beagle 위 박스 상단으로 이동한다.
3. 수직 하강해 박스를 집고 같은 경로로 상승한다.
4. 관절 1을 180도 회전해 뒤쪽 배출 위치로 이동한다.
5. 수직 하강해 놓고 상승한 뒤 빈 그리퍼를 원래 방향으로 돌린다.
6. 높은 staging을 거쳐 카메라 시야를 비우는 vision parking으로 복귀하고 `COMPLETE`를 발행한다.

취소와 종료:

```bash
./scripts/unload_omx_process_container.sh cancel
./scripts/unload_omx_process_container.sh down
```

소프트웨어 취소는 물리 E-stop을 대신하지 않는다. 위험 상황에서는 OMX 전원을 차단하거나
물리 E-stop을 사용한다.

