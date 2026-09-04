# 2026-08-10 MoveJ Pick/Place 작업 로그 및 인계

## 오늘 확정한 전체 흐름

```text
Staging
→ 박스 목표 선택
→ Pick XY 접근
→ Pitch pregrasp
→ 그리퍼 열기
→ Pick 하강
→ 그리퍼 닫기
→ 사용자 grasp 확인
→ Loaded lift
→ Place XY + pitch 접근
→ Place 하강
→ 그리퍼 열기
→ Staging 복귀
```

Staging부터 loaded lift까지의 pick 흐름은 오늘 최종 place 구조를 수정하면서 변경하지 않았다.

## 실물 검증값

- Pick 하강 완료 허용 Z: `25-38 mm`
- Pick 실측 성공 자세: TCP `Z=27.23 mm`, pitch `90.44 deg`
- Lift 실측: TCP `Z=27.2 → 127.5 mm`, XY shift `9.02 mm`
- Place 하강 승인 자세: TCP `Z=35.30 mm`, pitch `82.97 deg`, 목표 XY 오차 `5.75 mm`
- Place 하강 완료 허용 Z: `33-38 mm`
- Place 최저 계획 경로 Z: `30 mm`

## 오늘 변경한 Place 구조

기존 활성 흐름의 `place rotation`과 `place lift correction`을 제거했다. 회전 단계에서 하중 보정이 반복 적용되며 TCP Z가 불필요하게 상승할 수 있었기 때문이다.

Loaded lift 뒤에는 `movej_xy_approach_node.py`를 재사용하는 `movej_place_recovery`가 기억된 place XY와 pitch를 한 번에 맞춘다.

- 계획 목표 Z: `60 mm`
- Pitch 범위: `78-82 deg`
- 우선 pitch: `80 deg`
- 최종 XY 허용오차: `10 mm`
- 실물 시험 결과: 계획 Z `59.1 mm` → 실측 Z `38.9 mm`
- 실측 pitch: `81.65 deg`
- 실측 XY 오차: `9.22 mm`

이후 즉시 place 하강을 실행하고 완료되면 그리퍼를 연다. 기존 rotation 및 correction 파일은 진단용으로 남겼지만 `pick_coordinator.launch.py`에서는 실행하지 않는다.

## 변경 파일

- `src/omx_box_control/scripts/pick_coordinator_node.py`
- `src/omx_box_control/launch/pick_coordinator.launch.py`
- `src/omx_box_control/config/pick_coordinator.yaml`
- `src/omx_box_control/config/movej_place_recovery.yaml`
- `docs/CODEX_HANDOFF.md`

## 검증 결과

- Python 구문 검사 통과
- 관련 YAML 파싱 통과
- ROS Jazzy 컨테이너에서 `omx_box_control` 빌드 성공
- `ros2 launch omx_box_control pick_coordinator.launch.py --show-args` 성공
- 코드 저장 중에는 로봇 동작 명령을 보내지 않았다.

## 다음 실행 전 확인

1. 그리퍼 ID16 초기화 및 빨간 LED 해제 여부를 확인한다.
2. 그리퍼 단독 open/close 동작과 feedback을 확인한다.
3. 이전 coordinator와 stage 자식 노드가 남아 있지 않은지 확인한다.
4. `/omx_movej_controller/movej`의 물리 명령 publisher가 coordinator 하나뿐인지 확인한다.
5. 컨테이너에서 최신 workspace를 source한 뒤 coordinator launch를 새로 시작한다.
6. Staging 완료 후 박스를 다시 선택하고 기존 pick 절차를 진행한다.

## 그리퍼 주의사항

박스를 잡은 상태에서 close 목표 `0.0`, 최대 effort `10`이 오래 유지되면 ID16이 과부하로 shutdown되고 빨간 LED가 점멸할 수 있다. 빨간 LED가 보이면 반복 명령을 보내지 말고 로봇과 박스를 안전하게 지지한 뒤 그리퍼를 초기화한다.

## 2026-08-11 place transfer Z 수정

기존 place XY/pitch 접근의 `target_z=60mm`, `min_final_z=33mm` 설정은 계획상 59.1mm였던 자세가 실물에서 38.9mm까지 내려가도 성공으로 인정했다. 이를 높은 transfer와 별도 place descent 구조로 수정했다.

- place XY/pitch 목표 Z: `100mm`
- 계획 경로 및 실제 완료 하한: `75mm`
- 실제 목표 Z 허용오차: `±25mm`
- pitch: 기존 `78-82deg` 유지
- joint delta weight: `10.0 -> 0.02`로 변경해 높은 Z에서 XY 정확도를 우선
- 실측 Z가 목표 허용범위 밖이면 완료하지 않는 선택형 gate 추가

`123.6mm`도 검토했지만 joint4 한계에서 계획 XY 오차가 약 27mm로 증가해 사용하지 않았다. 최종 100mm 설정의 오프라인 IK 결과는 계획 Z 99.0mm, 경로 최저 Z 99.0mm, XY 오차 6.09mm, pitch 78.0deg다. 실제 로봇 검증은 새 coordinator launch로 재시작한 뒤 수행해야 한다.

## 2026-08-11 taller-box pick/place Z 최종 요청값

- Pick 목표 Z: `57mm` (직전 52mm에서 5mm 상승)
- Pick 완료범위: `50-63mm`
- Place release 목표 Z: `67mm` (pick 목표보다 10mm 높음)
- Place 완료범위: `65-70mm`
- Place XY/pitch transfer 목표는 `100mm`, 실측 하한은 `75mm`로 유지

관련 YAML 파싱과 `omx_box_control` 빌드는 통과했다. 실행 중인 coordinator/stage node는 파라미터 파일을 자동 재로딩하지 않으므로 실제 검증 전에 launch를 완전히 재시작해야 한다.

## 2026-08-11 초기 staging 그리퍼 close 추가

초기 staging 궤적 완료 후 즉시 `WAIT_PICK_TARGET`으로 넘어가지 않고 `WAIT_STAGING_GRIPPER_CLOSE`에서 그리퍼 close action을 실행한다. 실제 gripper feedback이 `0.05rad` 이하일 때만 staging 완료 및 target 선택 상태로 전환한다. Return staging 뒤의 기존 final close는 그대로 유지한다.

## 2026-08-11 high-Z place XY transfer 실물 시험

Place를 `loaded lift -> high-Z XY-only transfer -> Z/pitch alignment -> place descent`로 분리했다. 오프라인 계획에서 high-Z transfer는 Z 130.0mm, XY 오차 0mm였고 후속 alignment는 Z 99.0mm, XY 오차 6.08mm, pitch 78deg였다.

첫 실물 high-Z transfer는 계획 130mm에 대해 실제 Z 110.6mm, XY 오차 2.71mm, 관절 오차 2.42deg였다. 초기 하한 120mm 때문에 실패했지만 loaded-lift 검증범위 안의 안전한 결과였다. 같은 절대 목표를 반복하자 hysteresis로 Z가 99.4mm까지 내려갔으므로 high-Z 명령은 반복하지 않고 첫 결과를 인정하도록 최종 gate를 `min_final_z=105mm`, `actual_target_z_tolerance=25mm`로 설정했다.

현재 자세에서 후속 alignment를 검증한 결과 Z 83.2mm, pitch 80.33deg, XY 오차 4.86mm였다. 이어 place descent는 Z 66.6mm, 실제 하강 16.6mm, pitch 82.88deg로 목표범위 65-70mm 안에서 완료했다. 그리퍼 open과 staging 복귀도 성공했다. 이번 cycle은 high-Z gate 복구 과정에서 수동 전달을 사용했으므로 최종 설정으로 완전 자동 연속 cycle을 한 번 더 검증해야 한다.

## 2026-08-11 7mm 낮은 박스 설정

직전 박스보다 7mm 낮은 시험 박스에 맞춰 pick/place 하강값과 관련 gate를 모두 7mm 내렸다. Pick 목표는 50mm, 완료범위는 43-56mm다. Place release 목표는 pick보다 10mm 높은 60mm, 완료범위는 58-63mm다. Place high-Z XY transfer 설정은 변경하지 않았다. Grasp recovery 범위도 pick 완료범위와 동일한 43-56mm로 갱신했다.

추가 실측 요청으로 다시 5mm 낮췄다. 최종 Pick 목표는 45mm, 완료범위는 38-51mm이며 Place 목표는 55mm, 완료범위는 53-58mm다. Grasp recovery 범위도 38-51mm로 동기화했다.

추가 요청으로 다시 5mm 낮췄다. 현재 Pick 목표는 40mm, 완료범위는 33-46mm이며 Place 목표는 50mm, 완료범위는 48-53mm다. Grasp recovery 범위는 33-46mm다.

## 2026-08-11 gripper close watchdog

ID16이 박스 접촉 후 action을 계속 EXECUTING으로 유지하는 문제를 coordinator에서 처리한다. Pick close가 3초 이상 끝나지 않고 실제 gripper 위치가 grasp 범위에서 0.5초 동안 0.003rad 이내로 안정되면 watchdog이 해당 action을 취소하고 측정된 접촉을 정상 grasp로 인정해 `WAIT_GRASP_CONFIRM`으로 전환한다. 빈 staging close와 place open에는 이 watchdog을 적용하지 않는다.

## 2026-08-11 RViz pick 위험영역

Pitch pregrasp IK를 Z 130mm, pitch 70-80deg, joint margin 조건으로 반경 sweep했다. 유효 반경 280mm까지는 안전, 280-285mm는 주의, 285mm 초과는 위험으로 분류한다. 실패 target `(0.2724, 0.0518)`은 유효 반경 약 287mm였다. Homography 기준 사각형을 10mm 셀로 나눠 `/camera_workspace_markers` MarkerArray에 녹색/노랑/빨강으로 발행하고, 선택 target marker도 영역 색상을 사용한다.
