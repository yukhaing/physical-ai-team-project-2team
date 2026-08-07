# Beagle 시뮬레이터 (Windows, 실물 로봇 불필요)

맵 구성 + 키보드 주행(teleop) + SLAM(지도 작성)을 한 화면에서 실습합니다.
왼쪽 창은 "실제 세계"(신의 시점), 오른쪽 창은 "로봇이 만든 지도"입니다.

## 실행

```powershell
# 기본 장면 (방 + 장애물 2개)
python simulator\beagle_sim.py

# 프리셋 장면
python simulator\beagle_sim.py --scene room_exit
python simulator\beagle_sim.py --scene corridor

# 직접 만든 맵
python simulator\beagle_sim.py --map simulator\maps\sample_room.json

# SLAM(스캔 정합 보정) 켜기 — 오도메트리만일 때와 지도 품질 비교
python simulator\beagle_sim.py --slam
python simulator\beagle_sim.py --odom-noise 0.15          # 오차 크게
python simulator\beagle_sim.py --odom-noise 0.15 --slam   # 보정 효과 확인
```

## 조작 (그래프 창 클릭 후)

| 키 | 동작 |
|---|---|
| ↑ ↓ ← → 또는 W S A D | 전진 / 후진 / 좌회전 / 우회전 |
| SPACE | 정지 |
| M | 지도 저장 (map_sim.png / .csv / .yaml) |
| R | 지도 초기화 |
| Q | 종료 |

## 맵 에디터

```powershell
python simulator\beagle_sim.py --edit simulator\maps\my_map.json
```

- 왼쪽 클릭 2번 = 벽 하나 추가
- 오른쪽 클릭 = 로봇 시작 위치
- U = 마지막 벽 취소, S = 저장, Q = 종료
- 저장한 맵은 `--map` 옵션으로 불러옵니다.

## 수업 아이디어

1. **오도메트리의 한계 체험**: `--odom-noise 0.15`로 주행하면 벽이 여러 겹으로 번집니다.
   왼쪽 창에서 빨간 원(실제 위치)과 파란 X(로봇의 추정 위치)가 벌어지는 것을 관찰하세요.
2. **SLAM 효과**: 같은 조건에 `--slam`을 붙이면 스캔 정합이 pose를 되돌려
   벽이 얇게 유지됩니다. `pose err` 수치로 비교하세요.
3. **맵 설계 미션**: 맵 에디터로 미로를 만들어 친구 팀과 교환하고,
   전체를 탐색해 지도 완성도를 겨루세요 (M으로 저장해 제출).
