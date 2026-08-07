"""Beagle 시뮬레이터 GUI 런처 — 명령어 없이 모든 모드를 선택합니다.

실행:  python simulator\sim_launcher.py   (또는 run_menu의 0번)

모드
  1) 주행 시뮬레이터  : teleop + 클릭 자율주행(A*+Pure Pursuit) + SLAM
  2) 맵 에디터        : 마우스로 벽을 그려 JSON 맵 저장
  3) A* 경로 데모     : sample_map.csv에서 경로 계획 결과 보기
  4) Pure Pursuit 데모: 계획된 경로를 따라가는 궤적 보기
"""
from __future__ import annotations

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

ROOT = Path(__file__).resolve().parents[1]
SCENES = ['default', 'room', 'room_exit', 'corridor']
NOISES = [0.03, 0.06, 0.10, 0.15]


def run_sim(config: dict) -> None:
    from simulator.beagle_sim import BeagleSimulator, load_map_json
    from common.beagle_safe import build_scene
    if config.get('map_file'):
        segments, start = load_map_json(config['map_file'])
    else:
        segments, start = build_scene(config['scene'])
    sim = BeagleSimulator(segments, start,
                          odom_noise=config['noise'], use_slam=config['slam'])
    sim.run()


def run_editor(config: dict) -> None:
    from simulator.beagle_sim import run_editor
    save = config.get('map_file') or str(ROOT / 'simulator' / 'maps' / 'my_map.json')
    if not save.endswith('.json'):
        save += '.json'
    run_editor(save)


def run_astar_demo(config: dict) -> None:
    import matplotlib.pyplot as plt
    import numpy as np
    from common.mapping import inflate_obstacles, load_map_csv
    from common.planning import astar, reduce_waypoints
    map_csv = ROOT / 'data' / 'sample_map.csv'
    if not map_csv.is_file():
        print('data/sample_map.csv 없음 -> python tools\\generate_sample_map.py 먼저 실행')
        return
    grid = inflate_obstacles(load_map_csv(map_csv), 2)
    start, goal = (6, 6), (70, 47)
    path = astar(grid, start, goal)
    waypoints = reduce_waypoints(grid, path)
    plt.figure(figsize=(8, 6))
    plt.imshow(grid, origin='lower', cmap='gray_r')
    if path:
        px, py = zip(*path); plt.plot(px, py, linewidth=2, label=f'A* path ({len(path)}칸)')
        wx, wy = zip(*waypoints); plt.scatter(wx, wy, s=70, label=f'waypoints ({len(waypoints)}개)')
    plt.scatter([start[0], goal[0]], [start[1], goal[1]], s=90, label='start/goal')
    plt.legend(); plt.grid(True); plt.title('A* on sample_map.csv')
    plt.tight_layout(); plt.show()


def run_pp_demo(config: dict) -> None:
    import matplotlib.pyplot as plt
    from common.geometry import Pose2D, integrate_differential_drive
    from common.mapping import inflate_obstacles, load_map_csv
    from common.planning import astar, grid_path_to_world, pure_pursuit_command, reduce_waypoints
    map_csv = ROOT / 'data' / 'sample_map.csv'
    if not map_csv.is_file():
        print('data/sample_map.csv 없음 -> python tools\\generate_sample_map.py 먼저 실행')
        return
    grid = inflate_obstacles(load_map_csv(map_csv), 2)
    path = grid_path_to_world(reduce_waypoints(grid, astar(grid, (6, 6), (70, 47))), 0.05)
    lookahead = config.get('lookahead', 0.25)
    pose = Pose2D(path[0][0], path[0][1], 0.0)
    xs, ys = [pose.x], [pose.y]
    for _ in range(2500):
        v, omega, _ = pure_pursuit_command(pose, path, lookahead_m=lookahead, speed_mps=0.11)
        left = v - omega * 0.0956 / 2.0
        right = v + omega * 0.0956 / 2.0
        pose = integrate_differential_drive(pose, left, right, 0.05)
        xs.append(pose.x); ys.append(pose.y)
        if math.hypot(pose.x - path[-1][0], pose.y - path[-1][1]) < 0.08:
            break
    px, py = zip(*path)
    plt.figure(figsize=(8, 6))
    plt.plot(px, py, 'o--', label='planned waypoints')
    plt.plot(xs, ys, label=f'trajectory (lookahead={lookahead})')
    plt.axis('equal'); plt.grid(True); plt.legend(); plt.title('Pure Pursuit demo')
    plt.tight_layout(); plt.show()


def show_launcher() -> dict | None:
    """Tk 창을 띄워 설정을 받고, 시작을 누르면 config dict를 반환합니다."""
    try:
        import tkinter as tk
        from tkinter import filedialog, ttk
    except ImportError:
        print('tkinter를 사용할 수 없어 기본 설정으로 시뮬레이터를 실행합니다.')
        return {'mode': 'sim', 'scene': 'default', 'map_file': None, 'noise': 0.06, 'slam': True}

    result: dict = {}
    root = tk.Tk()
    root.title('Beagle 시뮬레이터 런처')
    root.geometry('460x460')
    root.resizable(False, False)

    pad = {'padx': 12, 'pady': 4}
    tk.Label(root, text='Beagle 시뮬레이터', font=('Malgun Gothic', 15, 'bold')).pack(pady=(14, 2))
    tk.Label(root, text='모드와 옵션을 고르고 [시작]을 누르세요', fg='#666').pack()

    mode_var = tk.StringVar(value='sim')
    frame_mode = tk.LabelFrame(root, text=' 모드 ', font=('Malgun Gothic', 10, 'bold'))
    frame_mode.pack(fill='x', **pad)
    for val, txt in [('sim', '주행 시뮬레이터 (클릭=A* 자율주행, 키보드=수동)'),
                     ('edit', '맵 에디터 (마우스로 벽 그리기)'),
                     ('astar', 'A* 경로 계획 데모 (정적)'),
                     ('pp', 'Pure Pursuit 경로 추종 데모 (정적)')]:
        tk.Radiobutton(frame_mode, text=txt, variable=mode_var, value=val,
                       anchor='w').pack(fill='x', padx=8)

    frame_opt = tk.LabelFrame(root, text=' 옵션 ', font=('Malgun Gothic', 10, 'bold'))
    frame_opt.pack(fill='x', **pad)

    row1 = tk.Frame(frame_opt); row1.pack(fill='x', padx=8, pady=3)
    tk.Label(row1, text='장면(프리셋):').pack(side='left')
    scene_var = tk.StringVar(value='default')
    ttk.Combobox(row1, textvariable=scene_var, values=SCENES, width=12,
                 state='readonly').pack(side='left', padx=6)

    map_var = tk.StringVar(value='')
    row2 = tk.Frame(frame_opt); row2.pack(fill='x', padx=8, pady=3)
    tk.Label(row2, text='JSON 맵:').pack(side='left')
    tk.Entry(row2, textvariable=map_var, width=30).pack(side='left', padx=6)

    def browse():
        f = filedialog.askopenfilename(initialdir=str(ROOT / 'simulator' / 'maps'),
                                       filetypes=[('JSON map', '*.json')])
        if f:
            map_var.set(f)
    tk.Button(row2, text='찾기', command=browse).pack(side='left')

    row3 = tk.Frame(frame_opt); row3.pack(fill='x', padx=8, pady=3)
    tk.Label(row3, text='오도메트리 노이즈:').pack(side='left')
    noise_var = tk.DoubleVar(value=0.06)
    for nv in NOISES:
        tk.Radiobutton(row3, text=str(nv), variable=noise_var, value=nv).pack(side='left')

    slam_var = tk.BooleanVar(value=True)
    tk.Checkbutton(frame_opt, text='SLAM(스캔 정합 보정) 켜기 — 주행 중 E키로도 전환 가능',
                   variable=slam_var, anchor='w').pack(fill='x', padx=8, pady=3)

    row4 = tk.Frame(frame_opt); row4.pack(fill='x', padx=8, pady=3)
    tk.Label(row4, text='Pure Pursuit lookahead(m):').pack(side='left')
    la_var = tk.DoubleVar(value=0.25)
    for lv in (0.10, 0.25, 0.50):
        tk.Radiobutton(row4, text=str(lv), variable=la_var, value=lv).pack(side='left')

    def start():
        result.update({
            'mode': mode_var.get(),
            'scene': scene_var.get(),
            'map_file': map_var.get() or None,
            'noise': float(noise_var.get()),
            'slam': bool(slam_var.get()),
            'lookahead': float(la_var.get()),
        })
        root.destroy()

    def quit_all():
        result['mode'] = None
        root.destroy()

    btns = tk.Frame(root); btns.pack(pady=12)
    tk.Button(btns, text='  시작  ', font=('Malgun Gothic', 11, 'bold'),
              bg='#16C3B2', fg='white', command=start).pack(side='left', padx=8)
    tk.Button(btns, text='  종료  ', command=quit_all).pack(side='left', padx=8)
    tk.Label(root, text='시뮬레이터 조작: 화면 하단 안내 참고 (클릭=목표, G=취소, Q=종료)',
             fg='#888').pack(side='bottom', pady=6)

    root.mainloop()
    return result if result.get('mode') else None


def main() -> None:
    runners = {'sim': run_sim, 'edit': run_editor, 'astar': run_astar_demo, 'pp': run_pp_demo}
    while True:
        config = show_launcher()
        if config is None:
            print('런처 종료')
            break
        runners[config['mode']](config)


if __name__ == '__main__':
    main()
