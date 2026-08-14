from __future__ import annotations

import heapq
import math

import numpy as np

GridPoint = tuple[int, int]
Key = tuple[float, float]

_BLOCKED_THRESHOLD = 65


def _heuristic(a: GridPoint, b: GridPoint) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class DStarLite:
    """증분 재계획(incremental replanning) 경로 탐색 (Koenig & Likhachev, "D* Lite", 2002).

    common/planning.py의 astar()는 막힐 때마다 그래프 전체를 처음부터 다시 탐색합니다.
    D* Lite는 목표(goal)에서부터 거꾸로 탐색해두고, 로봇이 이동하거나(update_start)
    새 장애물이 발견되면(update_cell) 바뀐 부분 주변만 갱신합니다 -- 매번 전체를
    다시 계산하지 않아도 됩니다.

    구현 단순화: 교과서의 최적화된 버전은 UpdateVertex에서 "이 이웃의 rhs가 이
    간선 비용에 의존하고 있었는가"를 추적해서 필요한 예측자(predecessor)만
    갱신합니다. 여기서는 UpdateVertex(u)가 항상 rhs(u)를 이웃들로부터 처음부터
    다시 계산하는 더 단순한 버전을 씁니다 -- 그리드가 작으므로(수십~수백 칸)
    성능 차이는 무시할 만하고, 버그 낼 가능성이 훨씬 적습니다. g/rhs가 수렴하는
    고정점은 동일하므로 결과의 정확성은 동일합니다.

    사용법:
        planner = DStarLite(grid, start, goal)
        planner.compute_shortest_path()
        path = planner.extract_path()

        # 로봇이 한 칸 이동했을 때:
        planner.update_start(new_start)
        planner.compute_shortest_path()

        # 새 장애물을 발견했을 때(그리드 값 변경):
        planner.update_cell(cell, new_value)
        planner.compute_shortest_path()
        path = planner.extract_path()
    """

    def __init__(self, grid: np.ndarray, start: GridPoint, goal: GridPoint, *, allow_diagonal: bool = True) -> None:
        self.grid = grid
        self.height, self.width = grid.shape
        self.start = start
        self.goal = goal
        self.allow_diagonal = allow_diagonal
        self.km = 0.0
        self.g: dict[GridPoint, float] = {}
        self.rhs: dict[GridPoint, float] = {}
        self._in_queue: dict[GridPoint, Key] = {}
        self._heap: list[tuple[Key, GridPoint]] = []
        self.expansions = 0  # 마지막 compute_shortest_path() 호출에서 꺼낸(pop) 노드 수

        self.rhs[self.goal] = 0.0
        self._push(self.goal, self._calculate_key(self.goal))

    # ---------------- internal helpers ----------------

    def _blocked(self, point: GridPoint) -> bool:
        x, y = point
        return int(self.grid[y, x]) >= _BLOCKED_THRESHOLD

    def _g(self, s: GridPoint) -> float:
        return self.g.get(s, math.inf)

    def _rhs(self, s: GridPoint) -> float:
        return self.rhs.get(s, math.inf)

    def _calculate_key(self, s: GridPoint) -> Key:
        g_rhs = min(self._g(s), self._rhs(s))
        return (g_rhs + _heuristic(self.start, s) + self.km, g_rhs)

    def _push(self, s: GridPoint, key: Key) -> None:
        self._in_queue[s] = key
        heapq.heappush(self._heap, (key, s))

    def _pop_min(self) -> tuple[Key, GridPoint] | None:
        while self._heap:
            key, s = heapq.heappop(self._heap)
            if s in self._in_queue and self._in_queue[s] == key:
                del self._in_queue[s]
                return key, s
        return None

    def _top_key(self) -> Key:
        while self._heap:
            key, s = self._heap[0]
            if s in self._in_queue and self._in_queue[s] == key:
                return key
            heapq.heappop(self._heap)
        return (math.inf, math.inf)

    def _cost(self, a: GridPoint, b: GridPoint) -> float:
        if self._blocked(a) or self._blocked(b):
            return math.inf
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if dx and dy:
            # diagonal move -- disallow cutting between two blocked orthogonal
            # cells, matching astar()'s prevent_corner_cutting (a robot with
            # real width can't squeeze through a diagonal gap between obstacles)
            if self._blocked((a[0] + dx, a[1])) or self._blocked((a[0], a[1] + dy)):
                return math.inf
            return math.sqrt(2.0)
        return 1.0

    def _neighbors(self, s: GridPoint) -> list[GridPoint]:
        x, y = s
        moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            moves += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        result = []
        for dx, dy in moves:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                result.append((nx, ny))
        return result

    def _update_vertex(self, u: GridPoint) -> None:
        if u != self.goal:
            self.rhs[u] = min(
                (self._cost(u, v) + self._g(v) for v in self._neighbors(u)),
                default=math.inf,
            )
        if u in self._in_queue:
            del self._in_queue[u]  # stale heap entries are skipped lazily in _pop_min/_top_key
        if self._g(u) != self._rhs(u):
            self._push(u, self._calculate_key(u))

    # ---------------- public API ----------------

    def compute_shortest_path(self, max_expansions: int = 200_000) -> None:
        self.expansions = 0
        while self._heap and (
            self._top_key() < self._calculate_key(self.start) or self._rhs(self.start) != self._g(self.start)
        ):
            if self.expansions >= max_expansions:
                raise RuntimeError("D* Lite exceeded max_expansions -- unreachable goal or a bug")
            popped = self._pop_min()
            if popped is None:
                break
            k_old, u = popped
            self.expansions += 1
            k_new = self._calculate_key(u)
            if k_old < k_new:
                self._push(u, k_new)
            elif self._g(u) > self._rhs(u):
                self.g[u] = self._rhs(u)
                for s in self._neighbors(u):
                    self._update_vertex(s)
            else:
                self.g[u] = math.inf
                self._update_vertex(u)
                for s in self._neighbors(u):
                    self._update_vertex(s)

    def update_start(self, new_start: GridPoint) -> None:
        """로봇이 새 위치로 이동했을 때 호출합니다."""
        self.km += _heuristic(self.start, new_start)
        self.start = new_start

    def update_cell(self, cell: GridPoint, new_value: int) -> None:
        """그리드 값이 바뀌었을 때(예: 장애물 새로 발견) 호출합니다."""
        self.grid[cell[1], cell[0]] = new_value
        self._update_vertex(cell)
        for n in self._neighbors(cell):
            self._update_vertex(n)

    def path_cost(self) -> float:
        return self._g(self.start)

    def extract_path(self, max_steps: int = 10_000) -> list[GridPoint]:
        if self._g(self.start) == math.inf:
            return []
        path = [self.start]
        current = self.start
        for _ in range(max_steps):
            if current == self.goal:
                return path
            candidates = self._neighbors(current)
            if not candidates:
                return []
            best = min(candidates, key=lambda n: self._cost(current, n) + self._g(n))
            if self._cost(current, best) == math.inf or math.isinf(self._g(best)):
                return []
            current = best
            path.append(current)
        return []
