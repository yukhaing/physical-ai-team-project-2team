from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable, Mapping, Sequence


class CsvLogger:
    def __init__(self, path: str | Path, fieldnames: Sequence[str]) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fp = self.path.open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.fp, fieldnames=list(fieldnames), extrasaction="ignore")
        self.writer.writeheader()

    def write(self, row: Mapping[str, object] | None = None, /, **values: object) -> None:
        """한 행을 기록합니다. mapping 또는 키워드 인자를 모두 허용합니다."""

        payload: dict[str, object] = dict(row or {})
        payload.update(values)
        self.writer.writerow(payload)
        self.fp.flush()

    def close(self) -> None:
        if not self.fp.closed:
            self.fp.close()

    def __enter__(self) -> "CsvLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def save_rows(path: str | Path, fieldnames: Sequence[str], rows: Iterable[Mapping[str, object]]) -> Path:
    with CsvLogger(path, fieldnames) as logger:
        for row in rows:
            logger.write(row)
    return Path(path)
