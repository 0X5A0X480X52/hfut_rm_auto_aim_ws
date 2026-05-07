from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class Config:
    raw: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "Config":
        with Path(path).open("r", encoding="utf-8") as f:
            return cls(yaml.safe_load(f))

    def section(self, name: str) -> dict[str, Any]:
        return dict(self.raw.get(name, {}))

