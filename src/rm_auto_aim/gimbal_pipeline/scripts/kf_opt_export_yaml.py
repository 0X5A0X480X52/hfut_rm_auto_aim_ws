#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Export best_params.json to ROS2 YAML patch")
    p.add_argument("--params-json", required=True)
    p.add_argument("--out-yaml", required=True)
    p.add_argument("--node-name", default="gimbal_pipeline")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    with open(args.params_json, "r", encoding="utf-8") as f:
        params = json.load(f)

    lines = [
        f"/{args.node_name}:",
        "  ros__parameters:",
    ]

    for key, value in sorted(params.items()):
        lines.append(f"    {key}: {value}")

    out_path = Path(args.out_yaml)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(str(out_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
