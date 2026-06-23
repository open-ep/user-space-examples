#!/usr/bin/env python3
"""Generate the GitHub Actions build matrix from discovered C targets."""

import importlib.util
import json
from pathlib import Path


IMAGE_BY_GPIOD = {
    "v1": "debian:bookworm",
    "v2": "debian:trixie",
}


def load_discover_module():
    module_path = Path(__file__).with_name("discover-targets.py")
    spec = importlib.util.spec_from_file_location("discover_targets", module_path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load target discovery module: {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    discover_targets = load_discover_module().discover_targets
    repo_root = Path(__file__).resolve().parents[1]

    matrix = []
    for target in discover_targets(repo_root):
        gpiod = target["gpiod"]
        try:
            image = IMAGE_BY_GPIOD[gpiod]
        except KeyError as exc:
            raise SystemExit(f"unsupported libgpiod API version: {gpiod}") from exc

        matrix.append({**target, "image": image})

    print(f"matrix={json.dumps({'include': matrix}, separators=(',', ':'))}")


if __name__ == "__main__":
    main()
