#!/usr/bin/env python3
"""Generate the GitHub Actions build matrix from ci/build-targets.json."""

import json
from pathlib import Path


IMAGE_BY_GPIOD = {
    "v1": "debian:bookworm",
    "v2": "debian:trixie",
}


def main() -> None:
    manifest_path = Path(__file__).with_name("build-targets.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    matrix = []
    for target in manifest["targets"]:
        gpiod = target["gpiod"]
        try:
            image = IMAGE_BY_GPIOD[gpiod]
        except KeyError as exc:
            raise SystemExit(f"unsupported libgpiod API version: {gpiod}") from exc

        matrix.append({**target, "image": image})

    print(f"matrix={json.dumps({'include': matrix}, separators=(',', ':'))}")


if __name__ == "__main__":
    main()
