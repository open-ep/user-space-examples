#!/usr/bin/env python3
"""Discover C build targets for GitHub Actions."""

import argparse
import json
import re
import sys
from pathlib import Path


ARCH_BY_BOARD = {
    "d3-g": "linux/arm64",
    "frdm-imx93": "linux/arm64",
    "imx8mp": "linux/arm64",
    "kakip": "linux/arm64",
    "rk3588": "linux/arm64",
    "rpi2": "linux/arm/v7",
    "rpi5": "linux/arm64",
    "topst-d3-g": "linux/arm64",
}

GPIOD_V2_MARKERS = (
    "gpiod_chip_request_lines",
    "gpiod_line_settings_new",
    "gpiod_line_request_set_value",
    "struct gpiod_line_request *",
)

GPIOD_V1_MARKERS = (
    "gpiod_chip_get_line",
    "gpiod_line_request_output",
    "gpiod_line_set_value",
    "struct gpiod_line *",
)

SKIP_DIRS = {
    ".git",
    "build",
}

TARGET_SOURCE_RE = re.compile(r"^pixpaper-\d+-[a-z]+-test-(?P<target>[a-z0-9-]+)\.c$")


class TargetDiscoveryError(Exception):
    """Raised when a source file looks buildable but cannot be classified."""


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def should_skip(path: Path, root: Path) -> bool:
    parts = path.relative_to(root).parts
    return any(part in SKIP_DIRS for part in parts[:-1])


def iter_target_sources(root: Path):
    for path in sorted(root.rglob("*.c")):
        if should_skip(path, root):
            continue
        if TARGET_SOURCE_RE.match(path.name):
            yield path


def detect_gpiod_api(source: str, path: str) -> str:
    if any(marker in source for marker in GPIOD_V2_MARKERS):
        return "v2"
    if any(marker in source for marker in GPIOD_V1_MARKERS):
        return "v1"
    raise TargetDiscoveryError(
        f"{path}: cannot detect libgpiod API version; add a v1/v2 marker"
    )


def detect_board(target_suffix: str, path: str) -> str:
    for board in sorted(ARCH_BY_BOARD, key=len, reverse=True):
        if target_suffix == board or target_suffix.startswith(f"{board}-"):
            return board

    known = ", ".join(sorted(ARCH_BY_BOARD))
    raise TargetDiscoveryError(
        f"{path}: unknown board in target suffix '{target_suffix}'; "
        f"add it to ARCH_BY_BOARD (known: {known})"
    )


def target_from_source(path: Path, root: Path) -> dict:
    match = TARGET_SOURCE_RE.match(path.name)
    if not match:
        raise TargetDiscoveryError(f"{path}: unsupported target filename")

    rel_path = path.relative_to(root).as_posix()
    board = detect_board(match.group("target"), rel_path)
    arch = ARCH_BY_BOARD[board]

    source = path.read_text(encoding="utf-8", errors="ignore")
    return {
        "name": path.stem.replace("-test-", "-", 1),
        "src": rel_path,
        "arch": arch,
        "gpiod": detect_gpiod_api(source, rel_path),
    }


def discover_targets(root: Path | None = None) -> list[dict]:
    root = root or repo_root()
    errors = []
    targets = []

    for path in iter_target_sources(root):
        try:
            targets.append(target_from_source(path, root))
        except TargetDiscoveryError as exc:
            errors.append(str(exc))

    if errors:
        raise TargetDiscoveryError("\n".join(errors))

    return sorted(targets, key=lambda target: target["src"])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--compact",
        action="store_true",
        help="write compact JSON instead of pretty-printed JSON",
    )
    args = parser.parse_args()

    try:
        targets = discover_targets()
    except TargetDiscoveryError as exc:
        print(exc, file=sys.stderr)
        return 1

    indent = None if args.compact else 2
    separators = (",", ":") if args.compact else None
    print(json.dumps({"targets": targets}, indent=indent, separators=separators))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
