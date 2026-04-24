#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build the record_for_slam package, optionally with ONNX Runtime configured."
    )
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[3],
        help="Path to the colcon workspace root.",
    )
    parser.add_argument(
        "--package",
        default="record_for_slam",
        help="Package name passed to colcon --packages-select.",
    )
    parser.add_argument(
        "--onnxruntime-root",
        type=Path,
        default=None,
        help="Root directory of the ONNX Runtime release. Overrides auto-discovery.",
    )
    parser.add_argument(
        "--disable-yolo",
        action="store_true",
        help="Build without YOLO inference support, even if ONNX Runtime is installed.",
    )
    parser.add_argument(
        "--cmake-clean-cache",
        action="store_true",
        default=True,
        help="Pass --cmake-clean-cache to colcon build.",
    )
    parser.add_argument(
        "--no-cmake-clean-cache",
        dest="cmake_clean_cache",
        action="store_false",
        help="Do not pass --cmake-clean-cache.",
    )
    parser.add_argument(
        "colcon_args",
        nargs=argparse.REMAINDER,
        help="Extra arguments forwarded to colcon build. Prefix with '--'.",
    )
    return parser.parse_args()


def prepend_env_path(env: dict[str, str], key: str, value: Path) -> None:
    existing = env.get(key, "")
    env[key] = f"{value}:{existing}" if existing else str(value)


def candidate_onnxruntime_roots() -> list[Path]:
    candidates: list[Path] = []

    for env_var in ("ONNXRUNTIME_ROOT", "ORT_ROOT"):
        raw = os.environ.get(env_var)
        if raw:
            candidates.append(Path(raw))

    candidates.extend(
        [
            Path("/usr/local/onnxruntime"),
            Path("/opt/onnxruntime"),
            Path.home() / ".local" / "onnxruntime",
        ]
    )

    expanded: list[Path] = []
    for candidate in candidates:
        if candidate.name.startswith("onnxruntime-linux-") or candidate.name.startswith("onnxruntime-win-"):
            expanded.append(candidate)
            continue

        if candidate.exists() and candidate.is_dir():
            expanded.append(candidate)
            for child in sorted(candidate.iterdir()):
                if child.is_dir() and child.name.startswith("onnxruntime-"):
                    expanded.append(child)

    unique: list[Path] = []
    seen: set[str] = set()
    for path in expanded:
        key = str(path)
        if key not in seen:
            seen.add(key)
            unique.append(path)
    return unique


def resolve_onnxruntime_root(explicit_root: Path | None) -> Path | None:
    if explicit_root is not None:
        return explicit_root.resolve()

    for candidate in candidate_onnxruntime_roots():
        cmake_dir = candidate / "lib" / "cmake" / "onnxruntime"
        if cmake_dir.exists():
            return candidate.resolve()
    return None


def validate_workspace(workspace: Path) -> None:
    if not workspace.exists():
        raise FileNotFoundError(f"Workspace does not exist: {workspace}")
    if not (workspace / "src").exists():
        raise FileNotFoundError(
            f"Workspace src/ directory does not exist: {workspace / 'src'}"
        )


def check_ninja() -> None:
    if shutil.which("ninja") is None:
        raise RuntimeError(
            "Ninja build system not found. Install it first:\n"
            "  Ubuntu/Debian: sudo apt install ninja-build\n"
            "  macOS:         brew install ninja"
        )


def validate_onnxruntime_root(onnxruntime_root: Path) -> None:
    if not onnxruntime_root.exists():
        raise FileNotFoundError(
            f"ONNX Runtime root does not exist: {onnxruntime_root}"
        )

    cmake_dir = onnxruntime_root / "lib" / "cmake" / "onnxruntime"
    library_dir = onnxruntime_root / "lib"
    if not cmake_dir.exists():
        raise FileNotFoundError(
            f"onnxruntime CMake config directory does not exist: {cmake_dir}"
        )
    if not library_dir.exists():
        raise FileNotFoundError(
            f"onnxruntime lib directory does not exist: {library_dir}"
        )


def build_command(
    args: argparse.Namespace, onnxruntime_root: Path | None
) -> list[str]:
    command = [
        "colcon",
        "build",
        "--packages-select",
        args.package,
    ]
    if args.cmake_clean_cache:
        command.append("--cmake-clean-cache")

    command.extend(["--cmake-args"])
    command.append("-GNinja")
    if args.disable_yolo:
        command.append("-DRECORD_FOR_SLAM_ENABLE_YOLO=OFF")
    elif onnxruntime_root is not None:
        cmake_dir = onnxruntime_root / "lib" / "cmake" / "onnxruntime"
        command.extend(
            [
                "-DRECORD_FOR_SLAM_ENABLE_YOLO=ON",
                f"-Donnxruntime_DIR={cmake_dir}",
            ]
        )
    else:
        command.append("-DRECORD_FOR_SLAM_ENABLE_YOLO=ON")

    forwarded = list(args.colcon_args)
    if forwarded and forwarded[0] == "--":
        forwarded = forwarded[1:]
    command.extend(forwarded)
    return command


def main() -> int:
    args = parse_args()
    workspace = args.workspace.resolve()
    validate_workspace(workspace)
    check_ninja()

    onnxruntime_root: Path | None = None
    if not args.disable_yolo:
        onnxruntime_root = resolve_onnxruntime_root(args.onnxruntime_root)
        if args.onnxruntime_root is not None:
            validate_onnxruntime_root(onnxruntime_root)

    env = os.environ.copy()
    if onnxruntime_root is not None:
        env["ONNXRUNTIME_ROOT"] = str(onnxruntime_root)
        prepend_env_path(env, "CMAKE_PREFIX_PATH", onnxruntime_root)
        prepend_env_path(env, "LD_LIBRARY_PATH", onnxruntime_root / "lib")

    command = build_command(args, onnxruntime_root)

    print(f"workspace: {workspace}")
    print(f"cmake_clean_cache: {args.cmake_clean_cache}")
    if args.disable_yolo:
        print("onnxruntime: disabled by --disable-yolo")
    elif onnxruntime_root is not None:
        print(f"onnxruntime_root: {onnxruntime_root}")
    else:
        print("onnxruntime_root: not found; CMake will build without YOLO support unless you pass --onnxruntime-root")
    print("command:", " ".join(command))

    completed = subprocess.run(command, cwd=workspace, env=env, check=False)
    return completed.returncode


if __name__ == "__main__":
    sys.exit(main())
