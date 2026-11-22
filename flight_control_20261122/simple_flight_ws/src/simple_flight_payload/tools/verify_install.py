"""Nano-only cache verification/repair for these two pure-Python packages."""
import argparse
import hashlib
from pathlib import Path
import py_compile
import shutil
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=Path.home() / "simple_flight_ws")
    parser.add_argument("--repair", action="store_true")
    args = parser.parse_args()
    root = args.workspace.resolve(strict=True)
    if root.name != "simple_flight_ws":
        parser.error("Expected the explicitly named simple_flight_ws workspace")
    changed = []
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"

    def sync(source, destination):
        allowed = root / ("build" if "build" in destination.relative_to(root).parts[:1] else "install")
        if not destination.parent.resolve().is_relative_to(allowed.resolve()):
            raise RuntimeError("Destination escaped generated workspace directories")
        data = source.read_bytes()
        if destination.exists() and hashlib.sha256(data).digest() == hashlib.sha256(destination.read_bytes()).digest():
            return
        if not args.repair:
            raise RuntimeError("Stale/missing generated file: " + str(destination))
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(destination.name + ".codex_tmp")
        shutil.copy2(source, temporary)
        temporary.replace(destination)
        if destination.suffix == ".py":
            py_compile.compile(str(destination), doraise=True)
        changed.append(str(destination.relative_to(root)))

    for package in ("simple_flight_payload", "simple_flight_mission"):
        source = root / "src" / package
        for path in (source / package).glob("*.py"):
            sync(path, root / "build" / package / "build/lib" / package / path.name)
            sync(path, root / "install" / package / "lib" / version / "site-packages" / package / path.name)
        for folder in ("config", "scripts"):
            for path in (source / folder).glob("*"):
                if path.is_file():
                    sync(path, root / "install" / package / "share" / package / folder / path.name)
    print("Source/build/install verified; refreshed files:", changed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
