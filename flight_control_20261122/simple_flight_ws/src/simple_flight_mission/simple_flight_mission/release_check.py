"""Read-only release file checks. Never imports ROS or authorizes flight."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re


def check_release(root, manifest):
    root = Path(root).resolve()
    failures, verified, seen = [], 0, set()
    entries = manifest.get("files") if isinstance(manifest, dict) else None
    if (not isinstance(manifest, dict) or manifest.get("schema_version") != 1
            or not isinstance(entries, list) or not entries):
        failures.append(dict(reason="invalid_or_empty_manifest"))
        entries = []
    for entry in entries:
        name = entry.get("path") if isinstance(entry, dict) else None
        digest = entry.get("sha256") if isinstance(entry, dict) else None
        if (not isinstance(name, str) or not name or "\\" in name or ":" in name
                or PurePosixPath(name).is_absolute() or ".." in PurePosixPath(name).parts
                or name in seen or not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest)):
            failures.append(dict(path=name, reason="invalid_or_duplicate_entry"))
            continue
        seen.add(name)
        path = (root/name).resolve()
        if not path.is_relative_to(root):
            failures.append(dict(path=name, reason="path_outside_release_root"))
            continue
        try:
            hasher = hashlib.sha256()
            with path.open("rb") as handle:
                for chunk in iter(lambda: handle.read(1024*1024), b""):
                    hasher.update(chunk)
            actual = hasher.hexdigest()
        except OSError as error:
            failures.append(dict(path=name, reason="unreadable_or_missing", error=str(error)))
            continue
        if actual != digest:
            failures.append(dict(path=name, reason="hash_mismatch", expected=digest, actual=actual))
        else:
            verified += 1
    return dict(passed=not failures, files_verified=verified, files_expected=len(entries), failures=failures,
                scope="release_file_integrity_only", runtime_installation_verified=False, flight_authorized=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description="只读校验发布文件哈希；不检查飞行准备、不运行ROS、不授权起飞")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    result = check_release(args.root, manifest)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
