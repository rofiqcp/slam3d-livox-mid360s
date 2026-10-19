#!/usr/bin/env python3
"""Reject local research captures, generated files, and notes from Git."""
from __future__ import annotations

import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
files = subprocess.check_output(["git", "-C", str(ROOT), "ls-files", "-z"])
tracked = [pathlib.PurePosixPath(raw.decode()) for raw in files.split(b"\0") if raw]
invalid = []
for path in tracked:
    name = path.name.lower()
    parts = {part.lower() for part in path.parts}
    reason = None
    if parts.intersection({".local-only", "results", "datasets", "backups", "bags", "build", "install", "log", "build_external", "__pycache__"}):
        reason = "dataset, experiment or build artifact"
    elif re.search(r"20\d{6}\.md$", name) or "audit" in name and name.endswith(".md"):
        reason = "dated/historical research note"
    elif name.startswith(".env") and name != ".env.example":
        reason = "private environment file"
    elif name.endswith((".bag", ".db3", ".mcap", ".pcd", ".ply", ".las", ".laz", ".pem", ".p12", ".pfx", ".log")):
        reason = "raw data, log or credential file"
    if reason:
        invalid.append((str(path), reason))
if invalid:
    for path, reason in invalid:
        print(f"ERROR: {path}: {reason}", file=sys.stderr)
    sys.exit(1)
print(f"OK: {len(tracked)} tracked files conform to repository policy")
