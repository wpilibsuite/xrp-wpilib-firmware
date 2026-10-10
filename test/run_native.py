#!/usr/bin/env python3
"""Run nonblocking serial logging regressions with sanitizers."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
compiler = shlex.split(os.environ.get("CXX", "g++"))
flags = [
    "-std=c++17", "-g", "-O1", "-Wall", "-Wextra", "-Werror",
    "-fsanitize=address,undefined,float-cast-overflow",
    "-fno-sanitize-recover=all", "-fno-omit-frame-pointer",
    "-I" + str(root / "test/native/stubs"), "-I" + str(root / "include"),
]
suites = {"debug_log": []}
with tempfile.TemporaryDirectory(prefix="xrp-native-tests-") as build_dir:
    for suite, sources in suites.items():
        executable = str(Path(build_dir) / suite)
        command = compiler + flags + [
            str(root / ("test/native/" + suite + "_test.cpp")),
            *(str(root / source) for source in sources),
            str(root / "src/debug_log.cpp"), str(root / "src/debug_log_usb.cpp"),
            "-o", executable,
        ]
        subprocess.run(command, check=True)
        subprocess.run([executable], check=True)
