#!/usr/bin/env python3
"""Run host regression tests with sanitizers and the installed BTstack headers."""
import os
from pathlib import Path
import shlex
import runpy
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
framework = Path(os.environ.get(
    "ARDUINO_PICO_DIR",
    str(Path.home() / ".platformio/packages/framework-arduinopico"),
))
btstack = framework / "pico-sdk/lib/btstack/src"
if not (btstack / "l2cap.h").exists():
    raise SystemExit("Run 'pio run' first, or set ARDUINO_PICO_DIR to the Arduino-Pico framework directory.")

compiler = shlex.split(os.environ.get("CXX", "g++"))
flags = [
    "-std=c++17", "-g", "-O1", "-Wall", "-Wextra", "-Werror",
    "-fsanitize=address,undefined,float-cast-overflow",
    "-fno-sanitize-recover=all", "-fno-omit-frame-pointer",
    "-I" + str(root / "test/native/stubs"), "-I" + str(root / "include"),
]
suites = {
    "debug_log": [],
    "identify_led": [],
    "input_voltage": ["src/input_voltage.cpp"],
    "protocol": ["src/wpilib_protocol.cpp", "src/byteutils.cpp", "src/watchdog.cpp",
                 "src/XRPServo.cpp"],
    "config": ["src/config.cpp"],
    "transport": ["src/bluetooth_transport.cpp", "src/main.cpp", "src/config.cpp",
                  "src/wpilib_protocol.cpp", "src/byteutils.cpp", "src/watchdog.cpp",
                  "src/input_voltage.cpp"],
}
with tempfile.TemporaryDirectory(prefix="xrp-native-tests-") as build_dir:
    for suite, sources in suites.items():
        extra_flags = []
        if suite == "transport":
            extra_flags = ["-DENABLE_BLE", "-isystem", str(btstack),
                           "-isystem", str(framework / "include/rp2040")]
        executable = str(Path(build_dir) / suite)
        command = compiler + flags + extra_flags + [
            str(root / ("test/native/" + suite + "_test.cpp")),
            *(str(root / source) for source in sources),
            str(root / "src/debug_log.cpp"), str(root / "src/debug_log_usb.cpp"),
            "-o", executable,
        ]
        subprocess.run(command, check=True)
        subprocess.run([executable], check=True)

# Compile the same patched BTstack source used by the firmware. Unused radio
# functions are discarded by the linker; allocation and channel lookup are real.
patch_l2cap = runpy.run_path(str(root / "tools/btstack_l2cap.py"))["patch_l2cap"]
with tempfile.TemporaryDirectory(prefix="xrp-btstack-tests-") as build_dir:
    generated = Path(build_dir) / "l2cap.c"
    generated.write_text(patch_l2cap((btstack / "l2cap.c").read_text()))
    executable = str(Path(build_dir) / "l2cap_cid")
    command = shlex.split(os.environ.get("CC", "gcc")) + [
        "-std=c11", "-g", "-O1", "-Wall", "-Wextra",
        "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
        "-fno-omit-frame-pointer", "-ffunction-sections", "-fdata-sections",
        "-Wl,--gc-sections", "-DENABLE_BLE", "-DENABLE_CLASSIC",
        "-I" + build_dir, "-isystem", str(btstack),
        "-isystem", str(framework / "include/rp2040"),
        str(root / "test/native/l2cap_cid_test.c"),
        str(btstack / "btstack_linked_list.c"), str(btstack / "btstack_util.c"),
        "-o", executable,
    ]
    subprocess.run(command, check=True)
    subprocess.run([executable], check=True)
