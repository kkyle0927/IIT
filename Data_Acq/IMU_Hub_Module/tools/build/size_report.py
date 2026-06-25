#!/usr/bin/env python3
"""Post-build size report.

Local stub recreated for the Snapshot/HD/IMU_Resolution_For_Swiss build:
the original harness script lives in a separate repo and was not part of the
snapshot. This version simply prints the firmware section sizes via
arm-none-eabi-size and never fails the build.

Usage: python size_report.py <path-to-elf>
"""
import shutil
import subprocess
import sys


def main() -> int:
    if len(sys.argv) < 2:
        print("[size_report] no ELF path given, skipping.")
        return 0

    elf = sys.argv[1]
    size_tool = shutil.which("arm-none-eabi-size")
    if not size_tool:
        print("[size_report] arm-none-eabi-size not found on PATH, skipping.")
        return 0

    try:
        out = subprocess.run(
            [size_tool, "-B", elf],
            capture_output=True,
            text=True,
            check=False,
        )
        if out.stdout:
            print(out.stdout, end="")
        if out.stderr:
            print(out.stderr, end="")
    except Exception as exc:  # never break the build on a report step
        print(f"[size_report] skipped ({exc}).")

    return 0


if __name__ == "__main__":
    # Always exit 0 so a reporting step can never fail the build.
    main()
    sys.exit(0)
