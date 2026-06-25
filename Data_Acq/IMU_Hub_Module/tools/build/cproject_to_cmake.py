#!/usr/bin/env python3
"""Post-build .cproject -> CMakeLists sync.

Local stub recreated for the Snapshot/HD/IMU_Resolution_For_Swiss build:
the original harness script (which regenerates CMakeLists.txt from the CubeIDE
managed-build .cproject) lives in a separate repo and was not part of the
snapshot. The CubeIDE build does not depend on this sync, so this stub does
nothing and always succeeds, keeping the post-build step green.

Usage: python cproject_to_cmake.py <project-dir>
"""
import sys


def main() -> int:
    proj = sys.argv[1] if len(sys.argv) > 1 else "."
    print(f"[cproject_to_cmake] sync skipped (stub) for: {proj}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
