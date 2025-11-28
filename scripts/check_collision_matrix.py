#!/usr/bin/env python3
"""
Check if the MoveIt SRDF collision matrix is up to date.

Supports:
  --package-urdf   (ROS 2 package containing the URDF or Xacro)
  --urdf-path      (relative path inside that package)
  --package-srdf   (ROS 2 package containing the SRDF)
  --srdf-path      (relative path inside that package)
  --xacro-arg      (optional args to pass to xacro)

This allows the URDF/Xacro to live in one package,
and the SRDF (moveit config) to live in another.

Procedure:
 1. Resolve URDF and SRDF package paths via ament_index_python.
 2. Generate temporary URDF if input is .xacro.
 3. Run the C++ collision updater:
      ros2 run moveit_collision_matrix_updater moveit_collision_matrix_updater \
          <urdf> <srdf_in> 50000 0.95 <srdf_out>
 4. Compare <disable_collisions> entries.
 5. Exit with status 1 if differences are found.
"""

import argparse
import subprocess
import sys
import tempfile
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check if MoveIt SRDF collision matrix is up to date."
    )
    parser.add_argument(
        "--package-urdf",
        required=True,
        help="ROS 2 package name containing the URDF/Xacro.",
    )
    parser.add_argument(
        "--urdf-path",
        required=True,
        help="Path to URDF or Xacro relative to package share directory.",
    )
    parser.add_argument(
        "--package-srdf",
        required=True,
        help="ROS 2 package name containing the SRDF file.",
    )
    parser.add_argument(
        "--srdf-path",
        required=True,
        help="Path to SRDF file relative to the package share directory.",
    )
    parser.add_argument(
        "--xacro-arg",
        action="append",
        default=[],
        help="Additional xacro arguments, e.g. '--xacro-arg robot_variant:=athena'. "
        "Can be specified multiple times.",
    )
    return parser


def parse_args(argv=None):
    parser = build_arg_parser()
    return parser.parse_args(argv)


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------


def run_cmd(cmd, check=True):
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if check and result.returncode != 0:
        raise RuntimeError(
            f"Command failed: {' '.join(cmd)}\n"
            f"Return code: {result.returncode}\n"
            f"STDOUT:\n{result.stdout}\n"
            f"STDERR:\n{result.stderr}"
        )
    return result.stdout, result.stderr


def generate_urdf_from_xacro(xacro_path: Path, xacro_args, out_path: Path):
    cmd = ["xacro", str(xacro_path), *xacro_args, "-o", str(out_path)]
    run_cmd(cmd)


def run_collision_updater(urdf_path: Path, srdf_in: Path, srdf_out: Path):
    cmd = [
        "ros2",
        "run",
        "moveit_collision_matrix_updater",
        "moveit_collision_matrix_updater",
        str(urdf_path),
        str(srdf_in),
        "50000",
        "0.95",
        str(srdf_out),
    ]
    run_cmd(cmd)


def parse_disable_collisions(srdf_path: Path):
    """Return set of (link1, link2, reason) where link1/link2 are sorted."""
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    entries = set()

    for elem in root.iter():
        if elem.tag.endswith("disable_collisions"):
            link1 = elem.attrib.get("link1")
            link2 = elem.attrib.get("link2")
            reason = elem.attrib.get("reason", "")

            if link1 and link2:
                a, b = sorted([link1, link2])
                entries.add((a, b, reason))

    return entries


# ---------------------------------------------------------------------------
# Main core logic
# ---------------------------------------------------------------------------


def run(args) -> int:
    # Resolve packages via ament
    try:
        pkg_urdf_share = Path(get_package_share_directory(args.package_urdf))
        pkg_srdf_share = Path(get_package_share_directory(args.package_srdf))
    except Exception as e:
        print(
            f"[ERROR] Could not resolve package share directory: {e}", file=sys.stderr
        )
        return 1

    urdf_or_xacro = (pkg_urdf_share / args.urdf_path).resolve()
    srdf_input = (pkg_srdf_share / args.srdf_path).resolve()

    if not urdf_or_xacro.exists():
        print(f"[ERROR] URDF/Xacro not found: {urdf_or_xacro}", file=sys.stderr)
        return 1
    if not srdf_input.exists():
        print(f"[ERROR] SRDF not found: {srdf_input}", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory() as tmpdir_str:
        tmpdir = Path(tmpdir_str)

        # Convert Xacro if needed
        if urdf_or_xacro.suffix == ".xacro":
            urdf_path = tmpdir / "robot.urdf"
            print(f"[INFO] Generating URDF from Xacro: {urdf_or_xacro}")
            generate_urdf_from_xacro(urdf_or_xacro, args.xacro_arg, urdf_path)
        else:
            urdf_path = urdf_or_xacro

        # Prepare SRDF temp copies
        srdf_tmp_in = tmpdir / "input.srdf"
        srdf_tmp_out = tmpdir / "updated.srdf"
        shutil.copy2(srdf_input, srdf_tmp_in)

        # Run collision updater
        print("[INFO] Running collision matrix updater...")
        run_collision_updater(urdf_path, srdf_tmp_in, srdf_tmp_out)

        # Compare disable_collisions
        print("[INFO] Comparing SRDF collision entries...")
        original = parse_disable_collisions(srdf_tmp_in)
        updated = parse_disable_collisions(srdf_tmp_out)

        missing = original - updated
        added = updated - original

        if not missing and not added:
            print("[OK] Collision matrix is up to date.")
            return 0

        print("[ERROR] Collision matrix differences detected:", file=sys.stderr)

        if missing:
            print(
                "\nMissing entries (in SRDF, but not in regenerated):", file=sys.stderr
            )
            for a, b, reason in sorted(missing):
                print(f"  {a} -- {b} (reason={reason})", file=sys.stderr)

        if added:
            print("\nNew entries (regenerated but not in SRDF):", file=sys.stderr)
            for a, b, reason in sorted(added):
                print(f"  {a} -- {b} (reason={reason})", file=sys.stderr)

        print(
            "\nPlease regenerate the SRDF or update your manual entries.",
            file=sys.stderr,
        )
        return 1


def main(argv=None):
    args = parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
