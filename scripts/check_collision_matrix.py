#!/usr/bin/env python3
"""
Check if the MoveIt SRDF collision matrix is up to date.

Supports:
  --package-urdf   (ROS 2 package containing the URDF or Xacro)
  --urdf-path      (relative path inside that package)
  --package-srdf   (ROS 2 package containing the SRDF)
  --srdf-path      (relative path inside that package)
  --xacro-arg      (optional args to pass to xacro)
  --fix            (apply changes to the original SRDF by editing only the
                    <disable_collisions> lines)
  -y / --yes       (skip confirmation prompt when using --fix)

This allows the URDF/Xacro to live in one package,
and the SRDF (moveit config) to live in another.

Procedure:
 1. Resolve URDF and SRDF package paths via ament_index_python.
 2. Generate temporary URDF if input is .xacro.
 3. Run the C++ collision updater:
      ros2 run moveit_collision_matrix_updater moveit_collision_matrix_updater \
          <urdf> <srdf_in> 50000 0.95 <srdf_out>
 4. Compare <disable_collisions> entries.
 5. If --fix is used, apply changes to the original SRDF by:
      - replacing the <disable_collisions> block with the regenerated entries,
        sorted alphabetically by (link1, link2, reason).
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
    parser.add_argument(
        "--fix",
        action="store_true",
        help=(
            "If differences are found, update the original SRDF by replacing the "
            "block of <disable_collisions> entries with the regenerated, sorted set."
        ),
    )
    parser.add_argument(
        "-y",
        "--yes",
        action="store_true",
        help="When used with --fix, apply changes without asking for confirmation.",
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
        "500000",
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


def confirm(prompt: str, assume_yes: bool) -> bool:
    """Ask user for confirmation unless assume_yes is True."""
    if assume_yes:
        print(f"{prompt} [y/N] (auto-yes due to --yes)")
        return True

    try:
        answer = input(f"{prompt} [y/N]: ").strip().lower()
    except EOFError:
        # Non-interactive environment, treat as "no"
        return False

    return answer in ("y", "yes")


def extract_disable_collision_block(lines):
    """
    Extract the contiguous block of <disable_collisions .../> lines.

    Returns:
      start_idx (int or None): index of first collision line
      end_idx   (int or None): index of last collision line (inclusive)
      entries   (list[str]):   the lines in that block

    Assumes all <disable_collisions> lines appear in a single block.
    """
    start = None
    end = None
    entries = []

    for idx, line in enumerate(lines):
        if "<disable_collisions" in line:
            if start is None:
                start = idx
            end = idx
            entries.append(line)

    return start, end, entries


def format_collision_line(a: str, b: str, reason: str) -> str:
    """Generate a canonical disable_collisions line with required escaping."""

    def escape(link):
        # MoveIt SRDF uses escaped octomap identifier:
        if link == "<octomap>":
            return "&lt;octomap&gt;"
        return link

    return (
        f'    <disable_collisions link1="{escape(a)}" '
        f'link2="{escape(b)}" reason="{reason}"/>'
    )


def apply_collision_matrix_changes(
    srdf_original: Path,
    srdf_generated: Path,
    missing_entries,
    added_entries,
):
    """
    Apply collision matrix changes to the original SRDF file in a minimal way.

    We take the full set of <disable_collisions> entries from the generated SRDF,
    sort them alphabetically by (link1, link2, reason), and replace only the
    block of <disable_collisions> lines in the original file with this sorted set.

    This preserves all other content (comments, unrelated tags, etc.).
    """
    # Read the original SRDF
    orig_text = srdf_original.read_text(encoding="utf-8")
    orig_lines = orig_text.splitlines()

    # Find the block of <disable_collisions> lines in the original
    block_start, block_end, _ = extract_disable_collision_block(orig_lines)
    if block_start is None or block_end is None:
        print(
            "[ERROR] No <disable_collisions> block found in original SRDF. "
            "Cannot apply changes safely.",
            file=sys.stderr,
        )
        return

    # Parse the final (regenerated) set of collision entries
    updated_entries = parse_disable_collisions(srdf_generated)

    # Build sorted collision lines from updated entries
    sorted_lines = [
        format_collision_line(a, b, reason)
        for (a, b, reason) in sorted(updated_entries)
    ]

    # Construct the new file content:
    # - everything before the original block
    # - the new sorted block
    # - everything after the original block
    new_lines = []
    new_lines.extend(orig_lines[:block_start])
    new_lines.extend(sorted_lines)
    new_lines.extend(orig_lines[block_end + 1 :])

    srdf_original.write_text("\n".join(new_lines) + "\n", encoding="utf-8")
    print(f"[OK] Collision matrix updated and sorted in {srdf_original}")


# ---------------------------------------------------------------------------
# Main core logic
# ---------------------------------------------------------------------------


def run(args, prog_name: str, original_argv) -> int:
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
                "\nMissing entries (in SRDF, but not in regenerated):",
                file=sys.stderr,
            )
            for a, b, reason in sorted(missing):
                print(f"  {a} -- {b} (reason={reason})", file=sys.stderr)

        if added:
            print(
                "\nNew entries (regenerated but not in SRDF):",
                file=sys.stderr,
            )
            for a, b, reason in sorted(added):
                print(f"  {a} -- {b} (reason={reason})", file=sys.stderr)

        if not args.fix:
            # Build a command the user can copy-paste to auto-fix the SRDF
            # Use the same entry point name and args, plus --fix --yes.
            fix_cmd = " ".join([prog_name, *list(original_argv), "--fix", "--yes"])
            print(
                "\nRun the following command to update the <disable_collisions> "
                "block in the SRDF to the regenerated, sorted version:\n"
                f"  {fix_cmd}",
                file=sys.stderr,
            )
            return 1

        # Fix mode: ask for confirmation (unless --yes)
        prompt = (
            f"\nApply regenerated collision matrix to original SRDF by "
            f"replacing and sorting the <disable_collisions> block?\n  {srdf_input}"
        )
        if not confirm(prompt, assume_yes=args.yes):
            print("\nAborting; SRDF was NOT modified.", file=sys.stderr)
            return 1

        try:
            apply_collision_matrix_changes(
                srdf_original=srdf_input,
                srdf_generated=srdf_tmp_out,
                missing_entries=missing,
                added_entries=added,
            )
        except Exception as e:
            print(
                f"[ERROR] Failed to apply changes to SRDF {srdf_input}: {e}",
                file=sys.stderr,
            )
            return 1

        return 0


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    args = parse_args(argv)
    prog_name = Path(sys.argv[0]).name or "moveit_collision_matrix_updater"
    sys.exit(run(args, prog_name, argv))


if __name__ == "__main__":
    main()
