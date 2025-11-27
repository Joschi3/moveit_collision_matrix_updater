# MoveIt Collision Matrix Updater

This repository provides a standalone command-line tool that recomputes the **self-collision matrix** for a robot’s SRDF using the same logic as the MoveIt Setup Assistant — without requiring any GUI interaction.

It is designed for automated workflows, CI pipelines, and large workspaces where the collision matrix should be regenerated programmatically.

---

## Features

* Recomputes the SRDF **disable_collisions** table using MoveIt’s `DefaultCollisions` generator
* Accepts URDF and SRDF files directly
* Supports an optional **output SRDF path** (input file is left untouched)
* Preserves **octomap collision entries** (custom `<disable_collisions link1="&lt;octomap&gt;" ...>` lines) that MoveIt does not maintain
* Works in headless environments (Docker, build servers, CI)

---

## Usage

```bash
ros2 run moveit_collision_matrix_updater moveit_collision_matrix_updater <urdf_file> <srdf_file> [num_trials] [min_fraction] [output_srdf_file]
```

### Arguments

| Argument           | Description                                              | Default       |
| ------------------ | -------------------------------------------------------- |---------------|
| `urdf_file`        | Path to the URDF file                                    | —             |
| `srdf_file`        | Path to the input SRDF file                              | —             |
| `num_trials`       | Number of random samples used for collision detection    | `50000`       |
| `min_fraction`     | Minimum fraction of collisions before disabling the pair | `0.95`        |
| `output_srdf_file` | Output SRDF path (if omitted, input SRDF is overwritten) | same as input |

### Example

```bash
ros2 run moveit_collision_matrix_updater moveit_collision_matrix_updater \
    athena.urdf athena.srdf 100000 0.9 athena_updated.srdf
```

---

## Octomap Collision Preservation

Some robots include custom SRDF rules like:

```xml
<disable_collisions link1="&lt;octomap&gt;" link2="flipper_bl_link" reason="Default"/>
```

MoveIt’s collision generator does **not** preserve these entries.

This tool automatically:

1. Extracts all octomap-related `<disable_collisions>` rules from the input SRDF
2. Reinjects them into the output SRDF after updating the collision matrix

This ensures your octomap filtering rules always remain intact.

---

## Building

```bash
colcon build --packages-select moveit_collision_matrix_updater
source install/setup.bash
```

---

## License

MIT (or whatever your repo uses—adjust as needed)

---

If you want, I can also generate:

* a shorter README.md (bullet-list style),
* an extended one with diagrams / usage examples,
* or a version targeted for GitHub Pages.
