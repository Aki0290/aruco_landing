# Simulation overlay

This directory contains the ArUco markers, green probe model, Gazebo world,
gimbal / camera configuration, RViz fix, and automatic gimbal initialization
used by this project.

The overlay was tested with:

- `ardupilot_gz`: `8d368dc8298d902fa5d1675d0b0d1eaf45495555`
- `ardupilot_gazebo` (`ros2` branch): `cc0290d964dfa373531963a8fc39093a0836af0a`

Run `scripts/install_simulation_assets.sh` from the repository root to copy
the overlay into an existing `~/ardu_ws` and rebuild the affected packages.
The installer refuses to overwrite dirty upstream checkouts unless `--force`
is supplied.

