# Vendored Orbbec ROS 2 driver

This directory contains the Orbbec ROS 2 v1 driver required by the legacy
Gemini Pro (`SV1301S_U3`, PID `0x0614`). It is intentionally kept inside the
main ROS workspace so a single `colcon build` and `source install/setup.bash`
are sufficient.

## Provenance

- Upstream project: `OrbbecSDK_ROS2`, `main` (v1.x) distribution
- ROS wrapper packages: `1.5.15`
- Bundled Orbbec SDK: `1.10.35`
- Imported archive: `OrbbecSDK_ROS2-main.zip`
- Archive SHA-256:
  `fa3f290ff849c59032d0c4f912f04efc31f1752481baa09060fba9cf380ad36e`
- Imported on: `2026-09-23`

The upstream `LICENSE` and `NOTICE` files are retained in this directory.

## Included packages

- `orbbec_camera_msgs`
- `orbbec_camera`

`orbbec_description` is not included because this robot already owns the
Gemini Pro link and optical-frame geometry. Omitting that package also avoids
shipping unrelated camera meshes. The SDK libraries for x64 and arm64 are kept
unchanged so the same source tree remains buildable on the development PC and
robot computer. The unused arm32 SDK bundle was removed on `2026-09-23`; this
vendored copy does not support 32-bit ARM hosts.

Do not add generated `build`, `install`, `log`, `Log`, `__pycache__`, or nested
`.git` directories under this tree.
