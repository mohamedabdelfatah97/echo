# ECHO S01 — Front D455 ROS 2 Integration

Date: 19 November 2026
Status: Front RGB-D, optical TF, XYZ cloud and RViz verified; RTAB-Map pending
Platform: Isaac Sim 6.2, ROS 2 Jazzy, Ubuntu 24.04

## 1. Objective

Integrate one simulated Intel RealSense D455 assembly
into the existing Nova Carter warehouse without modifying
the verified navigation baseline.

This is the first-camera milestone of the planned four-D455
configuration (front, rear, left, right).

## 2. Scene

Verified scene:

scenes/s01_d455/s01_d455_front_ros_depth_raw_verified.usda

Stage configuration:
- metersPerUnit: 1.0
- upAxis: Z
- Original front Hawk disabled in derived scene
- Existing warehouse/navigation baseline preserved

The final scene depends on nine earlier S01 USD layers.
All ten S01 layer references were audited on 19 November 2026.

The base warehouse dependency exists within the repository.
The D455 model is referenced from NVIDIA's external Isaac Sim 6.2
asset URL and has not been vendored into this repository.

## 3. Verified ROS 2 topics

/echo/cameras/front/color/image_raw
  sensor_msgs/Image, rgb8, 640x480

/echo/cameras/front/color/camera_info
  sensor_msgs/CameraInfo, 640x480

/echo/cameras/front/depth/image_raw
  sensor_msgs/Image, 32FC1, metres, 640x480

/echo/cameras/front/depth/camera_info
  sensor_msgs/CameraInfo, 640x480

All four topics were discovered and message reception
was confirmed.

## 4. Camera calibration

Rendered RGB and depth intrinsics:

fx = fy = 317.0431199837856 pixels
cx = 320.0 pixels
cy = 240.0 pixels

Distortion: plumb_bob, five zero coefficients.

Isaac Sim forces fy = fx for square-pixel rendering.
RGB and depth optical origins are separated by 11.5 mm.

The depth CameraInfo relay uses the verified RGB
intrinsics and the corresponding depth-image header.

These are synthetic calibration parameters, not
physical D455 factory calibration.

## 5. Optical TF

Parent frame: RSD455

RGB optical:
  translation: (0, 0.0115, 0) metres

Depth optical:
  translation: (0, 0, 0) metres

Both static transforms use XYZW:
  (0.5, 0.5, -0.5, -0.5)

Both frames resolved successfully relative to base_link.
Measured RGB-depth separation: 11.50 mm.

## 6. Measured depth and synchronization

After scene reload:

Depth valid fraction: 1.0
Depth minimum: 0.4565 m
Depth median: 9.2555 m
Depth maximum: 9.5504 m

The original depth scale error was traced to missing
stage metadata (0.01 metres/unit instead of 1.0).

Ten-second RGB-D timestamp observation:
- RGB: 396 received frames
- Depth: 379 received frames
- RGB observed sim-time rate: 28.97 Hz
- Depth observed sim-time rate: 27.66 Hz
- Exact timestamp matches: 366/379 (96.6%)
- Maximum nearest timestamp difference: 33.33 ms

These are subscriber-observed results, not guarantees
of lossless camera synchronization.

## 7. Depth point cloud and RGB-depth registration

The ROS 2 Jazzy depth_image_proc point_cloud_xyz_node was tested
with the simulated 32FC1 depth stream.

Output topic:
  /echo/cameras/front/depth/points

A QoS incompatibility initially prevented point-cloud generation:
- Simulated depth publisher: RELIABLE
- Depth CameraInfo relay: BEST_EFFORT
- Converter CameraInfo subscriber: RELIABLE

The relay publisher was changed to RELIABLE.
After restarting the helpers, the XYZ point cloud was successfully
visualized in RViz, showing the warehouse floor, wall and shelving.

This verifies point-cloud generation and qualitative visualization,
not quantitative geometric accuracy.

An isolated depth_image_proc register_node experiment published:
  /echo/cameras/front/depth_registered/image_rect

The observed output header frame was:
  echo_d455_front_color_optical_frame

Registered image pixel validity and quantitative RGB-depth
reprojection accuracy remain unverified.

## 8. Unified startup

Primary launcher:
  cd ~/echo
  ./scripts/start_echo_d455_front_stack.sh

The launcher manages:
- D455 optical TF and depth CameraInfo helpers
- Depth-to-XYZ point-cloud conversion
- RViz using config/rviz/echo_d455_front.rviz

When an existing Isaac Sim instance was running, the launcher
successfully detected and reused it without opening another instance.

The operator still opens the verified USD scene manually and presses
Play in Isaac Sim.

The unified launcher was tested successfully with an already-running
Isaac Sim instance. Starting Isaac Sim from scratch through this
launcher and the complete shutdown lifecycle remain to be validated.

The original helper-only launcher remains available:
  scripts/start_echo_d455_front_helpers.sh

## 9. RTAB-Map preparation

ROS 2 Jazzy RTAB-Map packages were installed successfully:
- rtabmap_launch
- rtabmap_slam
- rtabmap_sync
- rtabmap_odom

Existing odometry topic:
  /chassis/odom

No RTAB-Map mapping, loop closure or relocalization result has
been established for the D455 scene yet.

## 10. Limitations

- Native simulated depth is idealized geometric depth, not a
  validated physical D455 stereo-depth/noise model.
- Full RGB-depth geometric reprojection validation is pending.
- Quantitative point-cloud accuracy is pending.
- Four-camera bandwidth and runtime performance are untested.
- Navigation regression testing is pending.
- RTAB-Map ingestion, mapping and relocalization are pending.
- The scene references an external NVIDIA D455 asset.

## 11. Next milestone

Preserve this verified front-camera baseline in Git.

Next, validate registered RGB-D geometry and integrate one
front D455 with RTAB-Map using isolated experimental outputs
and the existing /chassis/odom source.

Only after a single-camera RTAB-Map baseline is established
should the system expand to four cameras.

Preserve all existing navigation and Phase B baselines.
