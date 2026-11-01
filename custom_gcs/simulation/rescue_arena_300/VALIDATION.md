# Validation — 31 October 2026

Completed on the user's installed ROS 2 Humble / Gazebo / PX4 environment.

| Check | Result |
| --- | --- |
| Gazebo SDF schema check | Passed (`gz sdf -k`). |
| Xacro expansion and URDF tree | Passed (`xacro`, `check_urdf`). |
| Xacro / delivered URDF agreement | Visual/collision shapes, dimensions, local poses and colours match. |
| World contents | 300 × 300 m ground; exactly 50 humans and 180 trees; unique model names; no external asset URIs. |
| Human distribution | One person per zone, 50 distinct zones; closest pair 20.55 m. |
| Tree clearances | Canopies inside boundary, outside launch clearing, clear of human targets and one another. |
| GPS reference | WGS84 coordinates independently converted back to ENU, error below 0.00001 m. |
| Gazebo Harmonic 8.15.0 | World loaded and advanced 250 iterations. |
| Gazebo Garden 7.9.0 | World loaded and advanced 250 iterations; emitted `Unknown message type [8]` diagnostics but exited successfully. |
| PX4 1.16 / Harmonic | X500 spawned at launch pad; startup succeeded; reported `Ready for takeoff!`. |
| PX4 GPS | Simulated 3D fix; latitude 47.397971°, longitude 8.546164°, matching the world origin. |
| PX4 local position | Position/velocity validity flags true; estimator reference matched the world latitude/longitude. |
| Launcher shutdown | Termination completed in 1.47 s in the final check. |
| Camera rendering | Captured and visually inspected overview and human detail from Gazebo camera image topics. |
| Launcher lint | Both launch scripts passed ShellCheck. |

Rendering used the headless Ogre2 path. EGL and reserved visibility-mask warnings
were emitted, but both camera images were produced and inspected successfully.
The temporary verification cameras are not included in the delivered world.

No aircraft was armed or flown. Aerostack2 mission execution, camera detection,
geotagging accuracy in flight, and a full PX4 1.14 session were not tested.
The supplied default X500 does not include a mission camera. The human figures
are simple static geometry, so real-person detector performance is not implied.

Re-run file, layout and coordinate checks with `python3 validate_arena.py`.
