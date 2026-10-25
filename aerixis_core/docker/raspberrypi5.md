# AERIXIS on Raspberry Pi 5

> **Rescue drones:** use the onboard image in [raspberrypi5/README.md](raspberrypi5/README.md)
> (`docker/raspberrypi5/compose.yaml`): it builds only what an aircraft runs and starts every link
> (PX4 XRCE-DDS, AERIXIS, mission manager, GCS radio, mesh, A8 camera) supervised at boot. The generic
> image below builds the whole repository, simulators included.

## Which operating system?

Use a **64-bit Raspberry Pi OS** installation for the host and run AERIXIS in
the provided **64-bit Ubuntu 22.04 / ROS 2 Humble Docker image**. This is the
recommended Pi 5 setup: ROS 2 Humble's Ubuntu binaries target Jammy (22.04),
while Canonical's supported Pi 5 Ubuntu images are 24.04 and newer, not 22.04.
A native Humble install on those newer Ubuntu releases is not the supported
binary installation path.
Do not use a 32-bit Raspberry Pi OS.

Ubuntu 24.04 or newer can still be used as the host if you prefer it; Docker
keeps the Humble environment isolated from the host OS. Native Ubuntu 22.04
may work on a Pi 5 if installed from a compatible image, but it is not the
Canonical-supported Pi 5 route.

## Build the Pi image

Install Docker Engine and the Docker Compose plugin on the Pi, clone this
repository, and run from the repository root:

```bash
docker compose -f docker/compose.yaml build humble-pi5
```

This builds `aerostack2/humble:pi5` for `linux/arm64`, installs the ROS
dependencies, and builds the workspace one compile job at a time to reduce
memory pressure. The first build can take a long time and needs an internet
connection. Check that the image is ARM64:

```bash
docker image inspect aerostack2/humble:pi5 --format '{{.Architecture}}'
```

The output should be `arm64`. The Pi image uses ROS Humble's `ros-base` image
plus dependencies from this repository; it is separate from the general
`aerostack2/humble` desktop image.

## Run the simulator headless

The Pi can run the simulator without a GUI. From the repository root:

```bash
docker run --rm -it --network host \
  -v "$PWD/as2_simulation_assets/as2_gazebo_assets/test:/sim:ro" \
  aerostack2/humble:pi5 \
  ros2 launch as2_gazebo_assets launch_simulation.py \
    config_file:=/sim/config.json headless:=true
```

To also start the AERIXIS Gazebo platform bridge, open another terminal and
run:

```bash
docker run --rm -it --network host \
  -v "$PWD/as2_simulation_assets/as2_gazebo_assets/test:/sim:ro" \
  -e AEROSTACK2_SIMULATION_DRONE_ID=drone_sim_0 \
  aerostack2/humble:pi5 \
  ros2 launch as2_platform_gazebo platform_gazebo_launch.py \
    namespace:=drone_sim_0 \
    simulation_config_file:=/sim/config.json
```

Use `drone_sim_1` for the second vehicle. Running Gazebo with a GUI on the Pi
may be slow and requires working graphics access from the container. For a
better interactive simulation, run Gazebo and visualization on a desktop and
use the Pi for the vehicle-side AERIXIS nodes. ROS 2 nodes need compatible
`ROS_DOMAIN_ID` settings and network access to discover each other.

## Build an ARM64 image on another computer

With Docker Buildx configured for ARM64 emulation, from the repository root:

```bash
docker buildx build --platform linux/arm64 --load \
  -t aerostack2/humble:pi5 -f docker/raspberrypi5/Dockerfile .
```

Cross-building can be slow. The resulting image requires a 64-bit ARM host.

## Native installation

For a native Humble installation, use Ubuntu 22.04 ARM64 and follow the
repository's normal ROS 2 setup and build instructions. On Pi 5, this route
depends on obtaining a compatible Ubuntu 22.04 image; use the Docker setup
above if you want to keep the host on Raspberry Pi OS or Ubuntu 24.04.

See [the Gazebo assets guide](../as2_simulation_assets/as2_gazebo_assets/README.md)
for simulation configuration, sensors, and worlds. The Gazebo platform is for
simulation; controlling a real aircraft also requires the driver for its
flight controller and hardware.
