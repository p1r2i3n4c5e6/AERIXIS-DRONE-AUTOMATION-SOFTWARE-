# AERIXIS launchers

One launcher per system. Each one runs its system in the background. Logs and PID files go to
`~/.local/share/aerixis` (set `AERIXIS_STATE` to change this). Every launcher accepts
`start`, `stop`, `status` and `logs`.

| System | Where it runs | Start | Opens |
|---|---|---|---|
| Ground station | laptop | `launch/gcs.sh start` | GCS window, http://localhost:8765 |
| Simulation (PX4 SITL + Gazebo) | laptop | `launch/sim.sh start [N]` | Gazebo |
| Onboard stack | Raspberry Pi 5 (or laptop with `--sim`) | `launch/onboard.sh start` | onboard control panel, http://\<pi\>:8090 (last step) |

## Ground station (laptop)

```bash
launch/gcs.sh start                 # radio/UDP link is chosen on the Connect page
launch/gcs.sh start --udp-port 14555
launch/gcs.sh status                # link, heartbeat rate, every aircraft's connection
launch/gcs.sh stop
```

The GCS folder is `~/custom_gcs` (set `AERIXIS_GCS_DIR` to change this). If a GCS is already
running and this launcher did not start it, the launcher says so and leaves it alone.

## Simulation (laptop)

```bash
launch/sim.sh start 2                         # 2 aircraft, 20 m apart, plus their onboard stacks
launch/sim.sh start 1 --home 19.0760,72.8777  # fly the world over your area (lat,lon[,alt])
launch/sim.sh start 1 --px4-only              # PX4 only; then: launch/onboard.sh start --sim
launch/sim.sh logs px4 | logs onboard
launch/sim.sh stop
```

Then in the GCS: **Connect → Simulation (UDP 14555) → Add aircraft manually**, once per drone, with the
values `launch/sim.sh pairing` prints (drone ID, system ID, pairing token). The GCS never reads the
onboard files itself, so the steps are the same as with a Pi over the telemetry radio.

## Onboard stack (Raspberry Pi 5)

```bash
launch/onboard.sh start             # native install
launch/onboard.sh start --docker    # the container from docker/raspberrypi5
launch/onboard.sh install           # start at every boot (systemd) + open the panel at desktop login
launch/onboard.sh status | logs | token | panel | stop
```

Start order: FC link (XRCE agent), FC guardian, AERIXIS core with the mission manager, GCS
telemetry gateway, mesh, camera, and last the **onboard control panel**. The launcher waits
until every link reports running, then:

- **Pi with a desktop**: opens the panel in a browser window.
- **Ubuntu Server (no screen)**: prints the panel address. Open it from any laptop or phone on
  the drone's network: `http://<pi-address>:8090`.

It also prints the drone ID, system ID, GCS pairing token and **panel PIN**. Get them again with
`launch/onboard.sh token`.

### The onboard control panel (port 8090)

| Section | What you set |
|---|---|
| Onboard links | state of every link; restart one link or all |
| 1 GCS telemetry radio | serial radio (port, baud) or UDP (laptop IP, port) |
| 2 Flight controller | PX4 link: serial UART (port, baud) or UDP (SITL / Ethernet); USB health tap. Shows live PX4 data, GNSS, preflight and faults |
| 3 A8 camera | off / simulated / preview / track, camera IP, CPU cores. **Open camera dashboard** opens the A8 tracker's own web page (port 8080 on the Pi) |
| 4 Drone-to-drone mesh | LoRa/serial or IP mesh, fleet names, passphrase |
| 5 Identity and pairing | drone ID, system ID, show the pairing token |
| 6 Safety switches | allow flight, paths clear, emergency action, payload AUX, separation |

Anyone on the network can view the panel. Every change needs the PIN. The panel will not restart
a flight link (FC, guardian, core, manager, GCS, mesh) while the aircraft is armed or flying a
mission; only the camera can be restarted then. Settings fixed by an `AS2_*` environment
variable (for example in Docker `onboard.env`) are shown read-only.

### Laptop test of the onboard stack

```bash
launch/sim.sh start 1 --px4-only
launch/onboard.sh start --sim       # drone0 against SITL; panel on http://localhost:8090
launch/gcs.sh start                 # Connect -> Simulation -> Add aircraft manually (launch/onboard.sh token)
```

## Run a simulation yourself (step by step)

1. `launch/sim.sh start 2` — PX4 + Gazebo + both onboard stacks. Wait for "Simulation ready"; it prints
   each drone's ID, system ID and token (again any time: `launch/sim.sh pairing`).
   Optional: `--persons "x,y;x,y"` puts simulated people (plan metres) inside your KML area.
2. `launch/gcs.sh start` → **Connect**: Simulation, onboard port 14555, **Open link**; then **Add aircraft
   manually** once per drone with the values from step 1 (same as with a Pi).
3. **Swarm start**: every drone "ready to fly".
4. **Plan**: mission, accept the conditions, load the KML (use an area big enough for every drone, e.g.
   100 x 80 m for two), **Generate & assign trees**.
5. **Upload & start**: Upload all. In the menu switch **Safety** to READY, then Start all.
6. **Operations**: map and live tree. Menu buttons work on every page: **RTL**, **Land**, **Disarm** (only on
   the ground), **Safety** back to SAFE after the flight.
7. A normal flight unlocks itself after landing. After a cancel, abort or failure: press **Reconcile** on the
   drone's card in Upload & start.
8. `launch/sim.sh stop` when done. If the laptop changed Wi-Fi meanwhile, stop and start again.

## Troubleshooting

**Started a mission but nothing flies, after the laptop changed Wi-Fi.** ROS 2 nodes started
before a network change (Wi-Fi dropped, joined another network or hotspot, new IP) keep working
with each other, so the GCS still shows every link green. A process started after the change,
such as the behaviour-tree runner that Start launches, cannot find them. The runner then stops
after 5 s with `Service set_arming_state not available` (shown as ERROR on the drone card).
Restart the simulation so every node starts on the current network:

```bash
launch/sim.sh stop
launch/sim.sh start 2
```

The same applies on a Pi whose network changes while the onboard stack runs: restart the stack.
