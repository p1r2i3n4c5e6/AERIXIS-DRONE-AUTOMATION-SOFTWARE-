#!/usr/bin/env bash
# One-time Raspberry Pi 5 host setup for the AS2 rescue onboard container.
# Run ON THE PI from the repository root:   bash docker/raspberrypi5/setup_pi5.sh
# Safe to run again. It asks for sudo for the host changes and prints every step.
set -euo pipefail
cd "$(dirname "$0")"
step() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

[ "$(uname -m)" = "aarch64" ] || { echo "This is not a 64-bit ARM Pi (uname -m = $(uname -m)). Use 64-bit Raspberry Pi OS."; exit 1; }

step "1/6 Docker Engine"
if ! command -v docker >/dev/null; then
  echo "Docker is not installed. Install it with the official script, then run this again:"
  echo "  curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh"
  exit 1
fi
docker compose version >/dev/null || { echo "Docker Compose plugin missing (sudo apt install docker-compose-plugin)."; exit 1; }
id -nG | grep -qw docker || { sudo usermod -aG docker "$USER"; echo "Added $USER to the docker group: log out and back in, then run this again."; exit 0; }

step "2/6 Flight-controller UART (PX4 TELEM2 -> Pi GPIO14/15 = /dev/ttyAMA0)"
CONFIG=/boot/firmware/config.txt; [ -f "$CONFIG" ] || CONFIG=/boot/config.txt
if ! grep -q '^dtparam=uart0=on' "$CONFIG"; then
  echo "Enabling uart0 in $CONFIG (needs a reboot)"; echo 'dtparam=uart0=on' | sudo tee -a "$CONFIG" >/dev/null; REBOOT=1
fi
if [ -f /boot/firmware/cmdline.txt ] && grep -q 'console=serial0' /boot/firmware/cmdline.txt; then
  echo "Removing the Linux serial console from the UART (it would talk over PX4)"
  sudo sed -i 's/console=serial0,[0-9]* //' /boot/firmware/cmdline.txt; REBOOT=1
fi
sudo systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true

step "3/6 Serial permissions"
id -nG | grep -qw dialout || sudo usermod -aG dialout "$USER"

step "4/6 Radios: use stable names"
echo "Plugged-in USB serial devices (put these /dev/serial/by-id/... paths in onboard.env so ports never swap):"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "  (none plugged in right now)"

step "5/6 Settings"
if [ ! -f onboard.env ]; then
  cp onboard.env.example onboard.env
  echo "Created docker/raspberrypi5/onboard.env — edit AS2_DRONE_ID, AS2_SYSTEM_ID, radio devices and the mesh passphrase now."
  ${EDITOR:-nano} onboard.env
fi

step "6/6 Build and start (first build 40-60 min on a Pi 5; keep it cool)"
docker compose build
docker compose up -d
sleep 5
docker compose ps
echo
docker compose exec onboard /entrypoint.sh token || true
echo
echo "Done. The onboard service starts by itself after every reboot."
echo "  live output:   docker compose -f $(pwd)/compose.yaml logs -f onboard"
echo "  link states:   docker compose -f $(pwd)/compose.yaml exec onboard /entrypoint.sh status"
[ "${REBOOT:-0}" = 1 ] && echo "REBOOT NEEDED for the UART change: sudo reboot"
exit 0
