#!/usr/bin/env bash
# Hailo-8 / Hailo-8L (13 TOPS AI HAT+) on a Raspberry Pi 5 running Ubuntu 24.04 (host side).
# Raspberry Pi OS has 'apt install hailo-all'; Ubuntu has no package, so this builds Hailo's
# open-source PCIe driver for the running kernel, installs the matching firmware and udev rule,
# loads the driver at boot, and builds HailoRT + hailortcli for host-side checks.
# Driver, firmware and HailoRT must be the same version. Hailo-8/8L use the 4.x line (5.x is for
# Hailo-10/15).
#
#   sudo bash install_hailo_pi.sh            (about 15-25 min on a Pi 5)
#
# After a kernel update, run it again (the driver is built for one kernel).
set -euo pipefail
VER="${HAILO_VERSION:-4.24.0}"
SRC=/opt/hailo-src
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

step "Build tools and headers for kernel $(uname -r)"
# Finish any package install a power cut or reboot interrupted ("dpkg was interrupted").
dpkg --configure -a
apt-get update
apt-get install -y build-essential "linux-headers-$(uname -r)" git cmake curl

mkdir -p "$SRC" && cd "$SRC"
step "PCIe driver v$VER"
rm -rf hailort-drivers
git clone --depth 1 -b "v$VER" https://github.com/hailo-ai/hailort-drivers.git
cd hailort-drivers/linux/pcie
make all -j"$(nproc)"
make install
echo hailo_pci > /etc/modules-load.d/hailo.conf          # load at every boot
cd "$SRC/hailort-drivers"

step "Firmware v$VER"
./download_firmware.sh
mkdir -p /lib/firmware/hailo
mv -f "hailo8_fw.$VER.bin" /lib/firmware/hailo/hailo8_fw.bin

step "Device permissions (udev)"
cp ./linux/pcie/51-hailo-udev.rules /etc/udev/rules.d/
udevadm control --reload-rules

step "Load the driver"
modprobe -r hailo_pci 2>/dev/null || true
modprobe hailo_pci
udevadm trigger
sleep 2
ls -l /dev/hailo*

# The device number of /dev/hailo0 is assigned when the driver loads: tell docker compose.
MAJOR="$(awk '$2=="hailo"{print $1}' /proc/devices)"
ENV_FILE="$(cd -- "$(dirname -- "$0")" && pwd)/.env"
if [ -n "$MAJOR" ]; then
  touch "$ENV_FILE" && sed -i '/^HAILO_MAJOR=/d' "$ENV_FILE" && echo "HAILO_MAJOR=$MAJOR" >> "$ENV_FILE"
  chown --reference="$(dirname -- "$ENV_FILE")" "$ENV_FILE" 2>/dev/null || true
  echo "HAILO_MAJOR=$MAJOR written to $ENV_FILE (docker compose device rule)"
fi

step "HailoRT v$VER library + hailortcli (host checks)"
cd "$SRC"
rm -rf hailort
git clone --depth 1 -b "v$VER" https://github.com/hailo-ai/hailort.git
cmake -S hailort -B hailort/build -DCMAKE_BUILD_TYPE=Release -DHAILO_BUILD_EXAMPLES=OFF
cmake --build hailort/build --config release -j3
cmake --install hailort/build
ldconfig

step "Identify the chip"
hailortcli fw-control identify
echo
echo "HAILO_INSTALL_OK (version $VER). Optional, faster link: add 'dtparam=pciex1_gen=3' to"
echo "/boot/firmware/config.txt and reboot."
