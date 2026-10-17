#!/bin/bash
#
# Build a Team 100 camera Pi SD card image with rpi-image-gen.
#
# This does everything in ../doc/pi_setup.md.  rpi-image-gen doesn't
# install the stock Raspberry Pi OS Lite image; it builds a smaller image
# from the same Trixie packages (see config/ and layer/).  It has: user
# pi/raspberry, ssh with password, Los Angeles locale, wifi and bluetooth
# off, cloud-init reading network-config from the boot partition (the
# same as Raspberry Pi OS), camera packages, the python venv, and
# supervisord.
#
# Run it on a Debian or Ubuntu laptop (amd64 or arm64) as your normal user;
# it uses sudo when it needs to.  The first run installs dependencies and
# builds a base image (slow: 20+ minutes).  Later runs reuse the base image
# unless something in this directory changed, then stamp in the hostname and
# address for one Pi and optionally write it to an SD card.
#
# Usage: ./build_image.sh [options]
#   --ip ADDRESS     camera address, eg 10.1.0.31 or just 31 (else prompts)
#   --name NAME      hostname (default: camera<last octet>)
#   --device DEV     SD card device to write, eg /dev/sdb (else prompts)
#   --no-write       don't write an SD card, just make the .img file
#   --rebuild        rebuild the base image even if it looks up to date
#   --skip-deps      don't install/update dependencies
#   -h, --help       show this help

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RIG_REPO=https://github.com/raspberrypi/rpi-image-gen.git
# Pinned so that builds are repeatable.  Bump deliberately.
RIG_REV=6c9cea1dfccb85b6c376cdd585e84ff787414611
RIG_DIR=${RPI_IMAGE_GEN_DIR:-$SCRIPT_DIR/.rpi-image-gen}
WORK_DIR=${TEAM100_WORK_DIR:-$SCRIPT_DIR/work}
OUT_DIR=$SCRIPT_DIR/out
BASE_IMG=$OUT_DIR/team100-camera-base.img
BASE_STAMP=$BASE_IMG.inputs

NET_PREFIX=10.1.0
GATEWAY=10.1.0.2
DNS=8.8.8.8

ip=""
name=""
device=""
write=prompt
rebuild=0
skip_deps=0

say() { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mWARNING: %s\033[0m\n' "$*" >&2; }
die() { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }
usage() { sed -n '/^# Usage/,/^$/s/^# \{0,1\}//p' "$0"; }

while [[ $# -gt 0 ]]; do
   case $1 in
      --ip) ip=${2:?--ip needs a value}; shift 2 ;;
      --name) name=${2:?--name needs a value}; shift 2 ;;
      --device) device=${2:?--device needs a value}; write=yes; shift 2 ;;
      --no-write) write=no; shift ;;
      --rebuild) rebuild=1; shift ;;
      --skip-deps) skip_deps=1; shift ;;
      -h|--help) usage; exit 0 ;;
      *) usage; die "unknown option: $1" ;;
   esac
done


###############################################################################
# Preflight
###############################################################################

[[ $(uname -s) == Linux ]] || die "This needs a Linux laptop (Debian or Ubuntu)."
command -v apt-get >/dev/null || die "This needs a Debian or Ubuntu system (apt-get)."
[[ $EUID -ne 0 ]] || die "Run this as your normal user, not root. It uses sudo when needed."
case $(uname -m) in
   x86_64|aarch64) ;;
   *) die "Unsupported build machine architecture: $(uname -m)" ;;
esac


###############################################################################
# Per-Pi settings
###############################################################################

valid_octet() { [[ $1 =~ ^[0-9]{1,3}$ ]] && (( 10#$1 >= 1 && 10#$1 <= 254 )); }

# Accepts "31" or "10.1.0.31"; prints the last octet.
parse_ip() {
   local v=${1// /}
   v=${v%/24}
   [[ $v == "$NET_PREFIX".* ]] && v=${v#"$NET_PREFIX".}
   valid_octet "$v" || return 1
   echo $((10#$v))
}

while :; do
   if [[ -z $ip ]]; then
      echo
      echo "Each camera Pi gets its own static address, $NET_PREFIX.30 to $NET_PREFIX.35."
      read -r -p "Camera IP address (eg 31 or $NET_PREFIX.31): " ip
   fi
   if octet=$(parse_ip "$ip"); then
      break
   fi
   warn "'$ip' isn't an address on $NET_PREFIX.x"
   ip=""
done
[[ $octet -ne ${GATEWAY##*.} ]] || die "$GATEWAY is reserved for the laptop."
(( octet >= 30 && octet <= 36 )) || warn "$NET_PREFIX.$octet is outside the usual camera range (30-35)."
ip=$NET_PREFIX.$octet

if [[ -z $name ]]; then
   read -r -p "Hostname [camera$octet]: " name
   name=${name:-camera$octet}
fi
[[ $name =~ ^[a-zA-Z0-9]([a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?$ ]] || die "invalid hostname: $name"

echo
echo "  hostname: $name"
echo "  address:  $ip/24  (gateway $GATEWAY, dns $DNS)"
echo "  login:    pi / raspberry"


###############################################################################
# Dependencies
###############################################################################

install_deps() {
   say "Installing build dependencies (needs sudo)"
   sudo apt-get update -q
   local pkgs=(git ca-certificates debian-archive-keyring mtools fdisk util-linux)
   if [[ $(uname -m) != aarch64 ]]; then
      pkgs+=(binfmt-support)
      # Newer releases split qemu-user-static into qemu-user + qemu-user-binfmt.
      if apt-cache show qemu-user-static >/dev/null 2>&1; then
         pkgs+=(qemu-user-static)
      else
         pkgs+=(qemu-user qemu-user-binfmt)
      fi
   fi
   sudo apt-get -q -y --no-install-recommends install "${pkgs[@]}"

   say "Fetching rpi-image-gen $RIG_REV"
   if [[ ! -d $RIG_DIR/.git ]]; then
      git clone -q "$RIG_REPO" "$RIG_DIR"
   fi
   if ! git -C "$RIG_DIR" cat-file -e "$RIG_REV^{commit}" 2>/dev/null; then
      git -C "$RIG_DIR" fetch -q origin
   fi
   git -C "$RIG_DIR" -c advice.detachedHead=false checkout -q "$RIG_REV"

   say "Installing rpi-image-gen dependencies (needs sudo)"
   sudo "$RIG_DIR/install_deps.sh"

   # rpi-image-gen builds rootless, with podman user namespaces, which need
   # subordinate uid/gid ranges for this user.
   local u
   u=$(id -un)
   if ! grep -q "^$u:" /etc/subuid || ! grep -q "^$u:" /etc/subgid; then
      say "Adding subordinate uid/gid ranges for $u (needs sudo)"
      sudo usermod --add-subuids 100000-165535 --add-subgids 100000-165535 "$u"
      podman system migrate || true
   fi
}

check_host() {
   if [[ $(uname -m) != aarch64 ]] && ! ls /proc/sys/fs/binfmt_misc/qemu-aarch64* >/dev/null 2>&1; then
      sudo modprobe binfmt_misc || true
      sudo systemctl restart systemd-binfmt.service 2>/dev/null || true
      ls /proc/sys/fs/binfmt_misc/qemu-aarch64* >/dev/null 2>&1 \
         || die "arm64 emulation (qemu binfmt) isn't registered. Try rebooting, then rerun."
   fi
   if ! podman unshare true >/dev/null 2>&1; then
      warn "Rootless podman doesn't work for $(id -un)."
      if [[ $(sysctl -n kernel.apparmor_restrict_unprivileged_userns 2>/dev/null) == 1 ]]; then
         echo "Ubuntu is blocking unprivileged user namespaces. To allow them:" >&2
         echo "  sudo sysctl kernel.apparmor_restrict_unprivileged_userns=0" >&2
      fi
      die "Fix podman ('podman unshare true' must succeed), then rerun."
   fi
}

if [[ $skip_deps -eq 0 ]]; then
   install_deps
fi
[[ -x $RIG_DIR/rpi-image-gen ]] || die "rpi-image-gen not found at $RIG_DIR (don't use --skip-deps on the first run)"
check_host


###############################################################################
# Base image (shared by all Pis)
###############################################################################

# Everything that affects the base image.  If none of it changed, reuse it.
inputs_hash() {
   {
      echo "$RIG_REV"
      (cd "$SCRIPT_DIR" && find config layer -type f -print0 | sort -z | xargs -0 sha256sum)
      sha256sum "$SCRIPT_DIR/../supervisor/supervisord.conf"
   } | sha256sum | cut -d' ' -f1
}

mkdir -p "$OUT_DIR" "$WORK_DIR"
hash=$(inputs_hash)
if [[ $rebuild -eq 0 && -f $BASE_IMG && $(cat "$BASE_STAMP" 2>/dev/null) == "$hash" ]]; then
   say "Reusing base image $BASE_IMG (use --rebuild to force a new one)"
else
   say "Building base image with rpi-image-gen (this takes a while)"
   rm -f "$BASE_IMG" "$BASE_STAMP"
   (cd "$RIG_DIR" && ./rpi-image-gen build \
      -S "$SCRIPT_DIR" \
      -c team100-camera.yaml \
      -B "$WORK_DIR" \
      -- IGconf_team100_supervisor_conf="$(realpath "$SCRIPT_DIR/../supervisor/supervisord.conf")")

   built=$(ls -t "$WORK_DIR"/deploy-*/team100-camera.img 2>/dev/null | head -n 1 || true)
   [[ -n $built ]] || die "rpi-image-gen finished but no image found under $WORK_DIR"
   cp --sparse=always "$built" "$BASE_IMG"
   echo "$hash" > "$BASE_STAMP"
fi


###############################################################################
# Per-Pi image: write the cloud-init files into the boot (FAT) partition.
# These are the files the Raspberry Pi Imager writes, and network-config is
# the one from doc/pi_setup.md.
###############################################################################

img=$OUT_DIR/$name.img
say "Making $img"
cp --sparse=always "$BASE_IMG" "$img"

start=$(partx -g -o START -n 1 "$img" | tr -d ' ')
[[ $start =~ ^[0-9]+$ ]] || die "can't find the boot partition in $img"
fat="$img@@$((start * 512))"

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

cat > "$tmp/meta-data" <<EOF
dsmode: local
instance-id: team100-$name
EOF

cat > "$tmp/user-data" <<EOF
#cloud-config
hostname: $name
manage_etc_hosts: true
EOF

cat > "$tmp/network-config" <<EOF
# Applied by cloud-init at every boot.  You can edit this on the "bootfs"
# volume from any laptop to change the address.  Use spaces, not tabs.
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - $ip/24
      nameservers:
        addresses:
          - $DNS
      routes:
        - to: default
          via: $GATEWAY
      optional: true
EOF

for f in meta-data user-data network-config; do
   MTOOLS_SKIP_CHECK=1 mcopy -o -i "$fat" "$tmp/$f" "::$f"
done
MTOOLS_SKIP_CHECK=1 mtype -i "$fat" ::network-config | grep -q -- "- $ip/24" \
   || die "failed to write network-config into $img"
MTOOLS_SKIP_CHECK=1 mtype -i "$fat" ::config.txt | grep -q '^dtoverlay=disable-wifi-pi5' \
   || die "config.txt in $img doesn't disable the radios"


###############################################################################
# Write the SD card
###############################################################################

if [[ $write == prompt ]]; then
   echo
   read -r -p "Write $name.img to an SD card now? [y/N] " ans
   [[ $ans == [yY]* ]] && write=yes || write=no
fi

if [[ $write == yes ]]; then
   rootdisk=$(lsblk -no PKNAME "$(findmnt -no SOURCE /)" 2>/dev/null || true)
   if [[ -z $device ]]; then
      echo
      echo "Removable / USB disks (insert the SD card now if you haven't):"
      lsblk -dpo NAME,SIZE,TRAN,RM,MODEL | awk 'NR==1 || $4==1 || $3=="usb" || $3=="mmc"'
      echo
      read -r -p "Device to ERASE and write (eg /dev/sdb or /dev/mmcblk0): " device
   fi
   [[ -b $device ]] || die "$device isn't a block device"
   [[ $(lsblk -no TYPE "$device" | head -n 1) == disk ]] || die "$device isn't a whole disk (don't pick a partition)"
   [[ $(basename "$device") != "$rootdisk" ]] || die "$device holds this laptop's root filesystem!"

   lsblk -po NAME,SIZE,MODEL,MOUNTPOINTS "$device"
   read -r -p "Everything on $device will be erased. Type the device name again to confirm: " again
   [[ $again == "$device" ]] || die "confirmation didn't match; nothing written"

   say "Writing $img to $device (needs sudo)"
   for p in $(lsblk -lnpo NAME "$device" | tail -n +2); do
      sudo umount "$p" 2>/dev/null || true
   done
   sudo dd if="$img" of="$device" bs=4M conv=fsync status=progress
   sync
   say "Done. Remove the SD card."
fi

cat <<EOF

Image: $img

Next steps (see ../doc/pi_setup.md):
  * WRITE "$ip" ON THE PI CASE.
  * Put the SD card in the Pi and power it on. The first boot grows the
    filesystem to fill the card.
  * Set your laptop's ethernet to $GATEWAY and share its wifi connection.
  * ssh pi@$ip   (password: raspberry)
  * http://$ip:9001/ shows supervisor. "app" is FATAL until you deploy
    the code (see ../doc/deploy.md).
EOF
