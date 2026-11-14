#!/bin/bash
# Build a Team 100 camera Pi SD card image in Docker on a Mac.
#
# Run from the root of the all26 repo, on a branch with raspberry_pi/image
# (e.g. 2027):
#
#  ./raspberry_pi/image/build_image_docker_mac.sh 31            # makes raspberry_pi/image/out/camera31.img
#  ./raspberry_pi/image/build_image_docker_mac.sh 32 camera32   # optional hostname (default camera<N>)
#
# The first build takes 20+ minutes. The Docker container (t100img) is kept,
# so later builds reuse its base image and take a minute or two. To start
# over: docker rm -f t100img
#
# Then write the .img to an SD card with Raspberry Pi Imager ("Use custom")
# or balenaEtcher.

set -euo pipefail

CONTAINER=t100img
IP=${1:?Usage: $0 <camera number or IP, e.g. 31> [hostname]}
NAME=${2:-camera${IP##*.}}
SRC=/home/builder/all26/raspberry_pi/image

if ! docker info >/dev/null 2>&1; then
  echo "Docker isn't running. Start Docker Desktop and try again." >&2
  exit 1
fi
if ! git cat-file -e HEAD:raspberry_pi/image/build_image.sh 2>/dev/null; then
  echo "Run this from the all26 repo root, on a branch with raspberry_pi/image (e.g. git checkout 2027)." >&2
  exit 1
fi
if [ -n "$(git status --porcelain -- raspberry_pi/image raspberry_pi/supervisor)" ]; then
  echo "Uncommitted changes in raspberry_pi/image or raspberry_pi/supervisor:" >&2
  git status --short -- raspberry_pi/image raspberry_pi/supervisor >&2
  echo "This builds from the last commit ($(git rev-parse --short HEAD)), so commit them first." >&2
  exit 1
fi

# 1. The build container: made once, then reused.
if ! docker container inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "==> Making the build container"
  docker run -d --privileged --name "$CONTAINER" debian:trixie sleep infinity >/dev/null
  docker exec "$CONTAINER" bash -c '
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -q >/dev/null
    apt-get install -y -q sudo procps >/dev/null
    useradd -m -s /bin/bash builder
    echo "builder ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/builder'
  first=yes
else
  echo "==> Reusing the build container"
  docker start "$CONTAINER" >/dev/null
  first=no
fi

# 2. The image builder's files, as committed (work/ in the container, with the
#    cached base image, is kept).
echo "==> Copying in raspberry_pi/image and raspberry_pi/supervisor from $(git rev-parse --short HEAD)"
git archive HEAD raspberry_pi/image raspberry_pi/supervisor \
  | docker exec -i "$CONTAINER" bash -c \
      'mkdir -p /home/builder/all26 && tar -C /home/builder/all26 -xf - && chown -R builder: /home/builder/all26'

# 3. Build.
echo "==> Building $NAME ($IP)"
deps=$([ "$first" = yes ] && echo "" || echo "--skip-deps")
docker exec -u builder -w "$SRC" "$CONTAINER" ./build_image.sh --ip "$IP" --name "$NAME" --no-write $deps

# 4. Copy the image out to the Mac, and check it arrived intact.
mkdir -p raspberry_pi/image/out
OUT=raspberry_pi/image/out/$NAME.img
echo "==> Copying it to $OUT"
docker cp "$CONTAINER:$SRC/out/$NAME.img" "$OUT"
inside=$(docker exec "$CONTAINER" sha256sum "$SRC/out/$NAME.img" | cut -d' ' -f1)
outside=$(shasum -a 256 "$OUT" | cut -d' ' -f1)
if [ "$inside" != "$outside" ]; then
  echo "The copy doesn't match the built image. Run again." >&2
  exit 1
fi
docker stop "$CONTAINER" >/dev/null
echo "Done: $OUT ($(du -h "$OUT" | cut -f1)). Write it to an SD card with Raspberry Pi Imager."
