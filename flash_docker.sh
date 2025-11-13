#! /usr/bin/env bash
set -e

# if [ $# -eq 0 ]; then
#     echo "Usage: $0 <project>"
#     exit 1
# fi

docker run --rm \
  -v $(pwd):/ocelot \
  -w /ocelot \
  ocelot-builder ./build_project.sh $1

if [ "$1" != "ALL" ] && [ "$1" != "--clean" ]; then
  ./recover.sh $1
fi

# docker run -it --rm ocelot-builder $1
# docker run --rm --device /dev/bus/usb --privileged  ocelot-builder $1