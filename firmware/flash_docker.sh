#! /usr/bin/env bash
set -e

if [ $# -eq 0 ]; then
    echo "Usage: $0 <project>"
    exit 1
fi

docker run --rm --device /dev/bus/usb --privileged  ocelot-builder $1