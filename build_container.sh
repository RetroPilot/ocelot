#!/usr/bin/env bash
set -e

IMAGE_NAME="ocelot-builder"

DOCKER_BUILDKIT=1 docker build -t $IMAGE_NAME --label project=ocelot .
