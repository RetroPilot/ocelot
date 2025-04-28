#!/usr/bin/env bash
set -e

IMAGE_NAME="ocelot-builder"

# Check if image exists
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
  echo "Docker image $IMAGE_NAME does not exist. Building..."
  DOCKER_BUILDKIT=1 docker build -t $IMAGE_NAME --label project=ocelot .
else
  echo "Docker image $IMAGE_NAME already exists. Use flash_docker.sh to build and flash."
fi
