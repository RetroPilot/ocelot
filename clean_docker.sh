#!/usr/bin/env bash
set -e

# Remove containers with label ocelot
docker ps -a --filter "label=project=ocelot" -q | xargs -r docker rm -f
docker images --filter "label=project=ocelot" --format "{{.ID}}" | xargs -r docker rmi -f
