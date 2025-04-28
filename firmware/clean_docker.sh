#!/usr/bin/env bash
set -e

# Remove containers with label
echo "Removing containers with label 'project=ocelot'..."
docker ps -a --filter "label=project=ocelot" -q | xargs -r docker rm -f

# Remove images with label
echo "Removing images with label 'project=ocelot'..."
docker images --filter "label=project=ocelot" --format "{{.ID}}" | xargs -r docker rmi -f

echo "Label-based cleanup complete!"