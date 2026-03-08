#!/usr/bin/env bash
set -e

VERSION=${1:-"v1.0.0-test"}

echo "Testing release pipeline locally with version: $VERSION"

# build
echo "Building Docker container..."
./build_container.sh

echo "Building all projects..."
./flash_docker.sh ALL

# prepare artifacts
echo "Preparing release artifacts..."
mkdir -p release-artifacts
cp firmware/obj/bootstub.bin release-artifacts/
cp firmware/obj/*.bin.signed release-artifacts/

echo "Creating zip file..."
cd release-artifacts && zip -r ../ocelot-firmware-$VERSION.zip .
cd ..

echo "Release artifacts created:"
ls -la ocelot-firmware-$VERSION.zip
echo "Contents:"
unzip -l ocelot-firmware-$VERSION.zip

# cleanup
rm -rf release-artifacts

echo "Test complete! Zip file: ocelot-firmware-$VERSION.zip"