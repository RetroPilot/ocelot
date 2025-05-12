# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    scons \
    python3 \
    python3-pip \
    git \
    dfu-util \
    gcc-arm-none-eabi \
    binutils-arm-none-eabi \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install libusb1 pycryptodome requests

COPY . /ocelot

WORKDIR /ocelot

# ENTRYPOINT ["./build_project.sh"]
