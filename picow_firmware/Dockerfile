# Use Ubuntu as the base image
FROM ubuntu:latest

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    make \
    libtool \
    pkg-config \
    autoconf \
    automake \
    texinfo \
    libusb-1.0-0-dev \
    libhidapi-dev

# Clone the OpenOCD repository
RUN git clone https://github.com/openocd-org/openocd.git

# Change the working directory to the OpenOCD repository
WORKDIR /openocd

# Build OpenOCD
RUN ./bootstrap

# Configure OpenOCD with CMSIS-DAP via USB support
RUN ./configure --enable-cmsis-dap
RUN make
RUN make install

# Set the entrypoint to OpenOCD
ENTRYPOINT ["openocd"]
