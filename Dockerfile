FROM debian:bullseye-slim

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
ENV PYTHONUNBUFFERED 1

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt \
apt update && apt-get install --no-install-recommends -y \
    libusb-dev libusb-1.0-0-dev git cmake openssh-client ca-certificates make g++ automake build-essential autoconf software-properties-common libtool-bin udev \
    catkin \
    libroscpp-dev  \
    librosbag-dev  \
    libsensor-msgs-dev \
    libcv-bridge-dev \
    libtf2-dev  libtf-dev \
    libcamera-info-manager-dev  \
    libdynamic-reconfigure-config-init-mutex-dev  \
    libstd-msgs-dev \
    ros-std-msgs \
    libyaml-dev \
    libyaml-cpp-dev git cmake \
    python3-dynamic-reconfigure

RUN update-ca-certificates
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan bitbucket.org >> ~/.ssh/known_hosts

RUN git clone -v https://github.com/libusb/libusb.git /repos/libusb && \
    cd /repos/libusb && git fetch && \
    git reset --hard master && \
    cd /repos/libusb && ./autogen.sh --disable-udev && make install && \
    rm -rf /repos/libusb

RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules

ADD https://api.github.com/repos/luxonis/depthai-core/branches/tof_rgb_mono cache-check
RUN --mount=type=ssh --mount=type=cache,target=/root/.hunter  \
    git clone --recursive --branch tof_rgb_mono https://github.com/luxonis/depthai-core.git /repos/depthai_core && \
    mkdir -p /build/depthai-core && \
    cd /build/depthai-core && cmake -DDEPTHAI_BUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_SHARED_LIBS=On /repos/depthai_core && make -j4 install && \
    rm -rf /repos /build

#ADD https://api.github.com/repos/constructiverealities/cr-dai-tools/branches/develop /.cr-dai-tools-version
#RUN git clone https://github.com/constructiverealities/cr-dai-tools.git --recursive -b develop /ros_catkin_ws/src/cr-dai-tools

ADD . /ros_catkin_ws/src/cr-dai-tools
RUN cd /ros_catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install && rm -rf /ros_catkin_ws/build

ADD entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/opt/ros/noetic/bin/autonode"]