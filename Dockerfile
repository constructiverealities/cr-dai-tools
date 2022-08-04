FROM debian:bullseye-slim as build

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
ENV PYTHONUNBUFFERED 1

RUN apt update && apt-get install --no-install-recommends -y \
    libusb-dev libusb-1.0-0-dev git cmake openssh-client ca-certificates make g++ automake build-essential autoconf software-properties-common libtool-bin udev \
    gdb catkin \
    libroscpp-dev \
    libopencv-highgui-dev \
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

ARG DEPTHAI_REPO=constructiverealities/depthai
ARG DEPTHAI_TAG=cr/develop

ADD https://api.github.com/repos/$DEPTHAI_REPO/branches/$DEPTHAI_TAG cache-check

RUN --mount=type=cache,id=dai-build-$DEPTHAI_TAG,target=/build --mount=type=cache,target=/root/.hunter \
    git clone -b $DEPTHAI_TAG https://github.com/$DEPTHAI_REPO.git --recursive /repos/depthai_core && \
    mkdir -p /build/depthai-core/$DEPTHAI_TAG && \
    cd /build/depthai-core/$DEPTHAI_TAG && cmake -DDEPTHAI_OPENCV_SUPPORT=ON -DDEPTHAI_BUILD_EXAMPLES=OFF -DDEPTHAI_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_SHARED_LIBS=On /repos/depthai_core && make -j4 install

COPY . /ros_catkin_ws/src/cr-dai-tools

RUN cd /ros_catkin_ws &&  \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_BUILD=ON -DBUILD_DEPTHAI=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install && \
    find /usr/include && \
    cat /ros_catkin_ws/build/CMakeFiles/CMakeError.log

RUN mkdir -p needed_libs && \
    ldd /opt/ros/noetic/bin/* | awk 'NF == 4 {print $3}; NF == 2 {print $1}' | uniq | grep  usr/lib | xargs -I {} cp -v -L {} /needed_libs

CMD ["/opt/ros/noetic/bin/autonode"]

FROM debian:bullseye-slim
ENV PYTHONUNBUFFERED 1
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt update && apt-get install --no-install-recommends -y \
    libexpat1 udev

COPY --from=build /opt/ros/noetic /opt/ros/noetic
COPY --from=build /etc/udev/rules.d/ /etc/udev/rules.d/
COPY --from=build /needed_libs /usr/lib/x86_64-linux-gnu/
#COPY --from=build /libusb_root /

RUN groupadd --gid 1000 cr-user && useradd --uid 1000 --gid cr-user --shell /bin/bash -d /root -G sudo cr-user && echo "cr-user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
RUN chown cr-user -R /root
USER 1000:1000

ENV LD_PRELOAD=/lib/x86_64-linux-gnu/libSegFault.so
ENV ROS_LOAD_DISTRO=noetic
#ENTRYPOINT ["/entrypoint.sh"]
CMD ["/opt/ros/noetic/bin/autonode"]