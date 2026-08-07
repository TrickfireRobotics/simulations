ARG VSG_CHRONO_TAG=latest
FROM ghcr.io/trickfirerobotics/simulations-vsg-chrono:${VSG_CHRONO_TAG} AS vsg-chrono

ARG QGROUNDCONTROL_TAG=latest
FROM ghcr.io/trickfirerobotics/simulations-qgroundcontrol:${QGROUNDCONTROL_TAG} AS qgroundcontrol

FROM ubuntu:24.04 AS sim

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ="America/Los_Angeles"
ARG BUILD_JOBS

ARG VNC_PORT
ARG NOVNC_PORT
ARG ROSBRIDGE_PORT

# ---------------------------------------------------------------------------- #
#                                     BASE                                     #
# ---------------------------------------------------------------------------- #

RUN echo "------------------- BASE -------------------"

# Locale + timezone
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install --no-install-recommends -y locales curl gcc git libc6-dev sudo ca-certificates && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install --no-install-recommends -y libvulkan1 mesa-vulkan-drivers

ENV LANG=en_US.UTF-8

RUN ln -snf "/usr/share/zoneinfo/${TZ}" /etc/localtime && \
    echo "${TZ}" > /etc/timezone

# User
RUN userdel ubuntu 2>/dev/null || true && \
    useradd trickfire --uid 1000 --shell /bin/bash --create-home --no-log-init && \
    usermod -aG video trickfire
RUN echo "trickfire ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/user

# The Jetson/Tegra libdrm.so.2 is missing drmCloseBufferHandle, which
# libdrm_nouveau.so.2 needs. Without it, swrast_dri.so fails to load and
# Mesa GLX has no FB configs. This stub provides the missing symbol so Mesa
# software rendering works inside the container.
# Sketchy solution idk what to do with this...
# TODO: Remove this when graphics are fixed
RUN printf 'int drmCloseBufferHandle(int fd, unsigned int handle){ return 0; }\n' \
    > /tmp/drm_stub.c && \
    gcc -shared -fPIC -o /usr/local/lib/libdrm_stub.so /tmp/drm_stub.c && \
    rm /tmp/drm_stub.c && \
    ldconfig

# ---------------------------------------------------------------------------- #
#                                 VSG + CHRONO                                 #
# ---------------------------------------------------------------------------- #

RUN echo "----------------- VSG + CHRONO --------------"

# Prebuilt in the vsg-chrono image (see top of file) instead of compiled here.
ENV VSG_INSTALL_DIR=/opt/vsg
ENV LD_LIBRARY_PATH=/opt/vsg/lib

COPY --from=vsg-chrono /opt/vsg /opt/vsg
COPY --from=vsg-chrono /home/trickfire/chrono /home/trickfire/chrono

RUN chown -R trickfire:trickfire /home/trickfire/chrono

# ---------------------------------------------------------------------------- #
#                                    GAZEBO                                    #
# ---------------------------------------------------------------------------- #

RUN echo "------------------ GAZEBO ------------------"

# ROS 2 Jazzy apt source
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr gnupg2

RUN curl -L -s -f -o /tmp/ros2-apt-source.deb \
    https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.noble_all.deb && \
    echo "0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5 */tmp/ros2-apt-source.deb" | sha256sum --strict --check && \
    apt-get update && apt-get install /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb

# Gazebo Harmonic apt source
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-jazzy-ros-base \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-controllers \
    python3-tk

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install -y --no-install-recommends \
    libgz-cmake4-dev \
    libgz-gui8 \
    libgz-plugin3-dev \
    libgz-common6-dev \
    libgz-sim8-dev \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-rosbridge-suite

RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --break-system-packages pypresence trimesh pyfqmr

RUN rosdep init || true

# ---------------------------------------------------------------------------- #
#                                 VNC / DISPLAY                                #
# ---------------------------------------------------------------------------- #


RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    xvfb novnc x11vnc websockify openbox xterm python3-xdg x11-utils \
    mesa-utils libglx-mesa0 libgl1-mesa-dri x11-apps \
    xserver-xorg-core xserver-xorg-video-dummy kmod

# ---------------------------------------------------------------------------- #
#                                 VirtualGL                                    #
# ---------------------------------------------------------------------------- #

ARG VIRTUALGL_VERSION=3.1.4
RUN ARCH="$(dpkg --print-architecture)" && \
    case "${ARCH}" in \
    arm64) VGL_SHA256="5583e08ee5694d14c6ea0f25d2cfd73fa6d5077869c6645c334a948265a777bb" ;; \
    amd64) VGL_SHA256="02edc6b599571c385389af1a006f07a70c298e1d97c580a9bfd4b39d835c51e6" ;; \
    *) echo "No VirtualGL build for architecture '${ARCH}'" >&2; exit 1 ;; \
    esac && \
    curl -L -s -f -o /tmp/virtualgl.deb \
    "https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_${ARCH}.deb" && \
    echo "${VGL_SHA256} */tmp/virtualgl.deb" | sha256sum --strict --check && \
    dpkg -i /tmp/virtualgl.deb && \
    rm -f /tmp/virtualgl.deb

# ---------------------------------------------------------------------------- #
#                                  ARDUPILOT                                   #
# ---------------------------------------------------------------------------- #

RUN echo "---------------- ARDUPILOT -----------------"

ARG ARDUPILOT_REF=Copter-4.3.7
ARG ARDUPILOT_WAF_COMMIT=35eadbb64e2052099a853b571e507c33032b392c

ENV ARDUPILOT_DIR=/opt/ardupilot
ENV ARDUPILOT_GAZEBO_DIR=/opt/ardupilot_gazebo

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential ccache gawk python3-dev python-is-python3 libtool-bin rsync \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --break-system-packages \
    future lxml pymavlink pyserial ptyprocess pexpect catkin_pkg \
    "setuptools<81" "empy==3.3.4"

RUN usermod -a -G dialout trickfire

RUN git clone --recurse-submodules --branch "${ARDUPILOT_REF}" \
    https://github.com/ArduPilot/ardupilot.git "${ARDUPILOT_DIR}"

RUN cd "${ARDUPILOT_DIR}/modules/waf" && \
    git fetch --depth 1 origin "${ARDUPILOT_WAF_COMMIT}" && \
    git checkout "${ARDUPILOT_WAF_COMMIT}"

RUN sed -i '/#include <linux\/can\/raw.h>/a #include <cstdint>' \
    "${ARDUPILOT_DIR}/libraries/AP_HAL_SITL/CANSocketIface.cpp"

RUN cd "${ARDUPILOT_DIR}" && \
    ./waf configure --board sitl && \
    ./waf copter

RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git "${ARDUPILOT_GAZEBO_DIR}"

RUN cd "${ARDUPILOT_GAZEBO_DIR}" && \
    GZ_VERSION=harmonic cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    cmake --build build --parallel "${BUILD_JOBS:-$(nproc)}"

RUN chown -R trickfire:trickfire "${ARDUPILOT_DIR}" "${ARDUPILOT_GAZEBO_DIR}"

# ---------------------------------------------------------------------------- #
#                                QGROUNDCONTROL                                #
# ---------------------------------------------------------------------------- #

RUN echo "-------------- QGROUNDCONTROL ---------------"

COPY --from=qgroundcontrol /opt/qgroundcontrol /opt/qgroundcontrol

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    libxcb1 libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-randr0 libxcb-render0 libxcb-render-util0 libxcb-shape0 libxcb-shm0 \
    libxcb-sync1 libxcb-util1 libxcb-xfixes0 libxcb-xinerama0 libxcb-xkb1 \
    libx11-xcb1 libdbus-1-3 libpulse0 libsdl2-2.0-0 libspeechd2

RUN ln -s /opt/qgroundcontrol/AppRun /usr/local/bin/qgroundcontrol

# ---------------------------------------------------------------------------- #
#                                  DEV TOOLING                                 #
# ---------------------------------------------------------------------------- #

RUN echo "--------------- DEV TOOLING ----------------"

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    clangd openssh-server python3.12-venv wget zstd \
    build-essential cmake ninja-build \
    libeigen3-dev libvulkan-dev libxcb1-dev pkg-config

RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install ruff pre-commit --break-system-packages --ignore-installed

RUN curl -sSL "https://github.com/mvdan/sh/releases/latest/download/shfmt_$(curl -sSL https://api.github.com/repos/mvdan/sh/releases/latest | grep -oP '(?<="tag_name": ")v[^"]+')_linux_$(dpkg --print-architecture)" \
    -o /usr/local/bin/shfmt && \
    chmod +x /usr/local/bin/shfmt

RUN rosdep fix-permissions && \
    chown -R trickfire:trickfire /usr/local/lib/python3.12/dist-packages /usr/local/bin && \
    chmod -R a+rX /usr/local/lib/python3.12/dist-packages

COPY docker/xorg.nvidia.conf /etc/X11/xorg.nvidia.conf
COPY docker/xorg.dummy.conf /etc/X11/xorg.dummy.conf
COPY docker/xorg.vkms.conf /etc/X11/xorg.vkms.conf
COPY docker/bashrc.sh /home/trickfire/.bashrc

EXPOSE ${NOVNC_PORT}
EXPOSE ${VNC_PORT}
EXPOSE ${ROSBRIDGE_PORT}

ENV VNC_PORT=${VNC_PORT}
ENV NOVNC_PORT=${NOVNC_PORT}
ENV ROSBRIDGE_PORT=${ROSBRIDGE_PORT}


USER trickfire
