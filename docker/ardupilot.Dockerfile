ARG BASE_TAG=latest
FROM ghcr.io/trickfirerobotics/simulations-base:${BASE_TAG}

ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS

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
