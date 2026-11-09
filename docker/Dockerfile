ARG VSG_CHRONO_TAG=latest
ARG ARDUPILOT_TAG=latest

FROM ghcr.io/trickfirerobotics/simulations-vsg-chrono:${VSG_CHRONO_TAG} AS vsg-chrono

FROM ghcr.io/trickfirerobotics/simulations-ardupilot:${ARDUPILOT_TAG} AS sim

ARG DEBIAN_FRONTEND=noninteractive

ARG VNC_PORT
ARG NOVNC_PORT
ARG ROSBRIDGE_PORT

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
#                                QGROUNDCONTROL                                #
# ---------------------------------------------------------------------------- #

RUN echo "-------------- QGROUNDCONTROL ---------------"

ARG QGC_VERSION=v5.1.0
RUN ARCH="$(dpkg --print-architecture)" && \
    case "${ARCH}" in \
    arm64) QGC_FILE=QGroundControl-aarch64.AppImage; \
    QGC_SHA256=a782d9ed66bae3345ad32752bb18762d194cfadfe2b1d6359bfc31ef9a99989e ;; \
    amd64) QGC_FILE=QGroundControl-x86_64.AppImage; \
    QGC_SHA256=6844d46dadc7228a164662f66be9e02bc763b6d993f574083a265b26f03c307f ;; \
    *) echo "No QGroundControl build for architecture '${ARCH}'" >&2; exit 1 ;; \
    esac && \
    curl -L -s -f -o /tmp/qgc.AppImage \
    "https://github.com/mavlink/qgroundcontrol/releases/download/${QGC_VERSION}/${QGC_FILE}" && \
    echo "${QGC_SHA256} */tmp/qgc.AppImage" | sha256sum --strict --check && \
    chmod +x /tmp/qgc.AppImage && \
    cd /opt && /tmp/qgc.AppImage --appimage-extract && \
    mv squashfs-root /opt/qgroundcontrol && \
    rm -f /tmp/qgc.AppImage

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    libxcb1 libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-randr0 libxcb-render0 libxcb-render-util0 libxcb-shape0 libxcb-shm0 \
    libxcb-sync1 libxcb-util1 libxcb-xfixes0 libxcb-xinerama0 libxcb-xkb1 \
    libx11-xcb1 libdbus-1-3 libpulse0 libsdl2-2.0-0 libspeechd2 libpipewire-0.3-0t64

RUN ln -s /opt/qgroundcontrol/AppRun /usr/local/bin/qgroundcontrol

# ---------------------------------------------------------------------------- #
#                                  DEV TOOLING                                 #
# ---------------------------------------------------------------------------- #

RUN echo "--------------- DEV TOOLING ----------------"

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    clangd libomp-18-dev openssh-server python3.12-venv wget zstd \
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
COPY docker/zshrc.zsh /home/trickfire/.zshrc

EXPOSE ${NOVNC_PORT}
EXPOSE ${VNC_PORT}
EXPOSE ${ROSBRIDGE_PORT}

ENV VNC_PORT=${VNC_PORT}
ENV NOVNC_PORT=${NOVNC_PORT}
ENV ROSBRIDGE_PORT=${ROSBRIDGE_PORT}


USER trickfire
