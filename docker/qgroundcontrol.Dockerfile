# Prebuilt QGroundControl base image, built from source.

FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS
ARG QGC_REF=v5.1.0
ARG TARGETARCH

ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git python3 curl locales && \
    sed -i 's/^# *\(en_US.UTF-8 UTF-8\)/\1/' /etc/locale.gen && \
    locale-gen && \
    update-locale LANG=en_US.UTF-8

RUN git clone -c advice.detachedHead=false --depth 1 --recurse-submodules \
    --branch "${QGC_REF}" \
    https://github.com/mavlink/qgroundcontrol.git /opt/qgc-src

RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh && \
    uv venv --seed /opt/qgc-venv --python python3 && \
    VIRTUAL_ENV=/opt/qgc-venv uv sync \
    --project /opt/qgc-src/tools --extra scripts --extra qt \
    --no-install-project --frozen --active

RUN set -eux; \
    case "${TARGETARCH}" in \
    amd64) qt_host=linux; qt_arch=linux_gcc_64; qt_dir=gcc_64 ;; \
    arm64) qt_host=linux_arm64; qt_arch=linux_gcc_arm64; qt_dir=gcc_arm64 ;; \
    *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac; \
    qt_version=$(python3 /opt/qgc-src/tools/setup/read_config.py --get qt.version); \
    qt_modules=$(python3 /opt/qgc-src/tools/setup/read_config.py --get qt.modules); \
    mkdir -p /opt/Qt; \
    /opt/qgc-venv/bin/aqt install-qt "${qt_host}" desktop "${qt_version}" "${qt_arch}" \
    -O /opt/Qt -m ${qt_modules}; \
    ln -s "/opt/Qt/${qt_version}/${qt_dir}" /opt/Qt/current

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    python3 /opt/qgc-src/tools/setup/install_dependencies --platform debian

RUN /opt/Qt/current/bin/qt-cmake -S /opt/qgc-src -B /opt/qgc-src/build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/opt/qgc-venv/bin/python && \
    cmake --build /opt/qgc-src/build --target all --parallel "${BUILD_JOBS:-$(nproc)}" && \
    cmake --install /opt/qgc-src/build --config Release

# extract AppImage
RUN set -eux; \
    appimage=$(find /opt/qgc-src/build -maxdepth 1 -name 'QGroundControl-*.AppImage'); \
    chmod +x "${appimage}"; \
    cd /opt && "${appimage}" --appimage-extract; \
    mv squashfs-root /opt/qgroundcontrol; \
    rm -rf /opt/qgc-src /opt/qgc-venv /opt/Qt
