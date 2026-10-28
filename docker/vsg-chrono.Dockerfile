# Prebuilt VulkanSceneGraph + Chrono base image.

FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    git build-essential cmake \
    libeigen3-dev \
    libvulkan-dev \
    libxcb1-dev \
    mesa-vulkan-drivers \
    ninja-build \
    pkg-config \
    vulkan-tools

# ---------------------------------------------------------------------------- #
#                                      VSG                                     #
# ---------------------------------------------------------------------------- #

ENV VSG_INSTALL_DIR=/opt/vsg

RUN git clone -c advice.detachedHead=false --depth 1 --branch 16.1.0 \
    https://github.com/KhronosGroup/glslang.git /tmp/glslang && \
    cmake -GNinja -B /tmp/build_glslang -S /tmp/glslang \
    -DBUILD_SHARED_LIBS=ON -DENABLE_OPT=0 && \
    cmake --build /tmp/build_glslang --parallel ${BUILD_JOBS:-$(nproc)} && \
    cmake --install /tmp/build_glslang --prefix ${VSG_INSTALL_DIR} && \
    rm -rf /tmp/glslang /tmp/build_glslang

RUN git clone -c advice.detachedHead=false --depth 1 --branch v4.4.2 \
    https://github.com/KhronosGroup/KTX-Software.git /tmp/ktx && \
    cmake -GNinja -B /tmp/build_ktx -S /tmp/ktx \
    -DBUILD_SHARED_LIBS=ON \
    -DKTX_FEATURE_TESTS=OFF \
    -DKTX_FEATURE_TOOLS=OFF \
    -DKTX_FEATURE_DOC=OFF && \
    cmake --build /tmp/build_ktx --parallel ${BUILD_JOBS:-$(nproc)} && \
    cmake --install /tmp/build_ktx --prefix ${VSG_INSTALL_DIR} && \
    rm -rf /tmp/ktx /tmp/build_ktx

RUN git clone -c advice.detachedHead=false --depth 1 --branch 1.5.7 \
    https://github.com/google/draco.git /tmp/draco && \
    cmake -GNinja -B /tmp/build_draco -S /tmp/draco \
    -DBUILD_SHARED_LIBS=ON \
    -DDRACO_TESTS=OFF && \
    cmake --build /tmp/build_draco --parallel ${BUILD_JOBS:-$(nproc)} && \
    cmake --install /tmp/build_draco --prefix ${VSG_INSTALL_DIR} && \
    rm -rf /tmp/draco /tmp/build_draco

RUN git clone -c advice.detachedHead=false --depth 1 --branch v6.0.5 \
    https://github.com/assimp/assimp /tmp/assimp && \
    cmake -GNinja -B /tmp/build_assimp -S /tmp/assimp \
    -DBUILD_SHARED_LIBS=OFF \
    -DASSIMP_BUILD_TESTS=OFF \
    -DASSIMP_BUILD_ASSIMP_TOOLS=OFF \
    -DASSIMP_BUILD_ZLIB=ON && \
    cmake --build /tmp/build_assimp --parallel ${BUILD_JOBS:-$(nproc)} && \
    cmake --install /tmp/build_assimp --prefix ${VSG_INSTALL_DIR} && \
    rm -rf /tmp/assimp /tmp/build_assimp

RUN git clone -c advice.detachedHead=false --depth 1 --branch v1.1.15 \
    https://github.com/vsg-dev/VulkanSceneGraph.git /tmp/vsg && \
    cmake -GNinja -B /tmp/build_vsg -S /tmp/vsg \
    -DCMAKE_PREFIX_PATH=${VSG_INSTALL_DIR} \
    -DBUILD_SHARED_LIBS=ON && \
    cmake --build /tmp/build_vsg --parallel ${BUILD_JOBS:-$(nproc)} && \
    cmake --install /tmp/build_vsg --prefix ${VSG_INSTALL_DIR} && \
    rm -rf /tmp/vsg /tmp/build_vsg

RUN git clone -c advice.detachedHead=false --depth 1 --branch v1.1.12 \
    https://github.com/vsg-dev/vsgXchange.git /tmp/vsgXchange && \
    ASSIMP_CMAKE=$(find ${VSG_INSTALL_DIR}/lib/cmake -maxdepth 1 -name "assimp-*" -type d | head -1) && \
    cmake -GNinja -B /tmp/build_vsgXchange -S /tmp/vsgXchange \
    -DCMAKE_PREFIX_PATH=${VSG_INSTALL_DIR} \
    -DBUILD_SHARED_LIBS=ON \
    -Dvsg_DIR=${VSG_INSTALL_DIR}/lib/cmake/vsg \
    -Dassimp_DIR=${ASSIMP_CMAKE} && \
    cmake --build /tmp/build_vsgXchange --parallel ${BUILD_JOBS:-$(nproc)} && \
    cmake --install /tmp/build_vsgXchange --prefix ${VSG_INSTALL_DIR} && \
    rm -rf /tmp/vsgXchange /tmp/build_vsgXchange

RUN git clone -c advice.detachedHead=false --depth 1 --branch v0.7.0 \
    https://github.com/vsg-dev/vsgImGui.git /tmp/vsgImGui && \
    cmake -GNinja -B /tmp/build_vsgImGui -S /tmp/vsgImGui \
    -DCMAKE_PREFIX_PATH=${VSG_INSTALL_DIR} \
    -DBUILD_SHARED_LIBS=ON \
    -Dvsg_DIR=${VSG_INSTALL_DIR}/lib/cmake/vsg && \
    cmake --build /tmp/build_vsgImGui --parallel ${BUILD_JOBS:-$(nproc)} && \
    cmake --install /tmp/build_vsgImGui --prefix ${VSG_INSTALL_DIR} && \
    rm -rf /tmp/vsgImGui /tmp/build_vsgImGui

# ---------------------------------------------------------------------------- #
#                                    CHRONO                                    #
# ---------------------------------------------------------------------------- #

3# Pinned so the image is reproducible: a run's environment.json records this SHA and
# `sim chrono run` refuses to compare results across different ones. Bump deliberately,
# re-check docker/patches/ against the new tree, and rebuild the image.
ARG CHRONO_COMMIT=dbdc98932c496f10d37bae3f914cf148c19e9a49

RUN git clone --filter=blob:none --no-checkout \
    https://github.com/projectchrono/chrono.git /home/trickfire/chrono && \
    git -C /home/trickfire/chrono checkout --detach ${CHRONO_COMMIT} && \
    git -C /home/trickfire/chrono submodule update --init --recursive --depth 1 && \
    git -C /home/trickfire/chrono rev-parse HEAD > /home/trickfire/chrono-commit.txt

# Patches carried against the pinned tree. Each must apply with no fuzz, so a Chrono bump
# that makes one obsolete fails the image build instead of silently dropping the fix.
COPY docker/patches/ /tmp/chrono-patches/
RUN for patch in /tmp/chrono-patches/*.patch; do \
    echo "applying $(basename "${patch}")" && \
    git -C /home/trickfire/chrono apply --verbose "${patch}" || exit 1; \
    done && rm -rf /tmp/chrono-patches

RUN cmake -S /home/trickfire/chrono -B /home/trickfire/chrono/build \
    -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=${VSG_INSTALL_DIR} \
    -DCH_ENABLE_MODULE_VEHICLE=ON \
    -DCH_ENABLE_MODULE_VSG=ON \
    -Dvsg_DIR=${VSG_INSTALL_DIR}/lib/cmake/vsg \
    -DvsgXchange_DIR=${VSG_INSTALL_DIR}/lib/cmake/vsgXchange \
    -DvsgImGui_DIR=${VSG_INSTALL_DIR}/lib/cmake/vsgImGui

RUN cmake --build /home/trickfire/chrono/build \
    --target demo_VEH_SCMTerrain_RigidTire \
    --parallel ${BUILD_JOBS:-$(nproc)}
