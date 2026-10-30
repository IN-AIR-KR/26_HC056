# syntax=docker/dockerfile:1
# Phase-1 Pi 5 image: ROS Humble video/detection plus HailoRT.
# Full flight-stack packages are added in Phase 2 of RPI5_TRANSITION_PLAN.md.
FROM ubuntu:22.04 AS hailort-build
ARG DEBIAN_FRONTEND=noninteractive
ARG HAILORT_REF=v4.23.0
ARG HAILORT_COMMIT=08f088d3b443c7846af067269ce998c6d5d91449
ARG BUILD_JOBS=2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git build-essential cmake pkg-config python3-dev python3-pip \
    python3-setuptools python3-wheel libpci-dev libudev-dev \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch "${HAILORT_REF}" --recurse-submodules \
    https://github.com/hailo-ai/hailort.git /src/hailort \
    && test "$(git -C /src/hailort rev-parse HEAD)" = "${HAILORT_COMMIT}"
RUN cmake -S /src/hailort -B /src/build -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_INSTALL_LIBDIR=lib \
    -DHAILO_BUILD_EXAMPLES=OFF -DHAILO_BUILD_GSTREAMER=OFF -DHAILO_BUILD_SERVICE=OFF \
    && cmake --build /src/build --parallel "${BUILD_JOBS}" \
    && cmake --install /src/build && ldconfig
RUN python3 -m pip wheel --no-cache-dir --no-deps --no-build-isolation \
    --wheel-dir /wheels /src/hailort/hailort/libhailort/bindings/python/platform \
    && python3 -m pip wheel --no-cache-dir --no-build-isolation \
    --wheel-dir /wheels /wheels/hailort-*.whl

FROM ros:humble
ARG DEBIAN_FRONTEND=noninteractive
COPY --from=hailort-build /usr/local/ /usr/local/
COPY --from=hailort-build /wheels/ /tmp/wheels/
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-opencv python3-numpy libpci3 libudev1 \
    ros-humble-sensor-msgs ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
    && ldconfig \
    && python3 -m pip install --no-cache-dir --no-index --find-links=/tmp/wheels \
       --only-binary=:all: /tmp/wheels/hailort-*.whl \
    && rm -rf /tmp/wheels \
    && python3 -c "import cv2, numpy, hailo_platform" \
    && hailortcli --version

WORKDIR /ros_ws/src
COPY arms_ws/src/arms_msgs/ arms_msgs/
COPY arms_ws/src/arms_video/ arms_video/
COPY arms_ws/src/arms_detection/ arms_detection/
WORKDIR /ros_ws
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build \
    --packages-select arms_msgs arms_video arms_detection \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY deploy/raspi5/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /models /run/arms-csi
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
