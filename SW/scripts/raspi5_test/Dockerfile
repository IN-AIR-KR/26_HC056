# syntax=docker/dockerfile:1
# Build on Pi 5 (ARM64). No host /usr/lib or Python packages are bind-mounted.
FROM ubuntu:22.04 AS hailort-build
ARG DEBIAN_FRONTEND=noninteractive
# Match the reported device/host stack. Change ref AND commit together if upgrading.
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
    -DHAILO_BUILD_EXAMPLES=OFF -DHAILO_BUILD_GSTREAMER=OFF \
    -DHAILO_BUILD_SERVICE=OFF \
    && cmake --build /src/build --parallel "${BUILD_JOBS}" \
    && cmake --install /src/build && ldconfig
RUN python3 -m pip wheel --no-cache-dir --no-deps --no-build-isolation \
    --wheel-dir /wheels /src/hailort/hailort/libhailort/bindings/python/platform

FROM ubuntu:22.04 AS runtime
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates python3 python3-pip python3-opencv python3-numpy libpci3 libudev1 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=hailort-build /usr/local/ /usr/local/
COPY --from=hailort-build /wheels/ /tmp/wheels/
RUN ldconfig && python3 -m pip install --no-cache-dir /tmp/wheels/*.whl \
    && rm -rf /tmp/wheels \
    && python3 -c "import cv2, numpy, hailo_platform" \
    && hailortcli --version
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
WORKDIR /app
COPY frames.py camera_test.py hailo_test.py ./
RUN mkdir -p /shared /models /outputs
CMD ["python3", "camera_test.py", "--help"]
