# Small independent image: no Hailo, camera, ROS or GUI dependencies.
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-serial ca-certificates \
    && rm -rf /var/lib/apt/lists/*
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
WORKDIR /app
COPY elrs_monitor.py ./
CMD ["python3", "elrs_monitor.py", "--port", "/dev/elrs"]
