# This could also be another Ubuntu or Debian based distribution
FROM ros:foxy-ros-base

# Install Open3D system dependencies and pip
RUN apt-get update && apt-get install --no-install-recommends -y \
    libegl1 \
    libgl1 \
    libgomp1 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Open3D from the PyPI repositories
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --upgrade open3d

ENV EGL_PLATFORM=surfaceless WEBRTC_IP=127.0.0.1 WEBRTC_PORT=8889
WORKDIR /usr/app/src

COPY draw_webrtc.py ./
CMD ["python3", "./draw_webrtc.py"]
