# This could also be another Ubuntu or Debian based distribution
FROM ros:humble-ros-base

# Install system dependencies and pip
RUN apt-get update && apt-get install --no-install-recommends -y \
    libegl1 \
    libgl1 \
    libgomp1 \
    python3-pip \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./

# Install Open3D from the PyPI repositories
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --upgrade -r ./requirements.txt

WORKDIR /usr/app/src

COPY ./* ./
CMD ["python3", "./mapper_main.py"]

#docker build -t pathfinder .
#docker run --net host -v /dev/shm:/dev/shm pathfinder