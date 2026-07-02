# iKalibr + motiv LiDAR-IMU offset wrapper
# Extends the official iKalibr image with:
#   - xvfb           : headless virtual display for the Pangolin viewer
#   - micromamba env : Python 3.11 + rosbags to read ROS2 (.mcap) rosbag2 v9 bags
#                      (the base image's Python 3.8 rosbags is too old for v9)
#   - motiv scripts  : one-command folder -> LiDAR-IMU time offset
FROM ulong2/ie_kalibr_image:latest

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
      xvfb wget bzip2 ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Modern Python (3.11) for the ROS2 mcap -> ROS1 bag conversion, isolated from ROS Noetic's 3.8
ENV MAMBA_ROOT_PREFIX=/opt/mamba
RUN wget -qO /tmp/mm.tar.bz2 https://micro.mamba.pm/api/micromamba/linux-64/latest \
 && tar -xjf /tmp/mm.tar.bz2 -C /usr/local bin/micromamba \
 && rm /tmp/mm.tar.bz2 \
 && micromamba create -y -n conv python=3.11 \
 && /opt/mamba/envs/conv/bin/pip install --no-cache-dir rosbags pyyaml numpy \
 && micromamba clean -a -y

COPY calibrate_offset.py run_ikalibr.sh entrypoint.sh ikalibr-config.template.yaml /opt/motiv/
RUN chmod +x /opt/motiv/entrypoint.sh /opt/motiv/run_ikalibr.sh /opt/motiv/calibrate_offset.py

# Usage: docker run --rm -v /path/to/Robin_folder:/folder ikalibr-motiv /folder [options]
ENTRYPOINT ["/opt/motiv/entrypoint.sh"]
