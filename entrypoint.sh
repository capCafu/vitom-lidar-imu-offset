#!/bin/bash
# Run the offset driver with the conversion env's Python (3.11 + rosbags).
exec /opt/mamba/envs/conv/bin/python /opt/motiv/calibrate_offset.py "$@"
