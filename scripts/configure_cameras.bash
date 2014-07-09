#! /bin/bash
# script to configure exposure of ceiling webcams for better marker tracking
v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_auto=1
v4l2-ctl --device=/dev/video0  --set-ctrl=exposure_absolute=50

v4l2-ctl --device=/dev/video1 --set-ctrl=exposure_auto=1
v4l2-ctl --device=/dev/video1  --set-ctrl=exposure_absolute=50
