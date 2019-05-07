#!/bin/bash

rostopic pub --once /lidarDat inspec_msg/lidardat "{distance:[5,5,5,5,5,5,5,5]}"
rostopic pub --once /linedetector/lines2d inspec_msg/line2d_array "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
lines:
- {dx: 1.0, dy: 1.0, x0: 0.0, y0: -20.0, id: 0}"
