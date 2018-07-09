#!/bin/bash

rostopic pub -s -r 20 /ctrerobot/joy sensor_msgs/Joy "header:
  seq: 0
  stamp: now
  frame_id: ''
axes:
- 0
buttons: [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0] "
