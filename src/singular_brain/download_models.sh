#!/bin/bash
mkdir -p /home/konark/robot_ws/src/singular_brain/models
cd /home/konark/robot_ws/src/singular_brain/models
wget -nc https://github.com/chuanqi305/MobileNet-SSD/raw/master/mobilenet_iter_73000.caffemodel
wget -nc https://github.com/chuanqi305/MobileNet-SSD/raw/master/deploy.prototxt
