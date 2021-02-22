#!/bin/bash
clear
raspivid -n -t 0 -rot 180 -w 480 -h 360 -fps 30 -b 6000000 -o - | gst-launch-1.0 -e -v fdsrc ! h264parse ! rtph264pay pt=96 config-interval=1 ! gdppay ! tcpserversink host=192.168.0.102 port=5000

#raspivid -n -t 0 -rot 180 -w 960 -h 720 -fps 30 -b 6000000 -o - | gst-launch-1.0 -e -v fdsrc ! h264parse ! rtph264pay pt=96 config-interval=1 ! gdppay ! udpsink host=192.168.0.104 port=5000

#raspivid -t 0 -rot 180 -cd MJPEG -w 960 -h 720 -fps 40 -b 6000000 -o - | gst-launch-1.0 -e -v fdsrc ! "image/jpeg,framerate=40/1" ! jpegparse ! rtpjpegpay ! udpsink host=192.168.0.101 port=5000
