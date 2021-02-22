#!/bin/bash

raspivid -n -rot 180 -w 640 -h 480 -t 0 -b 9000000 -fps 20 -o - | nc 192.168.1.104 5000

#raspivid -n -cd MJPEG -w 640 -h 480 -o - -t 0 -rot 180 -b 9000000 | nc 192.168.1.104 5000

#raspivid -t 0 -rot 180 -w 960 -h 720 -hf -ih -fps 30 -o - | nc -k -l 5000