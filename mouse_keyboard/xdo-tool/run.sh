#!/bin/bash
#to enable remote running
pkill gedit
export DISPLAY=:0 
(gedit&) 2> /dev/null 
sleep 10
./xdo
