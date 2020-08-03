#! /bin/bash

PATH_PACKAGE="/home/apptronik/Repository/Sejong_Dynamic_Control_Toolkit"

# avconv -y -v quiet -f video4linux2 -s 720x480 -i /dev/video0 $PATH_PACKAGE/experiment_data/test.mp4
# sudo avconv -y -v quiet -f video4linux2 -s 720x480 -i /dev/video0 $PATH_PACKAGE/experiment_data/test.mp4 &
roslaunch hume_test system.launch &
sudo avconv -y -v quiet -f video4linux2 -s 720x480 -i /dev/video0 $PATH_PACKAGE/experiment_data/test.mp4 
# $PATH_PACKAGE/Addition/Data_Manager/build/Status_Display -v &

exit 0
