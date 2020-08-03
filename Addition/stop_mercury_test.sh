PATH_PACKAGE="/home/apptronik/Repository/Sejong_Dynamic_Control_Toolkit"

echo 'turn off video recording'
sudo kill $(pgrep avconv)

echo 'turn off data saving'
kill $(pgrep Status_Display)

echo 'turn off controller'
kill $(pgrep roslaunch)

