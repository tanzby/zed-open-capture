# export zed data script

Help you extract frames data from Rosbag file, as many VIO systems used.
!!! Python version script can not extract timestamp by real time sequence. You should use C++ version !!!

## usage

```shell script
source devel/setup.zsh
rosrun zed_open_capture anyexport_zed_data -b yourfile.bsg -s /path/to/saved/folder```
```

for example
```shell script
➜ rosrun zed_open_capture anyexport_zed_data -b 2020-11-20-01-33-05.bag -s ./       
Reading the rosbag file from 2020-11-20-01-33-05.bag
Writing sensor_msgs::Imu data to CSV, Extract Image to png
Finished extracting data!
```