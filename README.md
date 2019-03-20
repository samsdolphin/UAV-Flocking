# nuc

export PATH=/usr/local/cuda-9.0/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin  
source /opt/ros/kinetic/setup.bash  
source /home/nvidia/catkin_ws/devel/setup.bash  
export ROS_MASTER_URI=http://localhost:11311  
export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/aarch64-linux-gnu:/usr/local/cuda-9.0/lib64:  
export DJIROS_APPID="1059272"  
export DJIROS_ENCKEY="436d9704acc398a571d8c85c91bb7cf1cd66c5ffdeb8632bcb1b54f56d1d1715"  
#source /home/nvidia/ELEC5660_lab_code/devel/setup.bash  
alias cmk='cd ~/catkin_ws; catkin_make; cd -'  
