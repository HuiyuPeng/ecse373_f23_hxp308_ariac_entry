# ecse373_f23_hxp308_ariac_entry
## Install ARIAC 2019
### Create a catkin workspace for the simulation environment
`mkdir -p ~/ecse_373_ariac_ws/src`
`cd ~/ecse_373_ariac_ws/src`
### Clone the repository
`git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git`
### Install any missing dependencies
`rosdep install --from-paths ariac --ignore-src -r -y`
### Add it to your ROS environment.
`cd ../`
`catkin_make`
`source devel/setup.bash`
### Launch a modified version of the competition environment.
`roslaunch ecse_373_ariac ecse_373_ariac.launch`


