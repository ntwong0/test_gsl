# test_gsl AKA covar
## 0. First things
Ensure that gsl is installed:
`sudo apt-get install libgsl2 libgsl-dev libgsl-dbg gsl-bin`

## 1. Getting the package
Do the following to add to the src of your ROS workspace (i.e. catkin_ws if you kept the dirname from ROS tutorial), *almost* as you would any other package - we're going to give this repo's dirname "covar" instead of "test_gsl".
* `cd path/to/catkin_ws/src; git clone https://github.com/ntwong0/test_gsl.git -b ROSify covar`

## 2. Installation
catkin_make as usual:
* `cd path/to/catkin_ws; catkin_make`

**Please note RE: the loam_velodyne package**
* Although `covar` is a general `std_msgs/Odometry` republisher, it was built with [`loam_velodyne`](https://github.com/laboshinl/loam_velodyne.git) in mind.
    * Note that its build instructions specifies the `-DCMAKE_BUILD_TYPE=Release` flag. Please obey this, otherwise your `loam_velodyne` node will not work!
* Note that loam_velodyne uses the XZ plane rather than the XY plane, so you'll to translate the axes. The `covar` launch file `loam_velodyne_with_covar.launch` creates the two static transforms you need, but if you want to run them yourself: 
    1. Tie `/camera_init` to `/world`: `rosrun tf2_ros static_transform_publisher 0 0 0  0 0  1.5708 /world /camera_init`
    2. Tie `/map` to `/world` (assuming your map tf is `/map`): `rosrun tf2_ros static_transform_publisher 0 0 0  0 0 0 /world /map`

## 3. Running
As always, be sure to `source` the appropriate `setup.bash` file, i.e.
* `cd path/to/catkin_ws; source devel/setup.bash`

You can run covar independently via `rosrun`:
* `rosrun covar covar_node`

If you're using `covar` with `loam_velodyne`, you can run the following command:
* `roslaunch covar loam_velodyne_with_covar.launch`

## 4. Service calls - this is why you "bought" it 
Generate the covariance matrix with the following:
* For pose only, 100 samples
    * `rosservice call /gen_pose_covar 100`
* For twist only, 100 samples
    * `rosservice call /gen_twist_covar 100`
* For pose and twist, 100 samples pose and 200 samples twist
    * `rosservice call /gen_both_covar 100 200`

## 5. Versioning
* 0.1.0 (published 2018-11-01 via 6c56a64)
    * [x] Republishes an input std_msgs/Odometry topic
        * [x] Excludes the covariance of the input's messages
    * [x] Generates covariances for pose and/or twist from the input std_msgs/Odometry topic
    * [x] Accepts rosservice calls to perform covariance generation
* 0.2.0 (ETA 2018-11-30)
    * [x] Provide `.launch` file for ease of execution via `roslaunch`
    * [ ] `rosparam`-ify input and output topic
    * [ ] `rosparam`-ify covariance matrices for pose and twist
    * [ ] `rosservice call` for changing covariance matrices on-the-fly
* 0.3.0 (ETA 2018-12-07)
    * [ ] Also generate covariance for accelerometer data

## 999. Dummy compilation note:
To compile, do the following:
<!-- gcc <filename> -lgsl -lgslcblas -->
g++ -std=c++11 src/file.cpp -lgsl -lgslcblas -Iinclude

Header files of the gsl library are located at /usr/include/gsl
