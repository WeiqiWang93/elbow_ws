# ROS package bot_common_ros README
## made by OffWorld Inc. (www.offworld.ai)


### How To: auto calibrate cameras with respect to arm base
#### Steps:
* Powering chisel/saw arm  \
            1. Turn red knob to on position.\
            2. Turn on inverter switch (about 2 inches below OBP power button).\
            3. Press green button on UR 10 box (wait for sound).
* Connecting to chisel bot:\
            1. Connect to chisel ethernet network.\
            2. Ping 192.168.3.102 to verify connection.\
            3. Open VNC viewer and connect to chisel (password: chiselbot).

* Attach checkerboard attachment to end-effector.  
* In bot_common_ros/launch/camera_calibration_new.launch set use1/2/3/4 to true and others to false depending on which camera we are calibrating.\
* run `roslaunch bot_common_ros camera_calibration_new.launch`
* Adjust the end effector using VNC so that the checkerboard is completely in the FOV of the camera.
* run `rosrun tuw_checkerboard tuw_checkerboard_node image:=/camera_01/rgb/image_raw camera_info:=/camera_01/rgb/camera_info` adjusting for the specific camera number
* Check the image to ensure checkerboard were successfully found.
* run `rosrun dynamic_reconfigure dynparam set /tuw_checkerboard checkerboard_square_size 0.024` to set the square size
* If the camera image is rotated (check by looking at the axes of TCP and checkerboard frame in rviz), then run `rosrun dynamic_reconfigure dynparam set /tuw_checkerboard rotate_camera_image_180 true` to rotate by 180 degrees.
* Set the rosparam values between checkerboard and nominal TCP such that the axes of checkerboard in RVIZ align with the tuw image. Also verify that the chisel TCP coincides with the checkerboard.
* Load the params files (containing the fixed extrinsics TCP to base) by running the following:\
            1. `roscd bot_common_ros/config/params`\
            2. `rosparam load arm_params.yaml`\
            3. `rosparam load transform_params.yaml`
* run `rosrun bot_common_ros ur10_driver.py _input_param_path:=/ur10_driver/arm_2_chisel/params  _dry_run:=False`
* In arm params edit arm/name in checkerboard calibration depending on whether the cameras are wrt saw base or chisel base
checkerboard_calibration:\
  arm:\
    id: 1\
    name: "/chisel"\
    TCP: [-0.084, 0.06, 0.025, 0.0, 0.0, -1.57] # position and orientation of TCP w.r.t. Nominal-TCP\
    mass: 0.0\
    CG: [0.0, 0.0, 0.0] # center of gravity in m of end-effector
* run `rosrun tf tf_echo "saw_base" "camera_04_link"` with the correct "base" and "camera_number"

<!---
Commented out toby's lines for now
<run `camera_auto_calibration.sh [camera_id]`, don't forget the leading `0` 
#copy the translation and quaternion of output of form 
```
-->
* Expected Output:
```
At time 1253924111.082
- Translation: [-1.989, 0.151, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.046, 0.999]
            in RPY [0.000, -0.000, -0.092]
```
* Copy and paste the values into `camera_calibration.yaml` file.

<!--into the corresponding values of `camera_calibration.yaml` file.-->

#### Load calibrated camera parameters
TODO: finalize this, it's not quite right  
Add the following code to your launch file
```
<arg name="device_1_prefix" default="01"/>
<rosparam command="load" file="$(find bot_common_ros)/config/params/camera_calibration.yaml"/>
<node pkg="tf" type="static_transform_publisher" name="camera_broadcaster1" args="x y z qx qy qz qw base camera_0$(arg device_1_prefix) 100" />
```

# Testing

## test ur10_driver
load rosparams from command line in the tests directory
```
roscd bot_common_ros/tests
rosparam load test_params.yaml
```
command line run, specifying if it's in sim and if using api  
`rosrun bot_common_ros ur10_driver.py _input_param_path:=/ur10_driver/arm_1/params _dry_run:=False _run_test:=True _run_api:=True _sim:=True`

## Run ur10_driver server
load rosparams from command line in the tests directory
```
roscd bot_common_ros/tests
rosparam load test_params.yaml
```
command line run, specifying if it's in sim and if using api  
`rosrun bot_common_ros ur10_driver.py _input_param_path:=/ur10_driver/arm_1/params _dry_run:=False`

## Run robo-saw-action-server
Remember to fill the inputs correctly; whether it's in sim, whether to run the arm-client as an api, input ros-topics.  
```
cd $OFFWORLD_ROOT/Code/offworld-ur/Chiseling\ Baselines
python robo_saw_action_server.py _sim:=True _client_topic:=/test_ur10_driver/arm _input_param_path:=/ur10_driver/arm_1/params _run_api:=False
```
