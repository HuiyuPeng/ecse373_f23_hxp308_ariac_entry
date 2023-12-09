# ros-ariac


## Launch this project

Preparing Existing Package
Continue to use the existing ROS/ARIAC package from the Laboratory #5. The Action Server interface will be used to send trajectories to make the arm move. The arm will be moved to the location of specific parts of interest.

## Asynchronous Spinner
ROS allows the “spin” process to be run in a multi-threaded manner. By doing so, the main while loop can proceed without having to continually distribute calls to ros::spinOnce() throughout the code. Add the following code before the while loop and remove all calls to ros::spinOnce().
```
ros::AsyncSpinner spinner(1); // Use 1 thread
spinner.start(); // A spinner makes calling ros::spin() unnecessary.
```
## Joint angles (offsets) from the joint_states topic
The current position of all the joints is updated frequently on the joint_states topic using the sensor_msgs/JointState message type. The environment, however, remaps /joint_states to /ariac/arm1/joint_states. In order to keep track of the joint_states, this is the topic to which to subscribe. Create a callback that simply stores joint_states to a global for use inside the main node loop. Use the ROS_INFO[_STREAM]_THROTTLE logger to publish the current joint states and the time of the most recent message every 10 seconds.

## Inverse Kinematics
The joint angles for reaching a desired pose is provided by the ik_pose service created in an earlier laboratory. Update the main code to use this service. For this, and all services, please add a call to ros::service::waitForService(<service_name>[,timeout_in_ms]). This function has an infinite default timeout. It is best to wait for a few seconds and indicate either the service was found or that it was not and exit.)

## How the lab works

## 1. Subscriber settings
Set up four distinct subscriber types. The first should focus on receiving order information, while the remaining three are dedicated to gathering data from logical cameras. It's important to note that the callback function for logical camera subscribers needs to handle two inputs: the message and the camera number.
s
## 2. Service client settings
Develop two service clients with specific functions: one to detect the commencement of a competition, and the other to acquire information on the location of products.

## 3. Finding the position of first order
By analyzing the order information received through the subscriber, the first product's details can be ascertained. Utilizing the product type from this initial order, combined with the /ariac/material_locations service, it's possible to determine the specific storage bin for the product. The exact position of the first order is then pinpointed by examining all the data collected from the subscribers monitoring the logical bin cameras.

## 4. Transform coordinate and fix the position
After identifying the first order's location, employ the tf2::lookupTransform and tf2::doTransform tools to convert the product's position from its current frame to the arm frame. It's crucial to first find the frame associated with the product's bin. This step ensures precise arm movement towards the part, helping to prevent any collisions. Additionally, a slight adjustment of the product's position might be necessary for optimal accuracy.

