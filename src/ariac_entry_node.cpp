#include <string>
#include <math.h>
#include <ros/ros.h>
// Include the begin file for the Trigger service from the std_srvs package.
#include <std_srvs/Trigger.h>
// Include the header file for the SetBool service from the std_srvs package.
#include "std_srvs/SetBool.h"

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/GetMaterialLocations.h>

// Transformation header files
#include "tf2_ros/tranform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

// Declare the variable in this way where necessary in the code.
std_srvs::Trigger begin_comp;
// Declare the variable in this way where necessary in the code.
std_srvs::SetBool my_bool_var;
// my_bool_var.request.data = true;

// Declaring a vector of data type.
std::vector<osrf_gear::Order> order_vector;
std::vector<osrf_gear::LogicalCameraImage> logic_camera_bin_vector, logic_camera_agv_vector, quality_control_sensor_vector;

osrf_gear::GetMaterialLocations get_loc_message;
osrf_gear::LogicalCameraImage camera_message;

geometry_msgs::TransformStamped tfStamped;
geometry_msgs::PoseStamped part_pose, goal_pose;

int total_logical_camera_bin_num = 6;
int total_logical_camera_agv_num = 2;
int total_quality_control_sensor_num = 2;
bool show_first_product_msg_once = true;
bool has_shown_first_order_msg = false;

/// Called when a new Order message is received.
void order_callback(const osrf_gear::Order::ConstPtr &order_msg)
{
    received_orders_.push_back(*order_msg);
}

void bin_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr &order_msg, int bin_camera_num)
{
    logic_camera_bin_vector[bin_camera_num] = *order_msg;
}

void agv_camera_callback(const osrf_gear::Order::ConstPtr &order_msg)
{
    logic_camera_agv_vector[agv_camera_num] = *order_msg;
}

void quality_camera_callback(const osrf_gear::Order::ConstPtr &order_msg)
{
    quality_control_sensor_vector[quality_camera_num] = *quality_order_msg;
}

int main(int argc, char **argv)
{
    // Last argument is the default name of the node.
    ros::init(argc, argv, "ariac_example_node");
    // Init the node
    ros::NodeHandle node;

    // Declare the transformation buffer to maintain a list of transformations
    tf2_ros::Buffer tfBuffer;
    // Instantiate a listener that listens to the tf and tf_static topics and to update the
    buffer.tf2_ros::TransformListener tfListener(tfBuffer);

    // Initiate variables
    order_vector.clear();
    logic_camera_agv_vector.clear();
    logic_camera_agv_vector.resize(2);
    logic_camera_bin_vector.clear();
    logic_camera_bin_vector.resize(6);
    quality_control_sensor_vector.clear();
    quality_control_sensor_vector.resize(2);

    // Variable to capture service call success.
    int service_call_succeeded;
    int get_location_call_succeeded;

    std::string logical_camera_name, logical_camera_bin_frame;
    ros::Subscriber camera_sub[6];

    my_bool_var.request.data = true;

    // Init Service client
    ros::ServiceClient begin_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient get_location_client = node.serviceClient<osrf_gear::GetMaterialLocation>("/ariac/material_locations");

    // Init Subscriber
    ros::Subscriber order_subscriber = node.subscribe("/ariac/orders", 1000, order_callback);

    // create 6 order_callback with 6 logical camera_bin
    for (int i = 0; i < total_logical_camera_bin_num; i++)
    {
        logical_camera_name = "ariac/logical_camera_bin" + std::to_string(i);
        camera_sub[i] = node.subscriber<osrf_gear::LogicalCameraImage>(logical_camera_name, 10, boost::bind(bin_camera_callback, _1, i));
    }

    ros::Subscriber agv_camera_sub1 = node.subscriber<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 10, boost::bind(agv_camera_callback, _1, 0));
    ros::Subscriber agv_camera_sub2 = node.subscriber<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 10, boost::bind(agv_camera_callback, _1, 1));

    ros::Subscriber quality_camera_sub1 = node.subscriber<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1", 10, boost::bind(quality_camera_callback, _1, 0));
    ros::Subscriber quality_camera_sub2 = node.subscriber<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2", 10, boost::bind(quality_camera_callback, _1, 1));

    // Call the Service
    service_call_succeeded = begin_client.call(begin_comp);
    // Sample output for failed service call.
    if (service_call_succeeded == 0)
    {
        ROS_ERROR("Competition service call failed! Goodness Gracious!!");
    }
    else
    {
        // Sample output for service success.
        if (begin_comp.response.success)
        {
            ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
        }
        // Sample output for service failure.
        else
        {
            ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
        }
    }

    // Set the frequency of loop in the node
    ros::Rate loop_rate(10);

    while (ros::ok() && service_call_succeeded)
    {
        if (order_vector.size() > 0)
        {
            osrf_gear::Order first_order = order_vector[0];
            osrf_gear::Shipment first_shipment = first_order.shipments[0];
            osrf_gear::Product first_product = first_shipment.products[0];
            get_loc_message.request.material_type = first_product.type;
            get_loc_call_succeeded = get_loc_client.call(get_loc_message);

            ROS_INFO_ONCE("Received order successfully! The first product type is : [%s]", first_product.type.c_str());

            if (get_location_call_succeeded)
            {
                ROS_INFO_ONCE("The storage location of the material type [%s] is: [%s]", first_product.type.c_str(), get_loc_message.response.storage_units[0].unit_id.c_str());
                // Search all logic camera data
                for (int j = 0; j < total_logical_camera_bin_num; j++)
                {
                    camera_message = logic_camera_bin_vector[j];
                    for (int k = 0; k < camera_message.models.size(); k++)
                    {
                        osrf_gear::Model product_mode = camera_message.models[k];
                        if (first_product.type == product_mode.type)
                        {
                            ROS_INFO_ONCE("The position of the first product is [x = %f, y = %f, z = %f]", product_mode.pose.position.x, product_mode.pose.position.y, product_mode.pose.position.z);
                            ROS_INFO_ONCE("The orientation of the first product is [qx = %f, qy = %f, qz = %f, qw = %f]", product_mode.pose.orientation.x, product_mode.pose.orientation.y, product_mode.pose.orientation.z, product_mode.pose.orientation.w);

                            logical_camera_bin_frame = "logical_camera_bin" + std::to_string(j) + "_frame";

                            // Retrieve the transformation
                            try
                            {
                                tfStamped = tfBuffer.lookupTransform("arm1_base_link", logical_camera_bin_frame, ros::Time(0.0), ros::Duration(1.0));
                                ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
                            }
                            catch (tf2::TransformException &ex)
                            {
                                ROS_ERROR("%s", ex.what());
                            }

                            // Copy pose from the logical camera.
                            part_pose.pose = product_mode.pose;
                            tf2::doTransform(part_pose, goal_pose, tfStamped);

                            // Given position
                            // Add height to the goal pose.
                            goal_pose.pose.position.z += 0.10; // 10 cm above the part
                            // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
                            goal_pose.pose.orientation.w = 0.707;
                            goal_pose.pose.orientation.x = 0.0;
                            goal_pose.pose.orientation.y = 0.707;
                            goal_pose.pose.orientation.z = 0.0;
                            ROS_WARN_ONCE("The pose position of transformed location of the first product: [x = %f], [y = %f], [z = %f]", product_mode.pose.position.x, product_mode.pose.position.y, product_mode.pose.position.z);
                            break;
                        }
                    }
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// Instance of custom class from above.
MyCompetitionClass comp_class(node);

// Subscribe to the '/ariac/current_score' topic.
ros::Subscriber current_score_subscriber = node.subscribe(
    "/ariac/current_score", 10,
    &MyCompetitionClass::current_score_callback, &comp_class);

// Subscribe to the '/ariac/competition_state' topic.
ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,
    &MyCompetitionClass::competition_state_callback, &comp_class);

// %Tag(SUB_CLASS)%
// Subscribe to the '/ariac/orders' topic.
ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);
// %EndTag(SUB_CLASS)%

// Subscribe to the '/ariac/joint_states' topic.
ros::Subscriber arm_1_joint_state_subscriber = node.subscribe(
    "/ariac/arm1/joint_states", 10,
    &MyCompetitionClass::arm_1_joint_state_callback, &comp_class);

ros::Subscriber arm_2_joint_state_subscriber = node.subscribe(
    "/ariac/arm2/joint_states", 10,
    &MyCompetitionClass::arm_2_joint_state_callback, &comp_class);

// %Tag(SUB_FUNC)%
// Subscribe to the '/ariac/proximity_sensor_1' topic.
ros::Subscriber proximity_sensor_subscriber = node.subscribe(
    "/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
// %EndTag(SUB_FUNC)%

// Subscribe to the '/ariac/break_beam_1_change' topic.
ros::Subscriber break_beam_subscriber = node.subscribe(
    "/ariac/break_beam_1_change", 10,
    &MyCompetitionClass::break_beam_callback, &comp_class);

// Subscribe to the '/ariac/logical_camera_1' topic.
ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_1", 10,
    &MyCompetitionClass::logical_camera_callback, &comp_class);

// Subscribe to the '/ariac/laser_profiler_1' topic.
ros::Subscriber laser_profiler_subscriber = node.subscribe(
    "/ariac/laser_profiler_1", 10, laser_profiler_callback);

ROS_INFO("Setup complete.");
start_competition(node);
ros::spin(); // This executes callbacks on new data until ctrl-c.

return 0;
}

/* // Declaring a vector of data type.
std::vector<std_msgs::int> int_vector;
// Clearing/initializing vector
int_vector.clear();
// Add information to the end of the vector
int_vector.push_back(5);
// HINT: The vector type used in your code will not be of type std_msgs::int.



// Retrieve the transformation
geometry_msgs::TransformStamped tfStamped;
try
{
    tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame",
                                         ros::Time(0.0), ros::Duration(1.0));
    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
              tfStamped.child_frame_id.c_str());
}
catch (tf2::TransformException &ex)
{
    ROS_ERROR("%s", ex.what());
}
// tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent","how_long_to_wait_for_transform");

// Create variables
geometry_msgs::PoseStamped part_pose, goal_pose;
// Copy pose from the logical camera.
part_pose.pose = ...;
tf2::doTransform(part_pose, goal_pose, transformStamped);



    /// Start the competition by waiting for and then calling the start ROS Service.
    void start_competition(ros::NodeHandle & node)
{

    // Calling the Service using the client before the server is ready would fail.
    if (!start_client.exists())
    {
        ROS_INFO("Waiting for the competition to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Competition is now ready.");
    }
    ROS_INFO("Requesting competition start...");
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    start_client.call(srv); // Call the start Service.
    if (!srv.response.success)
    { // If not successful, print out why.
        ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
    }
    else
    {
        ROS_INFO("Competition started!");
    }
}
// %EndTag(START_COMP)%

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
    explicit MyCompetitionClass(ros::NodeHandle &node)
        : current_score_(0), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false)
    {
        // %Tag(ADV_CMD)%
        arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm1/arm/command", 10);

        arm_2_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm2/arm/command", 10);
        // %EndTag(ADV_CMD)%
    }

    /// Called when a new message is received.
    void current_score_callback(const std_msgs::Float32::ConstPtr &msg)
    {
        if (msg->data != current_score_)
        {
            ROS_INFO_STREAM("Score: " << msg->data);
        }
        current_score_ = msg->data;
    }

    /// Called when a new message is received.
    void competition_state_callback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "done" && competition_state_ != "done")
        {
            ROS_INFO("Competition ended.");
        }
        competition_state_ = msg->data;
    }

    /// Called when a new Order message is received.
    void order_callback(const osrf_gear::Order::ConstPtr &order_msg)
    {
        ROS_INFO_STREAM("Received order:\n"
                        << *order_msg);
        received_orders_.push_back(*order_msg);
    }

    // %Tag(CB_CLASS)%
    /// Called when a new JointState message is received.
    void arm_1_joint_state_callback(
        const sensor_msgs::JointState::ConstPtr &joint_state_msg)
    {
        ROS_INFO_STREAM_THROTTLE(10,
                                 "Joint States arm 1 (throttled to 0.1 Hz):\n"
                                     << *joint_state_msg);
        // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
        arm_1_current_joint_states_ = *joint_state_msg;
        if (!arm_1_has_been_zeroed_)
        {
            arm_1_has_been_zeroed_ = true;
            ROS_INFO("Sending arm to zero joint positions...");
            send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
        }
    }

    void arm_2_joint_state_callback(
        const sensor_msgs::JointState::ConstPtr &joint_state_msg)
    {
        ROS_INFO_STREAM_THROTTLE(10,
                                 "Joint States arm 2 (throttled to 0.1 Hz):\n"
                                     << *joint_state_msg);
        // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
        arm_2_current_joint_states_ = *joint_state_msg;
        if (!arm_2_has_been_zeroed_)
        {
            arm_2_has_been_zeroed_ = true;
            ROS_INFO("Sending arm 2 to zero joint positions...");
            send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
        }
    }
    // %EndTag(CB_CLASS)%

    // %Tag(ARM_ZERO)%
    /// Create a JointTrajectory with all positions set to zero, and command the arm.
    void send_arm_to_zero_state(ros::Publisher &joint_trajectory_publisher)
    {
        // Create a message to send.
        trajectory_msgs::JointTrajectory msg;

        // Fill the names of the joints to be controlled.
        // Note that the vacuum_gripper_joint is not controllable.
        msg.joint_names.clear();
        msg.joint_names.push_back("shoulder_pan_joint");
        msg.joint_names.push_back("shoulder_lift_joint");
        msg.joint_names.push_back("elbow_joint");
        msg.joint_names.push_back("wrist_1_joint");
        msg.joint_names.push_back("wrist_2_joint");
        msg.joint_names.push_back("wrist_3_joint");
        msg.joint_names.push_back("linear_arm_actuator_joint");
        // Create one point in the trajectory.
        msg.points.resize(1);
        // Resize the vector to the same length as the joint names.
        // Values are initialized to 0.
        msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
        // How long to take getting to the point (floating point seconds).
        msg.points[0].time_from_start = ros::Duration(0.001);
        ROS_INFO_STREAM("Sending command:\n"
                        << msg);
        joint_trajectory_publisher.publish(msg);
    }
    // %EndTag(ARM_ZERO)%

    /// Called when a new LogicalCameraImage message is received.
    void logical_camera_callback(
        const osrf_gear::LogicalCameraImage::ConstPtr &image_msg)
    {
        ROS_INFO_STREAM_THROTTLE(10,
                                 "Logical camera: '" << image_msg->models.size() << "' objects.");
    }

    /// Called when a new Proximity message is received.
    void break_beam_callback(const osrf_gear::Proximity::ConstPtr &msg)
    {
        if (msg->object_detected)
        { // If there is an object in proximity.
            ROS_INFO("Break beam triggered.");
        }
    }

private:
    std::string competition_state_;
    double current_score_;
    ros::Publisher arm_1_joint_trajectory_publisher_;
    ros::Publisher arm_2_joint_trajectory_publisher_;
    std::vector<osrf_gear::Order> received_orders_;
    sensor_msgs::JointState arm_1_current_joint_states_;
    sensor_msgs::JointState arm_2_current_joint_states_;
    bool arm_1_has_been_zeroed_;
    bool arm_2_has_been_zeroed_;
};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if ((msg->max_range - msg->range) > 0.01)
    { // If there is an object in proximity.
        ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
    }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    size_t number_of_valid_ranges = std::count_if(
        msg->ranges.begin(), msg->ranges.end(), [](const float f)
        { return std::isfinite(f); });
    if (number_of_valid_ranges > 0)
    {
        ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
    }
}

// %Tag(MAIN)%
int main(int argc, char **argv)
{
    // Last argument is the default name of the node.
    ros::init(argc, argv, "ariac_example_node");

    ros::NodeHandle node;

    // Instance of custom class from above.
    MyCompetitionClass comp_class(node);

    // Subscribe to the '/ariac/current_score' topic.
    ros::Subscriber current_score_subscriber = node.subscribe(
        "/ariac/current_score", 10,
        &MyCompetitionClass::current_score_callback, &comp_class);

    // Subscribe to the '/ariac/competition_state' topic.
    ros::Subscriber competition_state_subscriber = node.subscribe(
        "/ariac/competition_state", 10,
        &MyCompetitionClass::competition_state_callback, &comp_class);

    // %Tag(SUB_CLASS)%
    // Subscribe to the '/ariac/orders' topic.
    ros::Subscriber orders_subscriber = node.subscribe(
        "/ariac/orders", 10,
        &MyCompetitionClass::order_callback, &comp_class);
    // %EndTag(SUB_CLASS)%

    // Subscribe to the '/ariac/joint_states' topic.
    ros::Subscriber arm_1_joint_state_subscriber = node.subscribe(
        "/ariac/arm1/joint_states", 10,
        &MyCompetitionClass::arm_1_joint_state_callback, &comp_class);

    ros::Subscriber arm_2_joint_state_subscriber = node.subscribe(
        "/ariac/arm2/joint_states", 10,
        &MyCompetitionClass::arm_2_joint_state_callback, &comp_class);

    // %Tag(SUB_FUNC)%
    // Subscribe to the '/ariac/proximity_sensor_1' topic.
    ros::Subscriber proximity_sensor_subscriber = node.subscribe(
        "/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
    // %EndTag(SUB_FUNC)%

    // Subscribe to the '/ariac/break_beam_1_change' topic.
    ros::Subscriber break_beam_subscriber = node.subscribe(
        "/ariac/break_beam_1_change", 10,
        &MyCompetitionClass::break_beam_callback, &comp_class);

    // Subscribe to the '/ariac/logical_camera_1' topic.
    ros::Subscriber logical_camera_subscriber = node.subscribe(
        "/ariac/logical_camera_1", 10,
        &MyCompetitionClass::logical_camera_callback, &comp_class);

    // Subscribe to the '/ariac/laser_profiler_1' topic.
    ros::Subscriber laser_profiler_subscriber = node.subscribe(
        "/ariac/laser_profiler_1", 10, laser_profiler_callback);

    ROS_INFO("Setup complete.");
    start_competition(node);
    ros::spin(); // This executes callbacks on new data until ctrl-c.

    return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%
 */