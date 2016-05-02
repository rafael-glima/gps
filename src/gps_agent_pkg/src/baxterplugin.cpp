#include "gps_agent_pkg/baxterplugin.h"
#include "gps_agent_pkg/positioncontroller.h"
#include "gps_agent_pkg/trialcontroller.h"
#include "gps_agent_pkg/encodersensor.h"
#include "gps_agent_pkg/util.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <typeinfo>

// ros::spin();


namespace gps_control {

// Plugin constructor.
GPSBAXTERPlugin::GPSBAXTERPlugin()
{
    // Some basic variable initialization.
    controller_counter_ = 0;
    controller_step_length_ = 50;
}

// Destructor.
GPSBAXTERPlugin::~GPSBAXTERPlugin()
{
    // Nothing to do here, since all instance variables are destructed automatically.
}

// Initialize the object and store the robot state.
bool GPSBAXTERPlugin::init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& n)
{
    // ROS_INFO_STREAM("baxterplugin initialization called with robot: ");

    // Variables.
    std::string root_name, active_tip_name, passive_tip_name;

    // Store the robot state.
    robot_ = robot;

    // Create FK solvers.
    // Get the name of the root.
    if(!n.getParam("root_name", root_name)) {
        ROS_ERROR("Property root_name not found in namespace: '%s'", n.getNamespace().c_str());
        return false;
    }

    // Get active and passive arm end-effector names.
    if(!n.getParam("active_tip_name", active_tip_name)) {
        ROS_ERROR("Property active_tip_name not found in namespace: '%s'", n.getNamespace().c_str());
        return false;
    }
    if(!n.getParam("passive_tip_name", passive_tip_name)) {
        ROS_ERROR("Property passive_tip_name not found in namespace: '%s'", n.getNamespace().c_str());
        return false;
    }

    // Create active arm chain.
    if(!active_arm_chain_.init(robot_, root_name, active_tip_name)) {
        ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), active_tip_name.c_str());
        return false;
    }

    // Create passive arm chain.
    if(!passive_arm_chain_.init(robot_, root_name, passive_tip_name)) {
        ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), passive_tip_name.c_str());
        return false;
    }

    // Create KDL chains, solvers, etc.
    // KDL chains.
    passive_arm_chain_.toKDL(passive_arm_fk_chain_);
    active_arm_chain_.toKDL(active_arm_fk_chain_);

    // Pose solvers.
    passive_arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(passive_arm_fk_chain_));
    active_arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(active_arm_fk_chain_));

    // Jacobian sovlers.
    passive_arm_jac_solver_.reset(new KDL::ChainJntToJacSolver(passive_arm_fk_chain_));
    active_arm_jac_solver_.reset(new KDL::ChainJntToJacSolver(active_arm_fk_chain_));

    // Pull out joint states.
    int joint_index;

    // Put together joint states for the active arm.
    joint_index = 1;
    while (true)
    {
        // Check if the parameter for this active joint exists.
        std::string joint_name;
        std::string param_name = std::string("/active_arm_joint_name_" + to_string(joint_index));
        if(!n.getParam(param_name.c_str(), joint_name))
            break;

        // Push back the joint state and name.
        pr2_mechanism_model::JointState* jointState = robot_->getJointState(joint_name);
        active_arm_joint_state_.push_back(jointState);
        ROS_INFO_STREAM("jointState name: " + joint_name + "and jointState: ");
        if (jointState == NULL)
            ROS_INFO_STREAM("jointState: " + joint_name + " is null");

        active_arm_joint_names_.push_back(joint_name);

        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the active arm joint state.
    if (active_arm_fk_chain_.getNrOfJoints() != active_arm_joint_state_.size())
    {
        ROS_INFO_STREAM("num_fk_chain: " + to_string(active_arm_fk_chain_.getNrOfJoints()));
        ROS_INFO_STREAM("num_joint_state: " + to_string(active_arm_joint_state_.size()));
        ROS_ERROR("Number of joints in the active arm FK chain does not match the number of joints in the active arm joint state!");
        return false;
    }

    // Put together joint states for the passive arm.
    joint_index = 1;
    while (true)
    {
        // Check if the parameter for this passive joint exists.
        std::string joint_name;
        std::string param_name = std::string("/passive_arm_joint_name_" + to_string(joint_index));
        if(!n.getParam(param_name, joint_name))
            break;

        // Push back the joint state and name.
        pr2_mechanism_model::JointState* jointState = robot_->getJointState(joint_name);
        passive_arm_joint_state_.push_back(jointState);
        if (jointState == NULL)
            ROS_INFO_STREAM("jointState: " + joint_name + " is null");
        passive_arm_joint_names_.push_back(joint_name);

        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the active arm joint state.
    if (passive_arm_fk_chain_.getNrOfJoints() != sizeof(passive_arm_joint_state_))
    {
        ROS_INFO_STREAM("num_fk_chain: " + to_string(passive_arm_fk_chain_.getNrOfJoints()));
        ROS_INFO_STREAM("num_joint_state: " + to_string(passive_arm_joint_state_.size()));
        ROS_ERROR("Number of joints in the passive arm FK chain does not match the number of joints in the passive arm joint state!");
        return false;
    }

    // Allocate torques array.
    active_arm_torques_.resize(active_arm_fk_chain_.getNrOfJoints());
    passive_arm_torques_.resize(passive_arm_fk_chain_.getNrOfJoints());

    // Subscribe to the joint states
    // ros::Subscriber sub = n.subscribe("joint_states", 1000, chatterCallback);
    


    // Initialize ROS subscribers/publishers, sensors, and position controllers.
    // Note that this must be done after the FK solvers are created, because the sensors
    // will ask to use these FK solvers!
    joint_states_subcriber_ = n.subscribe("/joint_states", 1, &GPSBAXTERPlugin::joint_states_subscriber_callback, this);
    initialize(n);

    // Tell the PR2 controller manager that we initialized everything successfully.
    return true;
}

// This is called by the controller manager before starting the controller.
void GPSBAXTERPlugin::starting()
{
    // Get current time.
    last_update_time_ = robot_->getTime();
    controller_counter_ = 0;

    // Reset all the sensors. This is important for sensors that try to keep
    // track of the previous state somehow.
    //for (int sensor = 0; sensor < TotalSensorTypes; sensor++)
    for (int sensor = 0; sensor < 1; sensor++)
    {
        sensors_[sensor]->reset(this,last_update_time_);
    }

    // Reset position controllers.
    passive_arm_controller_->reset(last_update_time_);
    active_arm_controller_->reset(last_update_time_);

    // Reset trial controller, if any.
    if (trial_controller_ != NULL) trial_controller_->reset(last_update_time_);

}

// This is called by the controller manager before stopping the controller.
void GPSBAXTERPlugin::stopping()
{
    // Nothing to do here.
}

// This is the main update function called by the realtime thread when the controller is running.
void GPSBAXTERPlugin::update()
{
    // Get current time.
    last_update_time_ = robot_->getTime();

    // Check if this is a controller step based on the current controller frequency.
    controller_counter_++;
    if (controller_counter_ >= controller_step_length_) controller_counter_ = 0;
    bool is_controller_step = (controller_counter_ == 0);

    // Update the sensors and fill in the current step sample.
    update_sensors(last_update_time_,is_controller_step);

    // Update the controllers.
    update_controllers(last_update_time_,is_controller_step);

    // Store the torques.
    for (unsigned i = 0; i < active_arm_joint_state_.size(); i++)
    {
        active_arm_joint_state_[i]->commanded_effort_ = active_arm_torques_[i];
        float gps_position = active_arm_joint_state_[i]->position_;
        float topic_position = left_arm_joint_positions[i];
        ROS_INFO("Index: %u position from gps: %f position from topic: %f difference between positions: %f",
            i, gps_position, topic_position, gps_position-topic_position);
        float gps_torque = active_arm_joint_state_[i]->measured_effort_;
        float topic_torque = left_arm_joint_torques[i];
        ROS_INFO("Index: %u torque from gps: %f torque from topic: %f difference between torques: %f",
            i, gps_torque, topic_torque, gps_torque-topic_torque);
    }

    for (unsigned i = 0; i < passive_arm_joint_state_.size(); i++)
        passive_arm_joint_state_[i]->commanded_effort_ = passive_arm_torques_[i];
}

// Get current time.
ros::Time GPSBAXTERPlugin::get_current_time() const
{
    return last_update_time_;
}

// Get current encoder readings (robot-dependent).
void GPSBAXTERPlugin::get_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const
{
    if (arm == gps::AUXILIARY_ARM)
    {
        if (angles.rows() != sizeof(right_arm_joint_positions))
            angles.resize(sizeof(right_arm_joint_positions));
        for (unsigned i = 0; i < angles.size(); i++)
            angles(i) = right_arm_joint_positions[i];
    }
    else if (arm == gps::TRIAL_ARM)
    {
        if (angles.rows() != sizeof(left_arm_joint_positions))
            angles.resize(sizeof(left_arm_joint_positions));
        for (unsigned i = 0; i < angles.size(); i++)
            angles(i) = left_arm_joint_positions[i];
    }
    else
    {
        ROS_ERROR("Unknown ArmType %i requested for joint encoder readings!",arm);
    }
}

void GPSBAXTERPlugin::joint_states_subscriber_callback(const sensor_msgs::JointStateConstPtr& joint_states)
{
    std::ostringstream positions;
    // map the indices correctly... this was painstaking to find. 
    unsigned index_map [7] = {2,0,1,4,3,5,6};
    for (unsigned i = 0; i < 7; i++)
    {
        left_arm_joint_positions[index_map[i]] = joint_states->position[i+31];
        left_arm_joint_torques[index_map[i]] = joint_states->effort[i+31];
        right_arm_joint_positions[index_map[i]] = joint_states->position[i+17];
        right_arm_joint_torques[index_map[i]] = joint_states->effort[i+17];
    }

    // std::vector<int> v(x, x + sizeof x / sizeof x[0]);

  // ROS_INFO("I am getting called, and my position type is: %6.4lf", left_arm_joint_states[1]);
}

}

// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(gps_agent_pkg, GPSBAXTERPlugin,
						gps_control::GPSBAXTERPlugin,
						pr2_controller_interface::Controller)

