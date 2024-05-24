#include <my_hw_interface/robot_hwi.h>
#include <sstream>

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {
    
// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
//Set the frequency of the control loop.
    loop_hz_= 4;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    sub = nh_.subscribe("/arduino_feedback", 1, &MyRobot::arduinoCallback, this);
    pub = nh_.advertise<my_hw_interface::rad_msg>("/joints_to_arduino", 1);

//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot() {
}

void MyRobot::arduinoCallback(const my_hw_interface::degree_msg &msg) {
    joint_position_[1] = (msg.joint1 * PI) / 180;
    joint_position_[2] = (msg.joint2 * PI) / 180;
    joint_position_[3] = (msg.joint3 * PI) / 180;
    joint_position_[4] = (msg.joint4 * PI) / 180;
    joint_position_[0] = (msg.joint5 * PI) / 180; 
}


void MyRobot::init() {
    
    joint_position_.resize(NUM_JOINTS);
    joint_position_command_.resize(NUM_JOINTS);

// Create joint_state_interface for Joint1
    hardware_interface::JointStateHandle jointStateHandle1("joint_1", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandle1);

// Create position joint interface as Joint1 accepts position command.
    hardware_interface::JointHandle jointPositionHandle1(joint_state_interface_.getHandle("joint_1"), &joint_position_command_[1]);
    position_joint_interface_.registerHandle(jointPositionHandle1);  

// Create joint_state_interface for Joint2
    hardware_interface::JointStateHandle jointStateHandle2("joint_2", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandle2);

// Create position joint interface as Joint2 accepts position command.
    hardware_interface::JointHandle jointPositionHandle2(joint_state_interface_.getHandle("joint_2"), &joint_position_command_[2]);
    position_joint_interface_.registerHandle(jointPositionHandle2);  

// Create joint_state_interface for Joint3
    hardware_interface::JointStateHandle jointStateHandle3("joint_3", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    joint_state_interface_.registerHandle(jointStateHandle3);

// Create position joint interface as Joint3 accepts position command.
    hardware_interface::JointHandle jointPositionHandle3(joint_state_interface_.getHandle("joint_3"), &joint_position_command_[3]);
    position_joint_interface_.registerHandle(jointPositionHandle3);  

// Create joint_state_interface for Joint4
    hardware_interface::JointStateHandle jointStateHandle4("joint_4", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
    joint_state_interface_.registerHandle(jointStateHandle4);

// Create position joint interface as Joint4 accepts position command.
    hardware_interface::JointHandle jointPositionHandle4(joint_state_interface_.getHandle("joint_4"), &joint_position_command_[4]);
    position_joint_interface_.registerHandle(jointPositionHandle4);  

// Create joint_state_interface for Joint5
    hardware_interface::JointStateHandle jointStateHandle5("ee_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle5);

// Create position joint interface as Joint5 accepts position command.
    hardware_interface::JointHandle jointPositionHandle5(joint_state_interface_.getHandle("ee_joint"), &joint_position_command_[0]);
    position_joint_interface_.registerHandle(jointPositionHandle5);  

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);  
}

//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {

    // ROS_INFO("%f %f %f %f %f", joint_position_command_[0], joint_position_command_[1], joint_position_command_[2],
    //     joint_position_command_[3], joint_position_command_[4]);

    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void MyRobot::read() {
    ros::spinOnce();
}

void MyRobot::write(ros::Duration elapsed_time) {
    static my_hw_interface::rad_msg angle_cmd;
    bool isChanged = false;
    static int msgCount = 0;
    
    // msgCount += 1;
    // ROS_INFO("%d \n %f %f %f %f %f", msgCount,
    // joint_position_command_[0], joint_position_command_[1], joint_position_command_[2],
    // joint_position_command_[3], joint_position_command_[4]);

    for(int i = 0; i < NUM_JOINTS; i++){
        if((std::ceil(joint_position_command_prev_[i] * 100) / 100) != (std::ceil(joint_position_command_[i] * 100) / 100)){
            isChanged = true;
            break; // exit loop
        }
    }
 
    if(isChanged) {
        angle_cmd.joint1 = joint_position_command_[1];
        angle_cmd.joint2 = joint_position_command_[2];
        angle_cmd.joint3 = joint_position_command_[3];
        angle_cmd.joint4 = joint_position_command_[4];
        angle_cmd.joint5 = 0.0;
        for(int i = 0; i < NUM_JOINTS; i++) {
           joint_position_command_prev_[i] = joint_position_command_[i];
        }
        pub.publish(angle_cmd);
        msgCount += 1;
        ROS_INFO("Published %d \n %f %f %f %f", msgCount,
        angle_cmd.joint1, angle_cmd.joint2, angle_cmd.joint3, angle_cmd.joint4);
    }
}

int main(int argc, char** argv)
{
    //Initialze the ROS node.
    ros::init(argc, argv, "robot_hwi_node");
    ros::NodeHandle nh;
   
    //Separate Spinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2); 

    // Create the object of the robot hardware_interface class and spin the thread. 
    MyRobot ROBOT(nh);
    spinner.spin();
    return 0;
}