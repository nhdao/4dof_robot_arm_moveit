#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <my_hw_interface/step_msg.h>
#include <my_hw_interface/step_srv.h>
#include <my_hw_interface/rad_msg.h>
#include <my_hw_interface/degree_msg.h>
#include <my_hw_interface/arr_cmd_msg.h>
#include <my_hw_interface/arr_fb_msg.h>

class MyRobot : public hardware_interface::RobotHW 
{
    public:
        MyRobot(ros::NodeHandle& nh);
        ~MyRobot();
        
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        
        void arduinoCallback(const my_hw_interface::degree_msg &msg) ;
        my_hw_interface::step_srv joint_read;
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::ServiceClient client;

    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        double joint_effort_[5];
        double joint_velocity_[5];
        double joint_position_command_prev_[5];

        std::vector<double> joint_position_;
        std::vector<double> joint_position_command_;

        double PI = 3.14;
        double GEAR_RATIO = 188.6;
        int STEPS_PER_REVOLUTION = 200;
        int NUM_JOINTS = 5;
        int DESIRED_BUFFER_SIZE = 10;
        int bufferHealth = 0;

        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};