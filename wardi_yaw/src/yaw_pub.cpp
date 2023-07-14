#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"


#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
// using std::placeholders::_1;

#include <array>



class Flying : public rclcpp::Node
{
    public:
        float thrust_;
        float roll_;
        float pitch_;
        float yaw_;
        Flying():Node("flying")
        {
            RCLCPP_INFO(this->get_logger(), "HI: PUBRATE");
            std::cout << "hey\n" ;
            

            subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rates_wardi", 10, std::bind(&Flying::topic_callback, this, std::placeholders::_1));





            offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode",10);
            // trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);
            vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command",10);
            rates_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint",10);

            offboard_setpoint_counter_ = 0;

            auto timer_callback = [this]()->void {

                if (offboard_setpoint_counter_ == 10){
                    // Change to offboard mode after 10 setpoints
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1,6);

                    // Arm the vehicle
                    this->arm();
                }

                // offboard control mode needs to be paired with traj_setpoint
                publish_offboard_controlmode();
                // publish_trajectory_setpoint();
                publish_rates_setpoint();

                if (offboard_setpoint_counter_ < 11){
                    offboard_setpoint_counter_++;
                }

            };

            timer_= this->create_wall_timer(100ms, timer_callback);

        }
        
        void arm();
        void disarm();

        void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
        
            thrust_ = msg->data[0];
            roll_ = msg->data[1];
            pitch_ = msg->data[2];
            yaw_ = msg->data[3];
            std::cout << "" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Callback Heard: '{%f, %f, %f, %f}'", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
            // std::cout << "Type of x : " << typeid(thrust_).name() << std::endl;
            arm();

        // // RCLCPP_INFO(this->get_logger(), "I heard: '{%f, %f, %f, %f}'", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);

        //     *thrust_ = msg->data[0];
        //     *roll_ = msg->data[1];
        //     *pitch_ = msg->data[2];
        //     *yaw_= msg->data[3];

        // // std::cout << "Type of x : " << typeid(thrust_).name() << std::endl;


        //     RCLCPP_INFO(this->get_logger(), "Here heard: '%f'", thrust_);
        }

    private:
        // void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
        // {
        
        // // RCLCPP_INFO(this->get_logger(), "I heard: '{%f, %f, %f, %f}'", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);

        // double thrust_ = msg->data[0];
        // double roll_ = msg->data[1];
        // double pitch_ = msg->data[2];
        // double yaw_= msg->data[3];

        // RCLCPP_INFO(this->get_logger(), "Here heard: '%f'", thrust_);
        // }
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;



        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        // rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rates_setpoint_publisher_;
        
        std::atomic<uint64_t> timestamp_;
        // float thrust_;
        // float roll_;
        // float pitch_;
        // float yaw_;

        uint64_t offboard_setpoint_counter_;

        void publish_offboard_controlmode();
        // void publish_trajectory_setpoint();
        void publish_rates_setpoint();
        void publish_vehicle_command(uint16_t command, float param1=0.0, float param2=0.0);
};

void Flying::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm Command Send");


}

void Flying::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm Command Send");
}

void Flying::publish_offboard_controlmode()
{
    OffboardControlMode msg{};
    msg.position= false;
    msg.velocity= false;
    msg.acceleration= false;
    msg.attitude= false;
    msg.body_rate= true;
    msg.timestamp= this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_ -> publish(msg);
}

// void Flying::publish_trajectory_setpoint()
// {
// 	TrajectorySetpoint msg{};
// 	msg.position = {0.0, 0.0, 5};
// 	msg.yaw = -3.14; // [-PI:PI]
//     // msg.velocity = {3, 3, 5};

// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
// 	trajectory_setpoint_publisher_->publish(msg);


// }
void Flying::publish_rates_setpoint()
{
	VehicleRatesSetpoint msg{};
	// msg.roll = *roll_;
	// msg.pitch = pitch_; // [-PI:PI]
    // msg.yaw = yaw_;
    // msg.thrust_body = {0.0, 0.0, -thrust_};


    msg.roll = 0.0;
	// msg.pitch = -roll_; // [-PI:PI]
    msg.pitch = 0.0;
    msg.yaw = yaw_;
    // msg.thrust_body = {0.0, 0.0, -1.0*sqrt(2.0)/2.0};
    // msg.thrust_body = {0, 0, -1.0};
    // msg.thrust_body = {0, 0, thrust_};
    // msg.thrust_body = {0, 0, -.705}; //.71 is good slow ascent & .705 is just below descent ascent
    msg.thrust_body = {0, 0, -thrust_};
    // RCLCPP_INFO(this->get_logger(), "Publishing Rates:");
    //  ={ {0.0, 0.0, 0.0} };
    RCLCPP_INFO(this->get_logger(), "Thrust is: '%f'", msg.thrust_body[2]);
    // RCLCPP_INFO(this->get_logger(), "RollRate is: '%f'", msg.roll);
    // RCLCPP_INFO(this->get_logger(), "PitchRate is: '%f'", msg.pitch);

    RCLCPP_INFO(this->get_logger(), "YawRate is: '%f'", msg.yaw);

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	rates_setpoint_publisher_->publish(msg);


}

void Flying::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}


int main(int argc, char *argv[])
{
	std::cout << "Flying?..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Flying>());

	rclcpp::shutdown();
	return 0;
}
