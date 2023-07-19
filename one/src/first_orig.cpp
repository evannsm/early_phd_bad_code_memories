#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>






class Flying : public rclcpp::Node
{
    public:
        Flying():Node("flying")
        {
        RCLCPP_INFO(this->get_logger(), "HI");
        std::cout << "hey" ;
        }

    private:
};





int main(int argc, char *argv[])
{
	std::cout << "Flying?..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Flying>());

	rclcpp::shutdown();
	return 0;
}
