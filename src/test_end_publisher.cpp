#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class TestEndPublisher : public rclcpp::Node{
public:
    TestEndPublisher(const std::string &node_name, const rclcpp::NodeOptions &options)
            : rclcpp::Node(node_name, options)
    {
        this->declare_parameter<std::string>("base_link", "link_0");
        this->declare_parameter<std::string>("end_effector_link_expect", "link_ee_expect");
        this->declare_parameter<double>("frequency", 100.0);

        parent_frame_ = this->get_parameter("base_link").as_string();
        child_frame_ = this->get_parameter("end_effector_link").as_string();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / this->get_parameter("frequency").as_double())),
                std::bind(&TestEndPublisher::broadcast_expect_ee_cb_, this));

    }

private:
    void broadcast_expect_ee_cb_()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame_;
        t.child_frame_id = child_frame_;
        t.transform.translation.x = 0.5;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.6;
        t.transform.rotation.x = 0.0; //0.383
        t.transform.rotation.y = 1.0; //0.924
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 0.0;

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string parent_frame_;
    std::string child_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestEndPublisher>("test_end_publisher",rclcpp::NodeOptions().use_intra_process_comms(true)));
    rclcpp::shutdown();
    return 0;
}