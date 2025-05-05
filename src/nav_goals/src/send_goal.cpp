#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <functional>
#include <memory>
#include <string> 
#include <unordered_map>

struct BookLocation {
    double x;
    double y;
    double qz;
    double qw;
};

class NavGoalSender : public rclcpp::Node {
public:
    NavGoalSender(const std::string& selected_book) :
        Node("nav_goal_sender")
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        while (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        }

        book_locations_ = {
            {"atomic_habits", {-2.778454600354296, -4.813561593419779, -0.7503099122891376, 0.6610862542215401}},
            {"outliers", {-0.12440975209287833, 4.340709593377683, 0.1992253318319828, 0.9799537066394699}},
            {"book_thief", {4.673507318987234, -6.535987759336687, -0.9959991668020258, 0.08936251859515955}}
        };

        //TODO: Make it dynamic with LLM
        //std::string selected_book = "book_thief";

        if (book_locations_.count(selected_book) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Book not found: %s", selected_book.c_str());
            return;
        }
        send_goal(book_locations_[selected_book]);
    }
private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    std::unordered_map<std::string, BookLocation> book_locations_;

    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Invalid Goal");
        } else {
            RCLCPP_INFO(this->get_logger(), "Valid Goal");
        }
    }

    void send_goal(const BookLocation &location) {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->get_clock()->now();
        goal_pose.pose.position.x = location.x;
        goal_pose.pose.position.y = location.y;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = location.qz;
        goal_pose.pose.orientation.w = location.qw;
        RCLCPP_INFO(this->get_logger(), "Position -> x: %.3f, y: %.3f", location.x, location.y);
        // tf2::Quaternion q;
        // q.setRPY(0, 0, location.yaw);
        // goal_pose.pose.orientation = tf2::toMsg(q);
        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose = goal_pose;
        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
        // send_goal_options.goal_response_callback = goal_response_callback;
        send_goal_options.goal_response_callback = std::bind(&NavGoalSender::goal_response_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: ros2 run nav_goals send_goal <book_name>" << std::endl;
        return 1;
    }
    std::string selected_book = argv[1];
    auto node = std::make_shared<NavGoalSender>(selected_book);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
