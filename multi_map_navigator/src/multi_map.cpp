#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/srv/get_parameters.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "multi_map_action/action/multi_map_goal.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include <functional>
#include <memory>
#include <sqlite3.h>

class MultiMapManager : public rclcpp::Node
{
public:
    // Define the action type
    using MultiMapGoal = multi_map_action::action::MultiMapGoal;
    using MultiMapGoalHandle = rclcpp_action::ServerGoalHandle<MultiMapGoal>;

    MultiMapManager() : Node("multi_map_manager")
    {
        // Declare parameters
        this->declare_parameter<std::string>("map_database", "/home/vidhun/map_ws/src/multi_map_navigator/map/map.db");
        this->declare_parameter<std::string>("map_folder_path", "/home/vidhun/map_ws/src/multi_map_navigator/map/");
        this->declare_parameter<std::string>("multi_map_action_name", "multi_map_goal");
        this->declare_parameter<std::string>("navigation_action_name", "navigate_to_pose");

        db_path_ = this->get_parameter("map_database").as_string();
        map_folder_ = this->get_parameter("map_folder_path").as_string();
        multi_map_action_name_ = this->get_parameter("multi_map_action_name").as_string();
        navigation_action_name_ = this->get_parameter("navigation_action_name").as_string();

        // Initialize the SQLite database
        if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open database");
            return;
        }

        // Creating action server
        action_server_ = rclcpp_action::create_server<MultiMapGoal>(
            this,
            multi_map_action_name_,
            std::bind(&MultiMapManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MultiMapManager::handle_cancel, this, std::placeholders::_1),
            std::bind(&MultiMapManager::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Action Server Initialized");

        // Initialize the action client
        navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, navigation_action_name_);

        // Wait for the action server to be available
        while (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }
        
        RCLCPP_INFO(this->get_logger(), "Action Server Available");

        // Initialize the parameter client
        get_parameters_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/map_server/get_parameters");

        if (!get_parameters_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "Parameter service not available");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Parameter service available");

        // Get the map name from the map server
        get_map_name_from_map_server();

        // Initialize the load map client
        load_map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
        if (!load_map_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "Load map service not available");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Load map service available");
    }

private:
    rclcpp_action::Server<MultiMapGoal>::SharedPtr action_server_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
    std::string map_name_;
    sqlite3 *db_;

    std::string db_path_;
    std::string map_folder_;
    std::string multi_map_action_name_;
    std::string navigation_action_name_;

    // Function to change the map
    void change_map(const std::string &map_name)
    {
        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = map_name;
        auto future = load_map_client_->async_send_request(request);
    }

    // Funtion to get the map name from the map server
    void get_map_name_from_map_server()
    {
        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        request->names.push_back("yaml_filename");
        auto future = get_parameters_client_->async_send_request(request);
        std::thread([this, future = std::move(future)]() mutable
                    {
            try {
                auto response = future.get();
                if (!response->values.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Map name: %s", response->values[0].string_value.c_str());
                    std::string map_name = response->values[0].string_value;
                    map_name = map_name.substr(map_name.find_last_of("/") + 1);
                    map_name = map_name.substr(0, map_name.find_last_of("."));
                    map_name_ = map_name;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Map name parameter not found");
                    map_name_ = "";
                }
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Exception while getting map name: %s", e.what());
                map_name_ = "";
            } })
            .detach();
    }

    // Handle the goal request
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MultiMapGoal::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle the cancel request
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<MultiMapGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    // Handle the accepted goal
    void handle_accepted(const std::shared_ptr<MultiMapGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto goal = goal_handle->get_goal();
        std::string target_map = goal->map_name;

        // Check if the target map is the same as the current map
        if (map_name_ == target_map)
        {
            RCLCPP_INFO(this->get_logger(), "Map already loaded. Sending goal directly.");
            // Send the navigation goal directly
            send_navigation_goal(goal_handle, goal->target_pose);
            return;
        }
        // select the overlap point from the database
        std::string region_name = map_name_ + "&" + target_map;
        sqlite3_stmt *stmt;
        std::string sql = "SELECT x, y FROM overlaps WHERE region = ? ORDER BY RANDOM() LIMIT 1;";
        sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
        sqlite3_bind_text(stmt, 1, region_name.c_str(), -1, SQLITE_STATIC);

        double x = 0.0, y = 0.0;
        bool found = false;

        // Execute the query and check for results
        if (sqlite3_step(stmt) == SQLITE_ROW)
        {
            // If a row is found, extract the x and y coordinates
            x = sqlite3_column_double(stmt, 0);
            y = sqlite3_column_double(stmt, 1);
            found = true;
            RCLCPP_INFO(this->get_logger(), "Overlap point found: x=%.2f, y=%.2f", x, y);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No overlap found for region: %s", region_name.c_str());
            // If no row is found, try the reverse order
            region_name = target_map + "&" + map_name_;
            sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
            sqlite3_bind_text(stmt, 1, region_name.c_str(), -1, SQLITE_STATIC);

            if (sqlite3_step(stmt) == SQLITE_ROW)
            {
                // If a row is found, extract the x and y coordinates
                x = sqlite3_column_double(stmt, 0);
                y = sqlite3_column_double(stmt, 1);
                found = true;
                RCLCPP_INFO(this->get_logger(), "Overlap point (reverse) found: x=%.2f, y=%.2f", x, y);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "No overlap found in reverse either: %s", region_name.c_str());
            }
        }
        // Finalize the statement
        sqlite3_finalize(stmt);

        // If no overlap point is found, abort the goal
        if (!found)
        {
            auto result = std::make_shared<MultiMapGoal::Result>();
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // Create a new goal for the overlap point
        nav2_msgs::action::NavigateToPose::Goal overlap_goal;
        overlap_goal.pose.header.frame_id = "map";
        overlap_goal.pose.header.stamp = this->now();
        // Set the overlap point as the goal
        overlap_goal.pose.pose.position.x = x;
        overlap_goal.pose.pose.position.y = y;
        overlap_goal.pose.pose.orientation.w = 1.0;

        // send the overlap goal
        auto future_goal_handle = navigate_to_pose_client_->async_send_goal(overlap_goal);

        // Create a new thread to handle the overlap navigation
        std::thread([this, goal_handle, future_goal_handle = std::move(future_goal_handle), target_map, goal]() mutable
                    {
            try {
                auto overlap_handle = future_goal_handle.get();
                if (!overlap_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Overlap goal rejected.");
                    auto result = std::make_shared<MultiMapGoal::Result>();
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }

                auto result_future = navigate_to_pose_client_->async_get_result(overlap_handle);
                auto result = result_future.get();
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to reach overlap point.");
                    auto result_msg = std::make_shared<MultiMapGoal::Result>();
                    result_msg->success = false;
                    goal_handle->abort(result_msg);
                    return;
                }

                // Successfully reached the overlap point
                RCLCPP_INFO(this->get_logger(), "Reached overlap point. Switching map...");
                // Change the map
                std::string map_path = map_folder_ + target_map + ".yaml";
                // Load the new map
                change_map(map_path);
                //update the current map name
                map_name_ = target_map;
                // Wait for a while to ensure the map is loaded
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                // move to the target pose
                send_navigation_goal(goal_handle, goal->target_pose);

            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Error in overlap navigation thread: %s", e.what());
                auto result = std::make_shared<MultiMapGoal::Result>();
                result->success = false;
                goal_handle->abort(result);
            } })
            .detach();
    }

    void send_navigation_goal(std::shared_ptr<MultiMapGoalHandle> goal_handle, const geometry_msgs::msg::Pose &pose)
    {
        // Create a new goal for the navigation
        nav2_msgs::action::NavigateToPose::Goal nav_goal;
        nav_goal.pose.header.frame_id = "map";
        nav_goal.pose.header.stamp = this->now();
        nav_goal.pose.pose = pose;

        // Send the navigation goal
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        // Set the feedback callback to handle the feedback from the navigation
        send_goal_options.feedback_callback =
            [this, goal_handle](rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
        {
            auto fb = std::make_shared<MultiMapGoal::Feedback>();
            fb->progress = feedback->distance_remaining;
            goal_handle->publish_feedback(fb);
        };

        // Set the result callback to handle the result of the navigation
        send_goal_options.result_callback =
            [this, goal_handle](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
        {
            auto res = std::make_shared<MultiMapGoal::Result>();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                res->success = true;
                goal_handle->succeed(res);
            }
            else
            {
                res->success = false;
                goal_handle->abort(res);
            }
        };

        // Send the goal to the action server
        auto send_goal_future = navigate_to_pose_client_->async_send_goal(nav_goal, send_goal_options);

        // Create a new thread to handle the navigation goal
        std::thread([this, goal_handle, send_goal_future = std::move(send_goal_future)]() mutable
                    {
            try {
                auto goal_handle_nav = send_goal_future.get();
                if (!goal_handle_nav) {
                    RCLCPP_ERROR(this->get_logger(), "Nav2 rejected final goal.");
                    auto result = std::make_shared<MultiMapGoal::Result>();
                    result->success = false;
                    goal_handle->abort(result);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Nav2 accepted final goal.");
                }
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in sending final goal: %s", e.what());
                auto result = std::make_shared<MultiMapGoal::Result>();
                result->success = false;
                goal_handle->abort(result);
            } })
            .detach();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiMapManager>());
    rclcpp::shutdown();
    return 0;
}
