/**
 * @file g1_arm_action_node.cpp
 * @brief ROS2 node for Unitree G1 robot arm action control
 * 
 * This node provides ROS2 service interfaces for controlling the G1 robot's arm actions.
 * It wraps the unitree_sdk2 G1ArmActionClient to provide:
 *   - Execute preset arm actions
 *   - Execute custom/teach arm actions
 *   - Stop current custom action
 *   - Get list of available actions
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/g1/arm/g1_arm_action_client.hpp"
#include "unitree/robot/g1/arm/g1_arm_action_error.hpp"

#include "g1_arm_action_ros2/srv/execute_arm_action.hpp"
#include "g1_arm_action_ros2/srv/get_arm_action_list.hpp"

using namespace unitree::robot;
using namespace unitree::robot::g1;

// Command type constants
constexpr uint8_t CMD_EXECUTE_PRESET = 0;
constexpr uint8_t CMD_EXECUTE_CUSTOM = 1;
constexpr uint8_t CMD_STOP_ACTION = 2;

class G1ArmActionNode : public rclcpp::Node
{
public:
    G1ArmActionNode() : Node("g1_arm_action_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("network_interface", "");
        this->declare_parameter<double>("timeout", 10.0);

        // Get parameters
        std::string network_interface = this->get_parameter("network_interface").as_string();
        double timeout = this->get_parameter("timeout").as_double();

        RCLCPP_INFO(this->get_logger(), "Initializing G1 Arm Action Node...");
        RCLCPP_INFO(this->get_logger(), "Network interface: %s", 
                    network_interface.empty() ? "(default)" : network_interface.c_str());
        RCLCPP_INFO(this->get_logger(), "Timeout: %.1f seconds", timeout);

        // Initialize DDS channel
        try {
            ChannelFactory::Instance()->Init(0, network_interface);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize DDS channel: %s", e.what());
            throw;
        }

        // Initialize arm action client
        arm_action_client_ = std::make_shared<G1ArmActionClient>();
        arm_action_client_->Init();
        arm_action_client_->SetTimeout(static_cast<float>(timeout));

        // Create services
        execute_action_service_ = this->create_service<g1_arm_action_ros2::srv::ExecuteArmAction>(
            "~/execute_arm_action",
            std::bind(&G1ArmActionNode::executeArmActionCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));

        get_action_list_service_ = this->create_service<g1_arm_action_ros2::srv::GetArmActionList>(
            "~/get_arm_action_list",
            std::bind(&G1ArmActionNode::getArmActionListCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "G1 Arm Action Node initialized successfully!");
        RCLCPP_INFO(this->get_logger(), "Services available:");
        RCLCPP_INFO(this->get_logger(), "  - ~/execute_arm_action");
        RCLCPP_INFO(this->get_logger(), "  - ~/get_arm_action_list");
    }

    ~G1ArmActionNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down G1 Arm Action Node...");
    }

private:
    /**
     * @brief Callback for execute_arm_action service
     */
    void executeArmActionCallback(
        const std::shared_ptr<g1_arm_action_ros2::srv::ExecuteArmAction::Request> request,
        std::shared_ptr<g1_arm_action_ros2::srv::ExecuteArmAction::Response> response)
    {
        int32_t ret = 0;

        switch (request->command_type)
        {
        case CMD_EXECUTE_PRESET:
            RCLCPP_INFO(this->get_logger(), "Executing preset action with ID: %d", request->action_id);
            ret = arm_action_client_->ExecuteAction(request->action_id);
            break;

        case CMD_EXECUTE_CUSTOM:
            RCLCPP_INFO(this->get_logger(), "Executing custom action: %s", request->action_name.c_str());
            ret = arm_action_client_->ExecuteAction(request->action_name);
            break;

        case CMD_STOP_ACTION:
            RCLCPP_INFO(this->get_logger(), "Stopping current custom action");
            ret = arm_action_client_->StopCustomAction();
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Invalid command type: %d", request->command_type);
            response->success = false;
            response->error_code = -1;
            response->message = "Invalid command type. Use 0 (preset), 1 (custom), or 2 (stop).";
            return;
        }

        // Process result
        response->error_code = ret;
        
        if (ret == 0) {
            response->success = true;
            response->message = "Action executed successfully.";
            RCLCPP_INFO(this->get_logger(), "Action executed successfully");
        } else {
            response->success = false;
            response->message = getErrorMessage(ret);
            RCLCPP_WARN(this->get_logger(), "Action failed with error code %d: %s", 
                        ret, response->message.c_str());
        }
    }

    /**
     * @brief Callback for get_arm_action_list service
     */
    void getArmActionListCallback(
        const std::shared_ptr<g1_arm_action_ros2::srv::GetArmActionList::Request> /*request*/,
        std::shared_ptr<g1_arm_action_ros2::srv::GetArmActionList::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Getting arm action list...");
        
        std::string action_list_data;
        int32_t ret = arm_action_client_->GetActionList(action_list_data);

        response->error_code = ret;
        
        if (ret == 0) {
            response->success = true;
            response->action_list = action_list_data;
            RCLCPP_INFO(this->get_logger(), "Successfully retrieved action list");
        } else {
            response->success = false;
            response->action_list = "";
            RCLCPP_WARN(this->get_logger(), "Failed to get action list, error code: %d", ret);
        }
    }

    /**
     * @brief Convert error code to human-readable message
     */
    std::string getErrorMessage(int32_t error_code)
    {
        switch (error_code)
        {
        case UT_ROBOT_ARM_ACTION_ERR_ARMSDK:
            return std::string(UT_ROBOT_ARM_ACTION_ERR_ARMSDK_DESC);
        
        case UT_ROBOT_ARM_ACTION_ERR_HOLDING:
            return std::string(UT_ROBOT_ARM_ACTION_ERR_HOLDING_DESC);
        
        case UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID:
            return std::string(UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID_DESC);
        
        case UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID:
            return "Invalid FSM ID. Actions are only supported in fsm id {500, 501, 801}. "
                   "In state 801, actions are only supported in fsm mode {0, 3}. "
                   "Subscribe to rt/sportmodestate to check the fsm id.";
        
        default:
            return "Unknown error occurred. Error code: " + std::to_string(error_code);
        }
    }

    // Member variables
    std::shared_ptr<G1ArmActionClient> arm_action_client_;
    rclcpp::Service<g1_arm_action_ros2::srv::ExecuteArmAction>::SharedPtr execute_action_service_;
    rclcpp::Service<g1_arm_action_ros2::srv::GetArmActionList>::SharedPtr get_action_list_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<G1ArmActionNode>();
        RCLCPP_INFO(node->get_logger(), "G1 Arm Action Node is running...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("g1_arm_action_node"), 
                     "Exception during node execution: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
