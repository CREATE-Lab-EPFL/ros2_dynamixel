#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <memory>
#include <vector>
#include <mutex>

#define ADDR_RETURN_DELAY_TIME 9
#define ADDR_OPERATING_MODE 11
#define ADDR_STATUS_RETURN_LEVEL 68
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_VELOCITY 128

#define CURRENT_CONTROL_MODE 0
#define EXTENDED_POSITION_CONTROL_MODE 4
#define PROTOCOL_VERSION 2.0
#define DEFAULT_KT 0.00115  // Nm/mA
#define PI 3.14159265359

class DynamixelNode : public rclcpp::Node
{
public:
    DynamixelNode() : Node("dynamixel_node")
    {
        this->declare_parameter("motor_ids", std::vector<int64_t>{1, 2});
        this->declare_parameter("position_motor_ids", std::vector<int64_t>{});
        this->declare_parameter("control_mode", "torque");
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 1000000);
        this->declare_parameter("velocity_filter_alpha", 0.1);
        this->declare_parameter("use_kt", true);
        this->declare_parameter("kt", DEFAULT_KT);

        std::vector<int64_t> motor_ids_int     = this->get_parameter("motor_ids").as_integer_array();
        std::vector<int64_t> pos_ids_int        = this->get_parameter("position_motor_ids").as_integer_array();
        std::string          mode               = this->get_parameter("control_mode").as_string();
        std::string          port               = this->get_parameter("port").as_string();
        int                  baudrate           = this->get_parameter("baudrate").as_int();
        velocity_filter_alpha_ = this->get_parameter("velocity_filter_alpha").as_double();
        use_kt_ = this->get_parameter("use_kt").as_bool();
        kt_ = this->get_parameter("kt").as_double();

        // When position_motor_ids is provided, motor_ids are always torque-controlled
        // and position_motor_ids are position-controlled (mixed mode).
        // Without position_motor_ids, control_mode applies to all motor_ids (backwards compat).
        bool mixed_mode = !pos_ids_int.empty();

        if (mixed_mode) {
            // motor_ids → torque (current) control
            torque_mode_ = true;
            for (auto id : motor_ids_int)
                motor_ids_.push_back(static_cast<uint8_t>(id));
            for (auto id : pos_ids_int)
                pos_motor_ids_.push_back(static_cast<uint8_t>(id));
        } else {
            // Backwards-compatible single-mode: all motor_ids use control_mode
            torque_mode_ = (mode != "position");
            for (auto id : motor_ids_int)
                motor_ids_.push_back(static_cast<uint8_t>(id));
        }

        goal_command_storage_.resize(motor_ids_.size(), 0.0);
        filtered_velocities_.resize(motor_ids_.size(), 0.0);
        pos_hold_raw_.resize(pos_motor_ids_.size(), 0);

        portHandler_  = dynamixel::PortHandler::getPortHandler(port.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler_->openPort() || !portHandler_->setBaudRate(baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Port initialization failed");
            return;
        }
        portHandler_->setPacketTimeout(10.0);

        // SyncRead covers torque motors only (position motors don't need velocity feedback)
        syncRead_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_,
                                                  ADDR_PRESENT_VELOCITY, 8);
        for (auto id : motor_ids_) {
            syncRead_->addParam(id);
            if (torque_mode_)
                setupTorqueMotor(id);
            else
                setupPositionMotor(id);
        }

        // SyncWrite for torque motors (goal_current, 2 bytes)
        uint16_t w_addr = torque_mode_ ? ADDR_GOAL_CURRENT   : ADDR_GOAL_POSITION;
        uint16_t w_size = torque_mode_ ? 2                    : 4;
        syncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, w_addr, w_size);

        // SyncWrite for position-hold motors (only in mixed mode)
        syncWritePos_ = nullptr;
        if (!pos_motor_ids_.empty()) {
            syncWritePos_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_,
                                                           ADDR_GOAL_POSITION, 4);
        }

        // Read initial positions for torque motors (zero reference)
        initial_positions_raw_.resize(motor_ids_.size());
        initial_positions_.resize(motor_ids_.size());
        syncRead_->txRxPacket();
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            uint32_t pos = syncRead_->getData(motor_ids_[i], ADDR_PRESENT_POSITION, 4);
            initial_positions_raw_[i] = static_cast<int32_t>(pos);
            initial_positions_[i]     = (static_cast<int32_t>(pos) / 4096.0) * 2.0 * PI;
            if (!torque_mode_)
                goal_command_storage_[i] = static_cast<double>(initial_positions_raw_[i]);
        }

        // Read and hold initial positions for position motors (mixed mode)
        for (size_t i = 0; i < pos_motor_ids_.size(); i++) {
            setupPositionMotor(pos_motor_ids_[i]);
            uint32_t pos_raw = 0;
            uint8_t  dxl_err = 0;
            packetHandler_->read4ByteTxRx(portHandler_, pos_motor_ids_[i],
                                           ADDR_PRESENT_POSITION, &pos_raw, &dxl_err);
            pos_hold_raw_[i] = static_cast<int32_t>(pos_raw);
            RCLCPP_INFO(this->get_logger(), "Position motor %d: holding at raw %d",
                        pos_motor_ids_[i], pos_hold_raw_[i]);
        }

        // Subscriptions
        if (torque_mode_) {
            command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/goal_torque", 10,
                std::bind(&DynamixelNode::torqueCallback, this, std::placeholders::_1));
        } else {
            command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/goal_position", 10,
                std::bind(&DynamixelNode::positionCallback, this, std::placeholders::_1));
        }

        position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_positions", 10);
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocities", 10);

        pos_msg_.data.reserve(motor_ids_.size());
        vel_msg_.data.reserve(motor_ids_.size());

        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&DynamixelNode::publishState, this));

        RCLCPP_INFO(this->get_logger(),
            "Dynamixel node: %zu torque motor(s), %zu position motor(s) — mode=%s",
            motor_ids_.size(), pos_motor_ids_.size(), mode.c_str());
    }

    ~DynamixelNode()
    {
        for (auto id : motor_ids_) {
            uint8_t dxl_error = 0;
            packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        }
        // Position motors stay enabled — they hold position until power-off
        delete syncRead_;
        delete syncWrite_;
        if (syncWritePos_) delete syncWritePos_;
        portHandler_->closePort();
    }

private:
    void setupTorqueMotor(uint8_t id)
    {
        uint8_t dxl_error = 0;
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_RETURN_DELAY_TIME, 0, &dxl_error);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_STATUS_RETURN_LEVEL, 1, &dxl_error);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE,
                                       CURRENT_CONTROL_MODE, &dxl_error);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        RCLCPP_INFO(this->get_logger(), "Torque motor %d: current mode", id);
    }

    void setupPositionMotor(uint8_t id)
    {
        uint8_t dxl_error = 0;
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_RETURN_DELAY_TIME, 0, &dxl_error);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_STATUS_RETURN_LEVEL, 1, &dxl_error);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE,
                                       EXTENDED_POSITION_CONTROL_MODE, &dxl_error);
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        RCLCPP_INFO(this->get_logger(), "Position motor %d: extended position mode", id);
    }

    void torqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != motor_ids_.size()) return;
        std::lock_guard<std::mutex> lock(goal_mutex_);
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            if (use_kt_) {
                goal_command_storage_[i] = msg->data[i] / kt_;
            } else {
                goal_command_storage_[i] = msg->data[i];
            }
        }
    }

    void positionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != motor_ids_.size()) return;
        std::lock_guard<std::mutex> lock(goal_mutex_);
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            double cmd_rad    = msg->data[i] * PI / 180.0;
            int32_t offset    = static_cast<int32_t>((cmd_rad / (2.0 * PI)) * 4096.0);
            goal_command_storage_[i] = static_cast<double>(initial_positions_raw_[i] + offset);
        }
    }

    void publishState()
    {
        std::vector<double> current_goals;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            current_goals = goal_command_storage_;
        }

        // --- Write commands to torque / position motors ---
        syncWrite_->clearParam();
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            uint8_t id = motor_ids_[i];
            if (torque_mode_) {
                int16_t current = static_cast<int16_t>(current_goals[i]);
                uint8_t param[2] = {DXL_LOBYTE(current), DXL_HIBYTE(current)};
                syncWrite_->addParam(id, param);
            } else {
                int32_t pos = static_cast<int32_t>(current_goals[i]);
                uint8_t param[4] = {
                    DXL_LOBYTE(DXL_LOWORD(pos)), DXL_HIBYTE(DXL_LOWORD(pos)),
                    DXL_LOBYTE(DXL_HIWORD(pos)), DXL_HIBYTE(DXL_HIWORD(pos))
                };
                syncWrite_->addParam(id, param);
            }
        }
        syncWrite_->txPacket();

        // --- Hold position motors at their initial positions (mixed mode) ---
        if (syncWritePos_ && !pos_motor_ids_.empty()) {
            syncWritePos_->clearParam();
            for (size_t i = 0; i < pos_motor_ids_.size(); i++) {
                int32_t pos = pos_hold_raw_[i];
                uint8_t param[4] = {
                    DXL_LOBYTE(DXL_LOWORD(pos)), DXL_HIBYTE(DXL_LOWORD(pos)),
                    DXL_LOBYTE(DXL_HIWORD(pos)), DXL_HIBYTE(DXL_HIWORD(pos))
                };
                syncWritePos_->addParam(pos_motor_ids_[i], param);
            }
            syncWritePos_->txPacket();
        }

        usleep(100);

        // --- Read state from motor_ids_ ---
        int comm_result = syncRead_->txRxPacket();
        if (comm_result != COMM_SUCCESS) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "SyncRead failed: %d", comm_result);
            return;
        }

        pos_msg_.data.clear();
        vel_msg_.data.clear();
        pos_msg_.data.reserve(motor_ids_.size());
        vel_msg_.data.reserve(motor_ids_.size());

        for (size_t i = 0; i < motor_ids_.size(); i++) {
            if (!syncRead_->isAvailable(motor_ids_[i], ADDR_PRESENT_VELOCITY, 8)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Data not available for motor %d", motor_ids_[i]);
                return;
            }

            uint32_t position     = syncRead_->getData(motor_ids_[i], ADDR_PRESENT_POSITION, 4);
            double   absolute_rad = (static_cast<int32_t>(position) / 4096.0) * 2.0 * PI;
            pos_msg_.data.push_back((absolute_rad - initial_positions_[i]) * 180.0 / PI);

            int32_t velocity = static_cast<int32_t>(
                syncRead_->getData(motor_ids_[i], ADDR_PRESENT_VELOCITY, 4));
            double vel_deg_s = velocity * 0.229 * 6.0;

            filtered_velocities_[i] = velocity_filter_alpha_ * vel_deg_s
                                    + (1.0 - velocity_filter_alpha_) * filtered_velocities_[i];
            vel_msg_.data.push_back(filtered_velocities_[i]);
        }

        position_pub_->publish(pos_msg_);
        velocity_pub_->publish(vel_msg_);
    }

    bool                    torque_mode_;
    std::vector<uint8_t>    motor_ids_;
    std::vector<uint8_t>    pos_motor_ids_;
    std::vector<int32_t>    initial_positions_raw_;
    std::vector<double>     initial_positions_;
    std::vector<int32_t>    pos_hold_raw_;
    std::vector<double>     goal_command_storage_;
    std::mutex              goal_mutex_;
    std::vector<double>     filtered_velocities_;
    double                  velocity_filter_alpha_;
    bool                    use_kt_;
    double                  kt_;

    std_msgs::msg::Float64MultiArray pos_msg_, vel_msg_;

    dynamixel::PortHandler   *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupSyncRead  *syncRead_;
    dynamixel::GroupSyncWrite *syncWrite_;
    dynamixel::GroupSyncWrite *syncWritePos_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelNode>());
    rclcpp::shutdown();
    return 0;
}
