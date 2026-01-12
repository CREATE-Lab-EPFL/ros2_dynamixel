#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <dynamixel_sdk.h>
#include <memory>
#include <vector>
#include <mutex> // Added for thread-safe storage

#define ADDR_RETURN_DELAY_TIME 9
#define ADDR_OPERATING_MODE 11
#define ADDR_STATUS_RETURN_LEVEL 68
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_VELOCITY 128

#define CURRENT_CONTROL_MODE 0
#define POSITION_CONTROL_MODE 4
#define EXTENDED_POSITION_CONTROL_MODE 4  // Same as position mode, but with extended range
#define PROTOCOL_VERSION 2.0
#define KT 0.000909  // Nm/mA
#define PI 3.14159265359

class DynamixelNode : public rclcpp::Node
{
public:
    DynamixelNode() : Node("dynamixel_node")
    {
        this->declare_parameter("motor_ids", std::vector<int64_t>{1, 2});
        this->declare_parameter("control_mode", "torque");
        this->declare_parameter("baudrate", 1000000);
        
        std::vector<int64_t> motor_ids_int = this->get_parameter("motor_ids").as_integer_array();
        std::string mode = this->get_parameter("control_mode").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        
        // Set control mode
        if (mode == "position") {
            control_mode_ = EXTENDED_POSITION_CONTROL_MODE;  // Use extended position mode
            goal_addr_ = ADDR_GOAL_POSITION;
            goal_size_ = 4;
        } else {
            control_mode_ = CURRENT_CONTROL_MODE;
            goal_addr_ = ADDR_GOAL_CURRENT;
            goal_size_ = 2;
        }
        
        for (auto id : motor_ids_int) {
            motor_ids_.push_back(static_cast<uint8_t>(id));
        }
        
        // Initialize thread-safe storage for commands
        goal_command_storage_.resize(motor_ids_.size(), 0.0);

        portHandler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        
        if (!portHandler_->openPort() || !portHandler_->setBaudRate(baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Port initialization failed");
            return;
        }
        
        // Reduce timeout to 10ms for 1Mbps
        portHandler_->setPacketTimeout(10.0);
        
        syncRead_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, 
                                                  ADDR_PRESENT_VELOCITY, 8);
        for (auto id : motor_ids_) {
            syncRead_->addParam(id);
            setupMotor(id);
        }
        
        syncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_,
                                                    goal_addr_, goal_size_);
        
        // Read initial positions as zero reference
        initial_positions_raw_.resize(motor_ids_.size());
        initial_positions_.resize(motor_ids_.size());
        syncRead_->txRxPacket();
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            uint32_t position = syncRead_->getData(motor_ids_[i], ADDR_PRESENT_POSITION, 4);
            // Store raw position count for extended position mode
            initial_positions_raw_[i] = static_cast<int32_t>(position);
            // This calculation is preserved from original for consistency
            initial_positions_[i] = (static_cast<int32_t>(position) / 4096.0) * 2.0 * PI; 
        }
        RCLCPP_INFO(this->get_logger(), "Initial positions set as zero reference");
        
        if (control_mode_ == POSITION_CONTROL_MODE) {
            command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/goal_position", 10,
                std::bind(&DynamixelNode::positionCallback, this, std::placeholders::_1));
        } else {
            command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/goal_torque", 10,
                std::bind(&DynamixelNode::torqueCallback, this, std::placeholders::_1));
        }
        
        position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_positions", 10);
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_velocities", 10);
        
        // Pre-allocate message objects to avoid allocations in loop
        pos_msg_.data.reserve(motor_ids_.size());
        vel_msg_.data.reserve(motor_ids_.size());
        
        // 1ms timer (1000Hz) - achievable at 1Mbps with 2 motors
        // At 1Mbps: ~60 bytes per cycle = ~0.6ms, leaving headroom for processing
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&DynamixelNode::publishState, this));
        
        RCLCPP_INFO(this->get_logger(), "Dynamixel node started: %zu motors in %s mode", 
                    motor_ids_.size(), mode.c_str());
    }
    
    ~DynamixelNode()
    {
        // Disable torque on motors before exit
        for (auto id : motor_ids_) {
            uint8_t dxl_error = 0;
            packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        }
        delete syncRead_;
        delete syncWrite_;
        portHandler_->closePort();
    }

private:
    void setupMotor(uint8_t id)
    {
        uint8_t dxl_error = 0;
        
        // Critical: Set return delay time to 0 for minimum latency (default is 250 = 500us)
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_RETURN_DELAY_TIME, 0, &dxl_error);
        
        // Critical: Set status return level to 1 (only respond to READ, not WRITE)
        // This eliminates status packets from SyncWrite, reducing communication overhead
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_STATUS_RETURN_LEVEL, 1, &dxl_error);
        
        // Set operating mode
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, 
                                       control_mode_, &dxl_error);
        // Enable torque
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        
        RCLCPP_INFO(this->get_logger(), "Motor %d optimized: return_delay=0, status_level=1", id);
    }
    
    // 1. Synchronization of Communication: Callback Change
    // Only store command data, no serial communication here.
    void torqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != motor_ids_.size()) return;
        
        std::lock_guard<std::mutex> lock(goal_mutex_);
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            // Convert torque (Nm) to raw current unit (mA) and store as double/float
            goal_command_storage_[i] = msg->data[i] / KT; 
        }
    }
    
    // Position callback for extended position control mode
    // Commands are relative to the recorded zero position (initial position)
    void positionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != motor_ids_.size()) return;
        
        std::lock_guard<std::mutex> lock(goal_mutex_);
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            // Command is in degrees, relative to initial position
            double cmd_deg = msg->data[i];
            double cmd_rad = cmd_deg * PI / 180.0;
            
            // Convert relative command to raw position count offset
            int32_t offset_counts = static_cast<int32_t>((cmd_rad / (2.0 * PI)) * 4096.0);
            
            // Add offset to initial position (extended position mode supports full 32-bit range)
            int32_t target_position = initial_positions_raw_[i] + offset_counts;
            
            goal_command_storage_[i] = static_cast<double>(target_position);
        }
    }

    // 1. Synchronization of Communication: Timer Loop Synchronization
    // Now handles Sync Write AND Sync Read back-to-back.
    void publishState()
    {
        std::vector<double> current_goals;
        
        // --- 1. Sync Write (Command) ---
        // Safely retrieve the latest goal commands
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            current_goals = goal_command_storage_;
        }
        
        syncWrite_->clearParam();
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            uint8_t id = motor_ids_[i];
            
            if (control_mode_ == POSITION_CONTROL_MODE) {
                // Position mode: Command is raw 32-bit position count (stored as double)
                int32_t position = static_cast<int32_t>(current_goals[i]);
                
                // Centralize byte-splitting logic here
                uint8_t param[4] = {
                    DXL_LOBYTE(DXL_LOWORD(position)),
                    DXL_HIBYTE(DXL_LOWORD(position)),
                    DXL_LOBYTE(DXL_HIWORD(position)),
                    DXL_HIBYTE(DXL_HIWORD(position))
                };
                syncWrite_->addParam(id, param);
                
            } else { 
                // Current (Torque) mode: Command is raw 16-bit current unit (stored as double)
                int16_t current = static_cast<int16_t>(current_goals[i]);
                
                uint8_t param[2] = {DXL_LOBYTE(current), DXL_HIBYTE(current)};
                syncWrite_->addParam(id, param);
            }
        }
        
        // Execute Sync Write (status return level = 1 means no response, so this is fast)
        syncWrite_->txPacket(); 
        
        // --- 2. Sync Read (State) ---
        // Execute Sync Read immediately after Sync Write
        int comm_result = syncRead_->txRxPacket();
        if (comm_result != COMM_SUCCESS) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "SyncRead failed: %d", comm_result);
            return;
        }
        
        // Reuse message objects to avoid allocations
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
            
            // Position in degrees (re-calculation based on original)
            uint32_t position = syncRead_->getData(motor_ids_[i], ADDR_PRESENT_POSITION, 4);
            double absolute_rad = (static_cast<int32_t>(position) / 4096.0) * 2.0 * PI;
            pos_msg_.data.push_back((absolute_rad - initial_positions_[i]) * 180.0 / PI);
            
            // Velocity in deg/s (re-calculation based on original)
            int32_t velocity = static_cast<int32_t>(syncRead_->getData(motor_ids_[i], ADDR_PRESENT_VELOCITY, 4));
            double vel_rpm = velocity * 0.229;
            vel_msg_.data.push_back(vel_rpm * 6.0);
        }
        
        position_pub_->publish(pos_msg_);
        velocity_pub_->publish(vel_msg_);
    }
    
    uint8_t control_mode_;
    uint16_t goal_addr_;
    uint16_t goal_size_;
    std::vector<uint8_t> motor_ids_;
    std::vector<double> initial_positions_;  // Initial positions in radians (for compatibility)
    std::vector<int32_t> initial_positions_raw_;  // Initial positions in raw counts (for extended mode)
    
    // Thread-safe storage for commands
    std::vector<double> goal_command_storage_; // Stores raw command values (16-bit current or 32-bit position count)
    std::mutex goal_mutex_; // Mutex for goal_command_storage_
    
    // Pre-allocated message objects to avoid allocations in loop
    std_msgs::msg::Float64MultiArray pos_msg_;
    std_msgs::msg::Float64MultiArray vel_msg_;

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupSyncRead *syncRead_;
    dynamixel::GroupSyncWrite *syncWrite_;
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
