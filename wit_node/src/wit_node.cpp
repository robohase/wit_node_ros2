#include "wit_node/wit_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wit
{

    WitNode::WitNode(const rclcpp::NodeOptions &options)
        : WitNode::WitNode("", options)
    {
    }

    WitNode::WitNode(const std::string &node_name, const rclcpp::NodeOptions &options)
        : rclcpp::Node("wit_ros", node_name, options)
    {
        if (this->init() == false)
        {
            rclcpp::shutdown();
        };
        const std::chrono::duration<double> dt(1.0 / this->publish_hz);
        timer_ = this->create_wall_timer(dt, std::bind(&WitNode::timer_callback, this));
    }
    WitNode::~WitNode()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for WitDriver finish.");
        wd_->shutdown();
    }
    void WitNode::initializeParameter()
    {
        this->wit_param_.port_ = this->declare_parameter<std::string>("port", this->wit_param_.port_);
        this->wit_param_.baud_rate_ = this->declare_parameter<int>("baud_rate", this->wit_param_.baud_rate_);
        this->frame_id = this->declare_parameter<std::string>("frame_id", this->frame_id);
        this->publish_hz = this->declare_parameter<double>("publish_hz", this->publish_hz);

        RCLCPP_INFO(this->get_logger(),
                    "port: %s", this->wit_param_.port_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "baud_rate: %d", this->wit_param_.baud_rate_);
        RCLCPP_INFO(this->get_logger(),
                    "frame_id: %s", this->frame_id.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "publish_hz: %2.1f", this->publish_hz);
    }
    bool WitNode::init()
    {
        RCLCPP_INFO(this->get_logger(), "initializing...");
        this->initializeParameter();
        this->createPublishers();
        this->createSubscrptions();

        /*********************
         ** Driver Init
         **********************/
        try
        {
            wd_ = std::make_unique<wit::WitDriver>();
            wd_->init(this->wit_param_);
            rclcpp::WallRate rate(100ms);
            rate.sleep();
        }
        catch (const ecl::StandardException &e)
        {
            switch (e.flag())
            {
            case (ecl::OpenError):
            {

                RCLCPP_ERROR(
                    this->get_logger(),
                    "Could not open connection [ %s ].", this->wit_param_.port_.c_str());
                break;
            }
            default:
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Initialization failed. Please restart.");
                RCLCPP_ERROR(
                    this->get_logger(),
                    e.what());
                break;
            }
            }
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Initialized!");
        return true;
    }

    void WitNode::createPublishers()
    {
        this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());
        this->gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps", rclcpp::SensorDataQoS());
        this->raw_data_pub = this->create_publisher<wit_msgs::msg::ImuGpsRaw>("~/raw_data", rclcpp::SensorDataQoS());
        this->related_yaw_pub = this->create_publisher<std_msgs::msg::Float64>("~/related_yaw", rclcpp::SensorDataQoS());
        this->imu_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/imu_pose", rclcpp::SensorDataQoS());
    }

    void WitNode::createSubscrptions()
    {
        this->reset_offset_sub_ = create_subscription<std_msgs::msg::Empty>(
            "~/reset_offset", 10,
            std::bind(&WitNode::subscribeResetOffset,
                      this, std::placeholders::_1));
    }
    void WitNode::subscribeResetOffset(const std_msgs::msg::Empty::ConstSharedPtr)
    {
        wd_->resetYawOffset();
        RCLCPP_INFO(this->get_logger(), "~/resetYawOffset.");
    }

    void WitNode::timer_callback()
    {
        tf2::Quaternion q_tf2;
        std_msgs::msg::Header header;

        std_msgs::msg::Float64::UniquePtr yaw_msg(new std_msgs::msg::Float64);
        sensor_msgs::msg::Imu::UniquePtr imu_msg(new sensor_msgs::msg::Imu);
        geometry_msgs::msg::PoseStamped::UniquePtr imu_pose(new geometry_msgs::msg::PoseStamped);
        sensor_msgs::msg::NavSatFix::UniquePtr gps_msg(new sensor_msgs::msg::NavSatFix);
        wit_msgs::msg::ImuGpsRaw::UniquePtr raw_msg(new wit_msgs::msg::ImuGpsRaw);

        if (this->wd_->isShutdown())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Driver has been shutdown. Stopping update loop.");
            rclcpp::shutdown();
        }
        if (!this->wd_->isConnected())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "serial port is not connetced, please connect.");
            rclcpp::shutdown();
        }

        // header
        header.frame_id = this->frame_id;
        header.stamp = rclcpp::Clock().now();

        // get raw data
        wit::Data::IMUGPS data = this->wd_->getData();

        // yaw
        yaw_msg->data = this->wd_->getRelatedYaw();

        // imu
        imu_msg->header = header;
        // imu gyro
        imu_msg->angular_velocity.x = data.w[0];
        imu_msg->angular_velocity.y = data.w[1];
        imu_msg->angular_velocity.z = data.w[2];
        // imu acc
        imu_msg->linear_acceleration.x = data.a[0];
        imu_msg->linear_acceleration.y = data.a[1];
        imu_msg->linear_acceleration.z = data.a[2];

        // imu orientarion
        q_tf2.setRPY(data.rpy[0], data.rpy[1], data.rpy[2]);
        imu_msg->orientation = tf2::toMsg(q_tf2);
        imu_msg->orientation_covariance =
            {0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001};
        imu_msg->angular_velocity_covariance =
            {0.00001, 0.0, 0.0,
            0.0, 0.00001, 0.0,
            0.0, 0.0, 0.00001};
        imu_msg->linear_acceleration_covariance =
            {0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01};

        // pose
        imu_pose->header = header;
        imu_pose->pose.orientation = imu_msg->orientation;

        // gps
        gps_msg->header = header;
        gps_msg->altitude = data.altitude;
        gps_msg->latitude = data.latitude;
        gps_msg->longitude = data.longtitude;
        gps_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        // raw
        raw_msg->header = header;
        for (size_t i = 0; i < 3; ++i)
        {
            raw_msg->acc.push_back(data.a[i]);
            raw_msg->gyro.push_back(data.w[i]);
            raw_msg->rpy.push_back(data.rpy[i]);
            raw_msg->mag.push_back(data.mag[i]);
            raw_msg->dop.push_back(data.gpsa[i]);
        }
        for (size_t i = 0; i < 4; ++i)
        {
            raw_msg->ps.push_back(data.d[i]);
            raw_msg->quarternion.push_back(data.q[i]);
        }
        raw_msg->sn = data.satelites;
        raw_msg->gpsh = data.gpsh;
        raw_msg->gpsy = data.gpsy;
        raw_msg->gpsv = data.gpsv;
        raw_msg->ap = data.pressure;
        raw_msg->longtitude = data.longtitude;
        raw_msg->altitude = data.altitude;
        raw_msg->latitude = data.latitude;
        raw_msg->time = data.timestamp;
        raw_msg->temperature = data.temperature;

        // publish
        this->imu_pub->publish(std::move(imu_msg));
        this->gps_pub->publish(std::move(gps_msg));
        this->raw_data_pub->publish(std::move(raw_msg));
        this->related_yaw_pub->publish(std::move(yaw_msg));
        this->imu_pose_pub->publish(std::move(imu_pose));
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(wit::WitNode)
