#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class RelativeOdometryFusion : public rclcpp::Node {
   public:
    RelativeOdometryFusion() : Node("relative_odometry_fusion") {
        this->declare_parameter("high_odom_topic", "high_odom");
        this->declare_parameter("low_odom_topic", "low_odom");
        this->declare_parameter("fused_odom_topic", "odom");
        this->declare_parameter("tf_from", "odom");
        this->declare_parameter("tf_to", "base_link");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("pub_tf", true);
        this->declare_parameter("pub_rate", 100.0);
        this->declare_parameter("use_kalman", false);

        this->declare_parameter("process_noise", 0.01);
        this->declare_parameter("high_freq_noise", 0.1);

        process_noise_    = this->get_parameter("process_noise").as_double();
        high_freq_noise_  = this->get_parameter("high_freq_noise").as_double();
        high_odom_topic_  = this->get_parameter("high_odom_topic").as_string();
        low_odom_topic_   = this->get_parameter("low_odom_topic").as_string();
        fused_odom_topic_ = this->get_parameter("fused_odom_topic").as_string();
        tf_from_          = this->get_parameter("tf_from").as_string();
        tf_to_            = this->get_parameter("tf_to").as_string();
        map_frame_id_     = this->get_parameter("map_frame_id").as_string();
        pub_tf_           = this->get_parameter("pub_tf").as_bool();
        pub_rate_         = this->get_parameter("pub_rate").as_double();
        use_kalman_       = this->get_parameter("use_kalman").as_bool();

        std::string log_msg;
        log_msg = "Parameters: high_odom_topic=" + high_odom_topic_ + ", low_odom_topic=" + low_odom_topic_ +
                  ", fused_odom_topic=" + fused_odom_topic_ + ", tf_from=" + tf_from_ + ", tf_to=" + tf_to_ +
                  ", map_frame_id=" + map_frame_id_ + ", pub_tf=" + std::to_string(pub_tf_) +
                  ", pub_rate=" + std::to_string(pub_rate_) + ", use_kalman=" + std::to_string(use_kalman_);
        RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());

        high_freq_initialized_ = false;
        low_freq_initialized_  = false;

        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(fused_odom_topic_, 10);

        high_freq_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            high_odom_topic_, 10, std::bind(&RelativeOdometryFusion::highFreqCallback, this, std::placeholders::_1));

        low_freq_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            low_odom_topic_, 10, std::bind(&RelativeOdometryFusion::lowFreqCallback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // auto publish_period = std::chrono::duration<double>(1.0 / pub_rate_);
        // timer_ = this->create_wall_timer(
        //     std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
        //     std::bind(&RelativeOdometryFusion::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Relative odometry fusion node initialized");
    }

   private:
    void highFreqCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!high_freq_initialized_) {
            high_freq_reference_   = *msg;
            high_freq_initialized_ = true;

            if (low_freq_initialized_) {
                last_fused_state_ = current_state_;
            }
            return;
        }

        if (!low_freq_initialized_) {
            return;
        }

        auto delta           = calculateDelta(high_freq_reference_, *msg);
        high_freq_reference_ = *msg;

        if (use_kalman_) {
            updateStateWithDelta(delta);
        } else {
            updateDirectly(*msg, delta);
        }

        if (high_freq_initialized_ && low_freq_initialized_) {
            publishFusedOdometry();
        }
    }

    void lowFreqCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!low_freq_initialized_) {
            initializeStateFromLowFreq(*msg);
            low_freq_initialized_ = true;
            return;
        }

        // 更新低频里程计参考
        low_freq_reference_ = *msg;

        if (use_kalman_) {
            correctStateWithLowFreq(*msg);
        } else {
            resetStateToLowFreq(*msg);
        }
    }

    // 从低频高精度数据初始化状态
    void initializeStateFromLowFreq(const nav_msgs::msg::Odometry &odom) {
        current_state_.pose           = odom.pose;
        current_state_.twist          = odom.twist;
        current_state_.header         = odom.header;
        current_state_.child_frame_id = odom.child_frame_id;

        // 保存低频里程计作为基准
        low_freq_reference_ = odom;

        if (use_kalman_) {
            // 初始化卡尔曼滤波状态
            initializeKalmanFilter();
        }
    }

    // 初始化卡尔曼滤波器
    void initializeKalmanFilter() {
        // 状态向量: [x, y, theta, v_x, v_y, omega]
        x_    = Eigen::VectorXd(6);
        x_(0) = current_state_.pose.pose.position.x;
        x_(1) = current_state_.pose.pose.position.y;

        // 从四元数获取偏航角
        double qx = current_state_.pose.pose.orientation.x;
        double qy = current_state_.pose.pose.orientation.y;
        double qz = current_state_.pose.pose.orientation.z;
        double qw = current_state_.pose.pose.orientation.w;
        x_(2)     = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        x_(3) = current_state_.twist.twist.linear.x;
        x_(4) = current_state_.twist.twist.linear.y;
        x_(5) = current_state_.twist.twist.angular.z;

        // 初始化协方差矩阵
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;

        // 过程噪声
        Q_ = Eigen::MatrixXd::Identity(6, 6) * process_noise_;

        // 高频测量噪声
        R_high_ = Eigen::MatrixXd::Identity(6, 6) * high_freq_noise_;
    }

    // 计算两个里程计读数之间的增量
    nav_msgs::msg::Odometry calculateDelta(const nav_msgs::msg::Odometry &ref, const nav_msgs::msg::Odometry &current) {
        nav_msgs::msg::Odometry delta;

        // 计算位置增量（简单差值）
        delta.pose.pose.position.x = current.pose.pose.position.x - ref.pose.pose.position.x;
        delta.pose.pose.position.y = current.pose.pose.position.y - ref.pose.pose.position.y;

        // 计算角度增量（处理角度环绕）
        double ref_yaw   = extractYaw(ref.pose.pose.orientation);
        double curr_yaw  = extractYaw(current.pose.pose.orientation);
        double delta_yaw = normalizeAngle(curr_yaw - ref_yaw);

        // 存储角度增量为四元数
        delta.pose.pose.orientation = createQuaternionFromYaw(delta_yaw);

        // 速度增量
        delta.twist.twist.linear.x  = current.twist.twist.linear.x - ref.twist.twist.linear.x;
        delta.twist.twist.linear.y  = current.twist.twist.linear.y - ref.twist.twist.linear.y;
        delta.twist.twist.angular.z = current.twist.twist.angular.z - ref.twist.twist.angular.z;

        return delta;
    }

    // 使用增量更新状态
    void updateStateWithDelta(const nav_msgs::msg::Odometry &delta) {
        // 更新时间步长 dt
        double dt = 0.01;  // 可以通过比较时间戳获取实际dt

        // 预测步骤

        // 状态转移矩阵
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 3)           = dt;  // x += v_x * dt
        F(1, 4)           = dt;  // y += v_y * dt
        F(2, 5)           = dt;  // theta += omega * dt

        // 提取增量到状态向量
        Eigen::VectorXd delta_x(6);
        delta_x(0) = delta.pose.pose.position.x;
        delta_x(1) = delta.pose.pose.position.y;
        delta_x(2) = extractYaw(delta.pose.pose.orientation);
        delta_x(3) = delta.twist.twist.linear.x;
        delta_x(4) = delta.twist.twist.linear.y;
        delta_x(5) = delta.twist.twist.angular.z;

        // 预测状态（使用系统模型和增量）
        x_ = F * x_ + delta_x;

        // 预测协方差
        P_ = F * P_ * F.transpose() + Q_;

        // 更新current_state_
        updateCurrentStateFromX();
    }

    // 使用低频高精度数据校正状态
    void correctStateWithLowFreq(const nav_msgs::msg::Odometry &odom) {
        // 提取低频里程计数据为观测向量
        Eigen::VectorXd z(6);
        z(0) = odom.pose.pose.position.x;
        z(1) = odom.pose.pose.position.y;
        z(2) = extractYaw(odom.pose.pose.orientation);
        z(3) = odom.twist.twist.linear.x;
        z(4) = odom.twist.twist.linear.y;
        z(5) = odom.twist.twist.angular.z;

        // 观测矩阵（直接观测所有状态）
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);

        // 提取低频里程计的协方差信息
        Eigen::MatrixXd R_low = Eigen::MatrixXd::Zero(6, 6);
        for (int i = 0; i < 6; i++) {
            R_low(i, i) = odom.pose.covariance[i * 6 + i];
        }

        // 如果协方差为零，设置一个小的默认值
        for (int i = 0; i < 6; i++) {
            if (R_low(i, i) <= 0.0) {
                R_low(i, i) = 0.001;
            }
        }

        // 计算卡尔曼增益
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_low;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 计算新息（观测与预测的差异）
        Eigen::VectorXd y = z - H * x_;
        // 角度需要特殊处理
        y(2) = normalizeAngle(y(2));

        // 更新状态
        x_ = x_ + K * y;

        // 更新协方差
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_                = (I - K * H) * P_;

        // 更新current_state_
        updateCurrentStateFromX();
    }

    // 从状态向量更新ROS消息
    void updateCurrentStateFromX() {
        current_state_.header.stamp    = this->now();
        current_state_.header.frame_id = "map";  // 使用map坐标系

        current_state_.pose.pose.position.x = x_(0);
        current_state_.pose.pose.position.y = x_(1);

        // 将偏航角转换为四元数
        current_state_.pose.pose.orientation = createQuaternionFromYaw(x_(2));

        current_state_.twist.twist.linear.x  = x_(3);
        current_state_.twist.twist.linear.y  = x_(4);
        current_state_.twist.twist.angular.z = x_(5);

        // 更新协方差
        for (int i = 0; i < 6; i++) {
            current_state_.pose.covariance[i * 6 + i] = P_(i, i);
        }
    }

    void publishFusedOdometry() {
        fused_odom_pub_->publish(current_state_);
        if (pub_tf_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp            = this->now();
            transform.header.frame_id         = tf_from_;
            transform.child_frame_id          = tf_to_;
            transform.transform.translation.x = current_state_.pose.pose.position.x;
            transform.transform.translation.y = current_state_.pose.pose.position.y;
            transform.transform.rotation      = current_state_.pose.pose.orientation;
            tf_broadcaster_->sendTransform(transform);
        }
    }

    // 工具函数：从四元数提取偏航角
    double extractYaw(const geometry_msgs::msg::Quaternion &q) {
        double qx = q.x;
        double qy = q.y;
        double qz = q.z;
        double qw = q.w;
        return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    // 工具函数：从偏航角创建四元数
    geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        q.w = std::cos(yaw * 0.5);
        return q;
    }

    // 工具函数：角度归一化到[-pi, pi]
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // 定时器回调函数，按照设定频率发布融合后的里程计
    void timerCallback() {
        if (high_freq_initialized_ && low_freq_initialized_) {
            publishFusedOdometry();
        }
    }

    // 非卡尔曼滤波模式：直接将低频里程计作为基准
    void resetStateToLowFreq(const nav_msgs::msg::Odometry &odom) {
        current_state_.header.stamp    = this->now();
        current_state_.header.frame_id = map_frame_id_;
        current_state_.child_frame_id  = tf_to_;

        // 从低频里程计复制位姿和速度
        current_state_.pose  = odom.pose;
        current_state_.twist = odom.twist;
    }

    // 非卡尔曼滤波模式：使用高频里程计的增量直接更新状态
    void updateDirectly(const nav_msgs::msg::Odometry &high_freq, const nav_msgs::msg::Odometry &delta) {
        current_state_.header.stamp    = this->now();
        current_state_.header.frame_id = map_frame_id_;
        current_state_.child_frame_id  = tf_to_;

        // 在低频里程计基础上叠加高频里程计的增量
        current_state_.pose.pose.position.x = low_freq_reference_.pose.pose.position.x + delta.pose.pose.position.x;
        current_state_.pose.pose.position.y = low_freq_reference_.pose.pose.position.y + delta.pose.pose.position.y;

        // 计算方向
        double low_yaw                       = extractYaw(low_freq_reference_.pose.pose.orientation);
        double delta_yaw                     = extractYaw(delta.pose.pose.orientation);
        double fused_yaw                     = normalizeAngle(low_yaw + delta_yaw);
        current_state_.pose.pose.orientation = createQuaternionFromYaw(fused_yaw);

        // 速度直接使用高频里程计的当前值
        current_state_.twist.twist = high_freq.twist.twist;
    }

    // 里程计数据
    nav_msgs::msg::Odometry high_freq_reference_;
    nav_msgs::msg::Odometry current_state_;
    nav_msgs::msg::Odometry last_fused_state_;
    nav_msgs::msg::Odometry low_freq_reference_;

    // 卡尔曼滤波状态
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_high_;

    double process_noise_;
    double high_freq_noise_;
    double pub_rate_;

    bool high_freq_initialized_;
    bool low_freq_initialized_;
    bool use_kalman_;

    // ROS发布器和订阅器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    fused_odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr high_freq_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr low_freq_sub_;
    rclcpp::TimerBase::SharedPtr                             timer_;  // 定时器，用于按照设定频率发布里程计
    std::shared_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    std::string high_odom_topic_;
    std::string low_odom_topic_;
    std::string fused_odom_topic_;
    std::string tf_from_;
    std::string tf_to_;
    std::string map_frame_id_;

    bool pub_tf_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativeOdometryFusion>());
    rclcpp::shutdown();
    return 0;
}