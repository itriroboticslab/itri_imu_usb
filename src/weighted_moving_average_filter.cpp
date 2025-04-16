#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <numeric>

// Helper function to compute weighted moving average
double computeWeightedMovingAverage(const std::deque<double>& window) {
    if (window.empty()) return 0.0;
    double sum = 0.0;
    double weight_sum = 0.0;
    size_t n = window.size();
    for (size_t i = 0; i < n; ++i) {
        double weight = static_cast<double>(n - i);  // Linear weights: n, n-1, ..., 1
        sum += weight * window[i];
        weight_sum += weight;
    }
    return sum / weight_sum;
}

class ImuWeightedMAFilter {
public:
ImuWeightedMAFilter() : nh_("~") {
        imu_sub_ = nh_.subscribe("/imu/data", 10, &ImuWeightedMAFilter::imuCallback, this);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_wma_filter", 10);
        
        // Initialize moving average window size
        window_size_ = 10;  // 10 samples for smoothing
    }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_;
        ros::Publisher imu_pub_;

        size_t window_size_;  // Moving average window size

        // Moving average windows for each axis
        std::deque<double> accel_x_window_, accel_y_window_, accel_z_window_;
        std::deque<double> vel_x_window_, vel_y_window_, vel_z_window_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {   
        sensor_msgs::Imu imu_msg;
        imu_msg.header = msg->header;
        imu_msg.orientation = msg->orientation;

        // ROS_INFO("Before filter Lin_acc.z = %f", msg->linear_acceleration.z);

        // Update windows acceleration
        accel_x_window_.push_back(msg->linear_acceleration.x);
        accel_y_window_.push_back(msg->linear_acceleration.y);
        accel_z_window_.push_back(msg->linear_acceleration.z);
        if (accel_x_window_.size() > window_size_) accel_x_window_.pop_front();
        if (accel_y_window_.size() > window_size_) accel_y_window_.pop_front();
        if (accel_z_window_.size() > window_size_) accel_z_window_.pop_front();
        // Compute moving averages
        imu_msg.linear_acceleration.x = computeWeightedMovingAverage(accel_x_window_);
        imu_msg.linear_acceleration.y = computeWeightedMovingAverage(accel_y_window_);
        imu_msg.linear_acceleration.z = computeWeightedMovingAverage(accel_z_window_);

        // ROS_INFO_STREAM("Raw Z: " << raw_z << ", Filtered Z: " << imu_msg.linear_acceleration.z);

        // Update windows velocities
        vel_x_window_.push_back(msg->angular_velocity.x);
        vel_y_window_.push_back(msg->angular_velocity.y);
        vel_z_window_.push_back(msg->angular_velocity.z);
        if (vel_x_window_.size() > window_size_) vel_x_window_.pop_front();
        if (vel_y_window_.size() > window_size_) vel_y_window_.pop_front();
        if (vel_z_window_.size() > window_size_) vel_z_window_.pop_front();
        // Compute moving averages
        imu_msg.angular_velocity.x = computeWeightedMovingAverage(vel_x_window_);
        imu_msg.angular_velocity.y = computeWeightedMovingAverage(vel_y_window_);
        imu_msg.angular_velocity.z = computeWeightedMovingAverage(vel_z_window_);


        // Covariance matrices
        imu_msg.orientation_covariance = msg->orientation_covariance;
        imu_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
        imu_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;

        // publish filtered msg
        imu_pub_.publish(imu_msg);
    }
};

int main(int argc, char** argv) {
    ROS_INFO("Starting node for Weighted moving average filter.");
    ros::init(argc, argv, "itri_imu_weighted_moving_average");

    try {
        ImuWeightedMAFilter imu_filter;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to initialize IMU Weighted Moving Average Filter: " << e.what());
        return 1;
    }

    return 0;
}