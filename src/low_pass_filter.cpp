#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuLPF {
public:
    ImuLPF() : nh_("~") {
        imu_sub_ = nh_.subscribe("/imu/data", 10, &ImuLPF::imuCallback, this);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_lp_filter", 10);
    }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_;
        ros::Publisher imu_pub_;
        // Set smoothing factor (adjustable via parameter if desired)
        double alpha_ = 0.08;  // Lower value = more smoothing

        double prev_linear_acceleration_x_ = 0.0;
        double prev_linear_acceleration_y_ = 0.0;
        double prev_linear_acceleration_z_ = 0.0;
        double prev_angular_velocity_x_ = 0.0;
        double prev_angular_velocity_y_ = 0.0;
        double prev_angular_velocity_z_ = 0.0;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {   
        sensor_msgs::Imu imu_msg;
        imu_msg.header = msg->header;
        imu_msg.orientation = msg->orientation;

        // ROS_INFO("Before filter Lin_acc.z = %f", msg->linear_acceleration.z);
        // Apply low-pass filter linear_acceleration
        imu_msg.linear_acceleration.x = alpha_ * msg->linear_acceleration.x + (1.0 - alpha_) * prev_linear_acceleration_x_;
        imu_msg.linear_acceleration.y = alpha_ * msg->linear_acceleration.y + (1.0 - alpha_) * prev_linear_acceleration_y_;
        imu_msg.linear_acceleration.z = alpha_ * msg->linear_acceleration.z + (1.0 - alpha_) * prev_linear_acceleration_z_;

        // ROS_INFO("After filter Lin_acc.z = %f", imu_msg.linear_acceleration.z);

        // Update previous values
        prev_linear_acceleration_x_ = imu_msg.linear_acceleration.x;
        prev_linear_acceleration_y_ = imu_msg.linear_acceleration.y;
        prev_linear_acceleration_z_ = imu_msg.linear_acceleration.z;

        // Apply low-pass filter angular_velocity
        imu_msg.angular_velocity.x = alpha_ * msg->angular_velocity.x + (1.0 - alpha_) * prev_angular_velocity_x_;
        imu_msg.angular_velocity.y = alpha_ * msg->angular_velocity.y + (1.0 - alpha_) * prev_angular_velocity_y_;
        imu_msg.angular_velocity.z = alpha_ * msg->angular_velocity.z + (1.0 - alpha_) * prev_angular_velocity_z_;

        // Update previous values
        prev_angular_velocity_x_ = imu_msg.angular_velocity.x;
        prev_angular_velocity_y_ = imu_msg.angular_velocity.y;
        prev_angular_velocity_z_ = imu_msg.angular_velocity.z;

        imu_msg.orientation_covariance = msg->orientation_covariance;
        imu_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
        imu_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;

        // publish filtered msg
        imu_pub_.publish(imu_msg);
    }
};

int main(int argc, char** argv) {
    ROS_INFO("Starting node for low pass filter.");
    ros::init(argc, argv, "itri_imu_lpf");

    try {
        ImuLPF imu_lpf;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to initialize IMU Low Pass Filter: " << e.what());
        return 1;
    }

    return 0;
}