#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/circular_buffer.hpp>

class ImuButterF {
public:
    ImuButterF() : nh_("~") {
        imu_sub_ = nh_.subscribe("/imu/data", 10, &ImuButterF::imuCallback, this);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_but_filter", 10);

        // Initialize Butterworth filter buffers (2nd order needs 3 samples: n, n-1, n-2)
        accel_x_in_.set_capacity(3); accel_y_in_.set_capacity(3); accel_z_in_.set_capacity(3);
        vel_x_in_.set_capacity(3); vel_y_in_.set_capacity(3); vel_z_in_.set_capacity(3);
        accel_x_out_.set_capacity(2); accel_y_out_.set_capacity(2); accel_z_out_.set_capacity(2);
        vel_x_out_.set_capacity(2); vel_y_out_.set_capacity(2); vel_z_out_.set_capacity(2);

        // Fill with zeros initially
        for (int i = 0; i < 3; ++i) {
            accel_x_in_.push_back(0.0); accel_y_in_.push_back(0.0); accel_z_in_.push_back(0.0);
            vel_x_in_.push_back(0.0); vel_y_in_.push_back(0.0); vel_z_in_.push_back(0.0);
        }
        for (int i = 0; i < 2; ++i) {
            accel_x_out_.push_back(0.0); accel_y_out_.push_back(0.0); accel_z_out_.push_back(0.0);
            vel_x_out_.push_back(0.0); vel_y_out_.push_back(0.0); vel_z_out_.push_back(0.0);
        }

        // Butterworth coefficients (2nd order, fs=100Hz, fc=5Hz, precomputed)
        // b_[0] = 0.0048; b_[1] = 0.0096; b_[2] = 0.0048;  // Feedforward
        // a_[0] = 1.0; a_[1] = -1.8227; a_[2] = 0.8372;    // Feedback (a[0] normalized to 1)

        // Butterworth coefficients (2nd order, fs=170Hz, fc=20Hz, precomputed)
        // b_[0] = 0.0838; b_[1] = 0.1676; b_[2] = 0.0838;
        // a_[0] = 1.0; a_[1] = -1.0122; a_[2] = 0.3475;  

        // Butterworth coefficients for fs=170Hz, fc=5Hz
        b_[0] = 0.0053; b_[1] = 0.0105; b_[2] = 0.0053;
        a_[0] = 1.0; a_[1] = -1.8591; a_[2] = 0.8802;  


    }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_;
        ros::Publisher imu_pub_;

        // Butterworth filter buffers (2nd order)
        boost::circular_buffer<double> accel_x_in_, accel_y_in_, accel_z_in_;
        boost::circular_buffer<double> vel_x_in_, vel_y_in_, vel_z_in_;
        boost::circular_buffer<double> accel_x_out_, accel_y_out_, accel_z_out_;
        boost::circular_buffer<double> vel_x_out_, vel_y_out_, vel_z_out_;

        // Filter coefficients (fs=100Hz, fc=5Hz)
        double b_[3];  // Feedforward: b0, b1, b2
        double a_[3];  // Feedback: a0, a1, a2

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {   
        sensor_msgs::Imu imu_msg;
        imu_msg.header = msg->header;
        imu_msg.orientation = msg->orientation;

        // ROS_INFO("Before filter Lin_acc.z = %f", msg->linear_acceleration.z);

        // Update input buffers acceleration
        accel_x_in_.push_back(msg->linear_acceleration.x);
        accel_y_in_.push_back(msg->linear_acceleration.y);
        accel_z_in_.push_back(msg->linear_acceleration.z);
        // Apply filter
        imu_msg.linear_acceleration.x = b_[0] * accel_x_in_[2] + b_[1] * accel_x_in_[1] + b_[2] * accel_x_in_[0]
                                      - a_[1] * accel_x_out_[1] - a_[2] * accel_x_out_[0];
        imu_msg.linear_acceleration.y = b_[0] * accel_y_in_[2] + b_[1] * accel_y_in_[1] + b_[2] * accel_y_in_[0]
                                      - a_[1] * accel_y_out_[1] - a_[2] * accel_y_out_[0];
        imu_msg.linear_acceleration.z = b_[0] * accel_z_in_[2] + b_[1] * accel_z_in_[1] + b_[2] * accel_z_in_[0]
                                      - a_[1] * accel_z_out_[1] - a_[2] * accel_z_out_[0];
        // Update output buffers
        accel_x_out_.push_back(imu_msg.linear_acceleration.x);
        accel_y_out_.push_back(imu_msg.linear_acceleration.y);
        accel_z_out_.push_back(imu_msg.linear_acceleration.z);

        // ROS_INFO("After filter Lin_acc.z = %f", imu_msg.linear_acceleration.z);

        // Update input buffers velocity
        vel_x_in_.push_back(msg->angular_velocity.x);
        vel_y_in_.push_back(msg->angular_velocity.y);
        vel_z_in_.push_back(msg->angular_velocity.z);
        // Apply filter
        imu_msg.angular_velocity.x = b_[0] * vel_x_in_[2] + b_[1] * vel_x_in_[1] + b_[2] * vel_x_in_[0]
                                   - a_[1] * vel_x_out_[1] - a_[2] * vel_x_out_[0];
        imu_msg.angular_velocity.y = b_[0] * vel_y_in_[2] + b_[1] * vel_y_in_[1] + b_[2] * vel_y_in_[0]
                                   - a_[1] * vel_y_out_[1] - a_[2] * vel_y_out_[0];
        imu_msg.angular_velocity.z = b_[0] * vel_z_in_[2] + b_[1] * vel_z_in_[1] + b_[2] * vel_z_in_[0]
                                   - a_[1] * vel_z_out_[1] - a_[2] * vel_z_out_[0];
        // Update output buffers
        vel_x_out_.push_back(imu_msg.angular_velocity.x);
        vel_y_out_.push_back(imu_msg.angular_velocity.y);
        vel_z_out_.push_back(imu_msg.angular_velocity.z);

        // Covariance matrices
        imu_msg.orientation_covariance = msg->orientation_covariance;
        imu_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
        imu_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;

        // publish filtered msg
        imu_pub_.publish(imu_msg);
    }
};

int main(int argc, char** argv) {
    ROS_INFO("Starting node for butterworth filter.");
    ros::init(argc, argv, "itri_imu_butterworth");

    try {
        ImuButterF imu_filter;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to initialize IMU Butterworth Filter: " << e.what());
        return 1;
    }

    return 0;
}