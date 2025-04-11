#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <string>
#include <numeric>
#include <cmath>

struct Vec3 {
    double x, y, z;
    Vec3(double x = 0, double y = 0, double z = 0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& other) const { return Vec3(x + other.x, y + other.y, z + other.z); }
    Vec3 operator-(const Vec3& other) const { return Vec3(x - other.x, y - other.y, z - other.z); }
    Vec3 operator/(double val) const { return Vec3(x / val, y / val, z / val); }
    Vec3 operator*(double val) const { return Vec3(x * val, y * val, z * val); }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

class SensorFilter {
public:
    SensorFilter(ros::NodeHandle& nh) {
        ros::NodeHandle pnh("~");
        pnh.param("enable_linear_acceleration", enable_accel_, true);
        pnh.param("enable_angular_velocity", enable_gyro_, true);
        pnh.param("use_moving_average", use_ma_, true);
        pnh.param("use_weighted", use_weighted_, false);
        pnh.param("use_outlier", use_outlier_, false);
        pnh.param("window_size", window_size_, 10);
        pnh.param("outlier_threshold_percent", outlier_threshold_percent_, 0.2);

        sub_ = nh.subscribe("/imu/data_raw", 10, &SensorFilter::imuCallback, this);
        pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data_filtered", 10);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::deque<Vec3> accel_buffer_;
    std::deque<Vec3> gyro_buffer_;
    int window_size_;
    bool enable_accel_, enable_gyro_, use_ma_, use_weighted_, use_outlier_;
    double outlier_threshold_percent_;

    Vec3 weightedAverage(const std::deque<Vec3>& buffer) {
        Vec3 sum;
        double weight_sum = 0;
        for (size_t i = 0; i < buffer.size(); ++i) {
            double weight = i + 1;
            sum = sum + buffer[i] * weight;
            weight_sum += weight;
        }
        return sum / weight_sum;
    }

    Vec3 mean(const std::deque<Vec3>& buffer) {
        Vec3 sum;
        for (const auto& v : buffer) sum = sum + v;
        return sum / buffer.size();
    }

    Vec3 rejectOutliers(const Vec3& val, const std::deque<Vec3>& buffer) {
        Vec3 m = mean(buffer);
        double threshold = m.norm() * outlier_threshold_percent_;
        if ((val - m).norm() > threshold) return m;
        return val;
    }

    void filterVec(Vec3& value, std::deque<Vec3>& buffer) {
        buffer.push_back(value);
        if (buffer.size() > window_size_) buffer.pop_front();

        if (use_outlier_) value = rejectOutliers(value, buffer);
        if (use_ma_) {
            value = use_weighted_ ? weightedAverage(buffer) : mean(buffer);
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        sensor_msgs::Imu filtered = *msg;

        if (enable_accel_) {
            Vec3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            filterVec(acc, accel_buffer_);
            filtered.linear_acceleration.x = acc.x;
            filtered.linear_acceleration.y = acc.y;
            filtered.linear_acceleration.z = acc.z;
        }

        if (enable_gyro_) {
            Vec3 gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            filterVec(gyro, gyro_buffer_);
            filtered.angular_velocity.x = gyro.x;
            filtered.angular_velocity.y = gyro.y;
            filtered.angular_velocity.z = gyro.z;
        }

        pub_.publish(filtered);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_filter_sss");
    ros::NodeHandle nh;
    SensorFilter filter(nh);
    ros::spin();
    return 0;
}
