#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <stdexcept>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

// Helper function to convert big-endian float to host (little-endian) float
float bigEndianToFloat(uint8_t* bytes) {
    union {
        float f;
        uint32_t u;
    } converter;

    // Swap bytes from big-endian (MSB first) to little-endian
    converter.u = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
    return converter.f;
}

class IMUPublisher {
public:
    IMUPublisher() : nh_("~"), serial_port_("/dev/ttyUSB0", 115200) {
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_robotOff", 10);
        
        try {
            if (serial_port_.isOpen()) {
                ROS_WARN("Serial port was already open, closing it first");
                serial_port_.close();
            }
            serial_port_.setBaudrate(115200);
            // serial_port_.setTimeout(serial::Timeout::simpleTimeout(1000));
            // Create a Timeout object explicitly
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Unable to open serial port: " << e.what());
            throw;
        }

        if (serial_port_.isOpen()) {
            ROS_INFO("Serial port initialized successfully");
        }
    }

    ~IMUPublisher() {
        if (serial_port_.isOpen()) {
            serial_port_.close();
            ROS_INFO("Serial port closed on shutdown");
        }
    }

    void run() {
        ROS_INFO_STREAM("Serial port state: " << (serial_port_.isOpen() ? "open" : "closed"));
        std::vector<uint8_t> buffer;
        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "base_imu";

        // Initialize with default valid values// Helper function to convert big-endian float to host (little-endian) float

        imu_msg.orientation.w = 1.0;
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
        imu_msg.orientation_covariance[0] = 0.000076;
        imu_msg.orientation_covariance[4] = 0.000076;
        imu_msg.orientation_covariance[8] = 0.00030;
        imu_msg.angular_velocity_covariance[0] = 0.000000015;
        imu_msg.angular_velocity_covariance[4] = 0.000000015;
        imu_msg.angular_velocity_covariance[8] = 0.000000015;
        imu_msg.linear_acceleration_covariance[0] = 0.00000035;
        imu_msg.linear_acceleration_covariance[4] = 0.00000035;
        imu_msg.linear_acceleration_covariance[8] = 0.00000035;

        const float DEG_TO_RAD = M_PI / 180.0;  // Conversion factor from deg/sec to rad/sec

        while (ros::ok()) {
            try {
                size_t bytes_available = serial_port_.available();
                if (bytes_available > 0) {
                    // std::vector<uint8_t> data = serial_port_.read(bytes_available);
                    std::string raw_data = serial_port_.read(bytes_available);
                    std::vector<uint8_t> data(raw_data.begin(), raw_data.end());
                    buffer.insert(buffer.end(), data.begin(), data.end());
                }

                // Total message length: 2 (header) + 2 (MID+LEN) + 3*4 (headers) + 16 + 12*3 (data) + 1 (checksum) = 69 bytes
                while (buffer.size() >= 69) {
                    // Check message header and MID/LEN
                    if (buffer[0] == 0xFA && buffer[1] == 0xFF && 
                        buffer[2] == 0x36 && buffer[3] == 0x40) {
                        
                        uint8_t* data_ptr = buffer.data();
                        // Skip message header and MID/LEN
                        data_ptr += 4;

                        // Quaternion section
                        if (data_ptr[0] == 0x20 && data_ptr[1] == 0x10 && data_ptr[2] == 0x10) {
                            data_ptr += 3;  // Skip header

                            float w = bigEndianToFloat(data_ptr); data_ptr += 4;
                            float x = bigEndianToFloat(data_ptr); data_ptr += 4;
                            float y = bigEndianToFloat(data_ptr); data_ptr += 4;
                            float z = bigEndianToFloat(data_ptr); data_ptr += 4;

                            imu_msg.orientation.x = x;
                            imu_msg.orientation.y = y;
                            imu_msg.orientation.z = z;
                            imu_msg.orientation.w = w;
                            
                            // Normalize quaternion
                            // tf2::Quaternion q(x, y, z, w);
                            // q.normalize();
                            // imu_msg.orientation.x = q.x();
                            // imu_msg.orientation.y = q.y();
                            // imu_msg.orientation.z = q.z();
                            // imu_msg.orientation.w = q.w();
                        } else {
                            buffer.erase(buffer.begin());
                            continue;
                        }

                        // Acceleration section (ROS: m/s^2, ItriIMU: m/s^2)
                        if (data_ptr[0] == 0x40 && data_ptr[1] == 0x20 && data_ptr[2] == 0x0C) {
                            data_ptr += 3; // Skip header
                                                      
                            imu_msg.linear_acceleration.x = bigEndianToFloat(data_ptr); data_ptr += 4;
                            imu_msg.linear_acceleration.y = bigEndianToFloat(data_ptr); data_ptr += 4;
                            imu_msg.linear_acceleration.z = bigEndianToFloat(data_ptr); 
                            
                            // Capture the 4 bytes for linear_acceleration.z
                            // uint8_t* accel_z_bytes = data_ptr;  
                            // ROS_INFO_STREAM("Linear Accel Z (hex): "
                            //     << std::hex << std::setfill('0') << std::setw(2) << (int)accel_z_bytes[0] << " "
                            //     << std::hex << std::setfill('0') << std::setw(2) << (int)accel_z_bytes[1] << " "
                            //     << std::hex << std::setfill('0') << std::setw(2) << (int)accel_z_bytes[2] << " "
                            //     << std::hex << std::setfill('0') << std::setw(2) << (int)accel_z_bytes[3]);

                            // ROS_INFO_STREAM("Linear Accel Z (dec): " << imu_msg.linear_acceleration.z);

                            data_ptr += 4;
                        }
                        else
                        {
                            buffer.erase(buffer.begin());
                            continue;
                        }

                        // Angular velocity section (ROS: rad/sec, ItriIMU: deg/sec)
                        if (data_ptr[0] == 0x80 && data_ptr[1] == 0x20 && data_ptr[2] == 0x0C) {
                            data_ptr += 3;  // Skip header
                            float roll_deg_sec = bigEndianToFloat(data_ptr);    data_ptr += 4;
                            float pitch_deg_sec = bigEndianToFloat(data_ptr);   data_ptr += 4;
                            float yaw_deg_sec = bigEndianToFloat(data_ptr);     data_ptr += 4;
                            // Convert to rad/sec
                            imu_msg.angular_velocity.x = roll_deg_sec * DEG_TO_RAD;
                            imu_msg.angular_velocity.y = pitch_deg_sec * DEG_TO_RAD;
                            imu_msg.angular_velocity.z = yaw_deg_sec * DEG_TO_RAD;
                        } else {
                            buffer.erase(buffer.begin());
                            continue;
                        }

                        // Magnetometer section (ItriIMU: Gauss. Not used in standard Imu message)
                        if (data_ptr[0] == 0xC0 && data_ptr[1] == 0x20 && data_ptr[2] == 0x0C) {
                            data_ptr += 3;  // Skip header
                            float mag_x = bigEndianToFloat(data_ptr); data_ptr += 4;
                            float mag_y = bigEndianToFloat(data_ptr); data_ptr += 4;
                            float mag_z = bigEndianToFloat(data_ptr); data_ptr += 4;
                        } else {
                            buffer.erase(buffer.begin());
                            continue;
                        }
                        
                        // Publish message (no checksum)
                        imu_msg.header.stamp = ros::Time::now();
                        imu_pub_.publish(imu_msg);
                        buffer.erase(buffer.begin(), buffer.begin() + 69);

                        // Verify checksum (simple sum of all bytes except checksum)
                        // uint8_t calculated_checksum = 0;
                        // for (int i = 0; i < 68; i++) {
                        //     calculated_checksum += buffer[i];
                        // }
                        // if (calculated_checksum == buffer[68]) {
                        //     ROS_WARN("Checksum is correct");
                        //     imu_msg.header.stamp = ros::Time::now();
                        //     imu_pub_.publish(imu_msg);
                        //     buffer.erase(buffer.begin(), buffer.begin() + 69);
                        // } else {
                        //     ROS_WARN("Checksum mismatch");
                        //     buffer.erase(buffer.begin());
                        // }
                    } else {
                        buffer.erase(buffer.begin());
                    }
                }
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("Error reading from serial port: " << e.what());
            }

            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    serial::Serial serial_port_;
};

int main(int argc, char** argv) {
    ROS_INFO("Starting node.");
    ros::init(argc, argv, "itri_imu_usb_pub");

    try {
        IMUPublisher imu_publisher;
        imu_publisher.run();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to initialize IMU publisher: " << e.what());
        return 1;
    }

    return 0;
}