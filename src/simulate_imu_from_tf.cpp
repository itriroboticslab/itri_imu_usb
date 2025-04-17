// simulate_imu_from_tf.cpp
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>

class SimulatedImuPublisher {
public:
  SimulatedImuPublisher(ros::NodeHandle& nh)
    : tfBuffer_(), tfListener_(tfBuffer_), distAcc_(0.0, 1.0), distGyro_(0.0, 0.1) {

    nh.param<std::string>("frame_id", frameId_, "base_link");
    nh.param<std::string>("ref_frame", refFrame_, "odom");
    nh.param<double>("noise_std_acc", noiseStdAcc_, 1.0);
    nh.param<double>("noise_std_gyro", noiseStdGyro_, 0.1);

    imuPub_ = nh.advertise<sensor_msgs::Imu>("/simulated_imu", 1);
    prevInitialized_ = false;
  }

  void update() {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tfBuffer_.lookupTransform(refFrame_, frameId_, ros::Time(0), ros::Duration(0.5));
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
      return;
    }

    ros::Time currentTime = tf.header.stamp;
    tf2::Vector3 position(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    tf2::Quaternion orientation(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);

    if (!prevInitialized_) {
      prevTime_ = currentTime;
      prevPos_ = position;
      prevVel_ = tf2::Vector3(0, 0, 0);
      prevQuat_ = orientation;
      prevInitialized_ = true;
      return;
    }

    double dt = (currentTime - prevTime_).toSec();
    if (dt <= 0.0) return;

    tf2::Vector3 vel = (position - prevPos_) / dt;
    tf2::Vector3 acc = (vel - prevVel_) / dt;

    tf2::Quaternion deltaQuat = orientation * prevQuat_.inverse();
    tf2::Matrix3x3 m(deltaQuat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf2::Vector3 gyro(roll / dt, pitch / dt, yaw / dt);

    // Add Gaussian noise
    tf2::Vector3 noisyAcc = acc + tf2::Vector3(distAcc_(gen_), distAcc_(gen_), distAcc_(gen_));
    tf2::Vector3 noisyGyro = gyro + tf2::Vector3(distGyro_(gen_), distGyro_(gen_), distGyro_(gen_));

    sensor_msgs::Imu imuMsg;
    imuMsg.header.stamp = ros::Time::now();
    imuMsg.header.frame_id = frameId_;

    imuMsg.linear_acceleration.x = noisyAcc.x();
    imuMsg.linear_acceleration.y = noisyAcc.y();
    imuMsg.linear_acceleration.z = noisyAcc.z();

    imuMsg.angular_velocity.x = noisyGyro.x();
    imuMsg.angular_velocity.y = noisyGyro.y();
    imuMsg.angular_velocity.z = noisyGyro.z();

    imuMsg.orientation = tf.transform.rotation; // optional

    imuPub_.publish(imuMsg);

    // Update previous state
    prevTime_ = currentTime;
    prevPos_ = position;
    prevVel_ = vel;
    prevQuat_ = orientation;
  }

private:
  ros::Publisher imuPub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  std::string frameId_;
  std::string refFrame_;

  tf2::Vector3 prevPos_, prevVel_;
  tf2::Quaternion prevQuat_;
  ros::Time prevTime_;
  bool prevInitialized_;

  double noiseStdAcc_, noiseStdGyro_;
  std::default_random_engine gen_;
  std::normal_distribution<double> distAcc_, distGyro_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulate_imu_from_tf");
  ros::NodeHandle nh("~");

  SimulatedImuPublisher simImu(nh);
  ros::Rate rate(100);
  while (ros::ok()) {
    simImu.update();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
