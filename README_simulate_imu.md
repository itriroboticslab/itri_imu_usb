# 🛰️ Simulated IMU Node & itri_imu_usb_sss

This package contains two key components for IMU data handling:

- **simulate_imu_from_tf**: Generates simulated IMU data based on TF transform difference, useful for testing state estimation or replacing physical IMUs in simulation.
- **itri_imu_usb_sss**: Interface node for reading real IMU data from USB-connected IMUs (e.g., ITRI IMU module).

---

## 📦 1. simulate_imu_from_tf

### ✅ Overview
This node estimates angular velocity and linear acceleration based on the relative pose change of a robot frame (e.g., `base_link`) using TF. Optionally adds Gaussian noise to mimic real IMU behavior.

### 🚀 How to Run
```bash
roslaunch itri_imu_usb simulate_imu.launch
```

### 🔧 Parameters
| Name            | Type   | Default    | Description |
|----------------|--------|------------|-------------|
| `frame_id`     | string | base_link  | The moving frame whose motion will be converted to IMU output. |
| `ref_frame`    | string | odom       | Reference frame (usually world or odom). |
| `noise_std_acc`| double | 1.0        | Standard deviation for linear acceleration noise. |
| `noise_std_gyro`| double| 0.1       | Standard deviation for angular velocity noise. |

### 🔄 Output Topic
- `/simulated_imu` (`sensor_msgs/Imu`)

### 🧠 Use Cases
- Test IMU-based state estimation without real sensors
- Inject controlled noise into state estimators
- Replay or simulate platform dynamics with TF only

---

## 📦 2. itri_imu_usb_sss

### ✅ Overview
This node reads IMU data from a USB-connected ITRI IMU sensor, parses it, and republishes as a standard `sensor_msgs/Imu` topic.

### 🚀 How to Run
```bash
roslaunch itri_imu_usb itri_imu_usb_sss.launch
```

### 🔧 Parameters
| Name               | Type   | Description |
|--------------------|--------|-------------|
| `port`             | string | Serial port (e.g., `/dev/ttyUSB0`) |
| `frame_id`         | string | TF frame ID (e.g., `imu_link`) |
| `baudrate`         | int    | Baud rate (default 115200 or sensor-dependent) |
| `acc_scale`        | double | Optional manual scaling for acceleration |
| `gyro_scale`       | double | Optional manual scaling for angular velocity |

### 🔄 Output Topic
- `/imu/data` (`sensor_msgs/Imu`)

---

## 📁 File Locations
| File | Location |
|------|----------|
| `simulate_imu_from_tf.cpp` | `legged_control/src/` |
| `simulate_imu.launch`      | `legged_control/launch/` |
| `imu_usb.launch`           | `itri_imu_usb_sss/launch/` |

---

## 👣 Suggested Workflow
1. Run `simulate_imu.launch` in simulation or replay mode
2. Run `imu_usb.launch` in real robot deployment
3. Use either output as input to your EKF or state estimator


