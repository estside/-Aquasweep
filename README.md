# 🤖 AquaSweep – Underwater Rover

**AquaSweep** is a custom-built underwater rover designed to move smoothly below the surface, estimate depth accurately, and process visual data in real time. It’s built using Raspberry Pi, Arduino, and 8 thrusters, and it's meant to be a flexible platform for future underwater cleaning and inspection tasks.

---

## 🌟 What It Can Do

- Move in all directions using 8 thrusters (6-DOF)
- Controlled manually via keyboard using ROS
- Estimates depth using pressure sensor + IMU (fused with Kalman or Complementary filter)
- Sends commands and receives sensor data through a custom ROS–Arduino serial setup
- **Uses OpenCV to detect edges** from the live camera feed (helps with navigation and future visual tasks)
- **Streams camera feed from Raspberry Pi to my Windows laptop** using a simple Flask server

---

## 🧰 Hardware Used

- Raspberry Pi (runs ROS and processes camera data)
- Arduino (controls thrusters and reads sensors)
- 8 Waterproof Thrusters
- Bar30 Pressure Sensor
- MPU6050 (for orientation and acceleration)
- USB/CSI Camera

---

## 🛠️ Setting It Up

```bash
# Clone the repo
git clone https://github.com/estside/aquasweep.git
cd aquasweep/ros_ws

# Build the ROS workspace
catkin_make
source devel/setup.bash

# Launch everything
rosrun your desired package.
