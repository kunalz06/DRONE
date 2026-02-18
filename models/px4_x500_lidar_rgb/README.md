# PX4 X500 LiDAR RGB Quadcopter

This model is a realistic PX4-targeted quadcopter for Gazebo Harmonic / GZ Sim.

## Included Hardware

- PX4 flight-controller body (`px4_fcu_link`) with IMU sensor at 250 Hz
- 4-motor quadcopter propulsion using `gz::sim::systems::MulticopterMotorModel`
- 2D LiDAR (`gpu_lidar`) configured as a wide-FOV scanning sensor
- Front RGB camera + down-facing RGB camera
- 6200mAh battery:
  - Electrical model using `gz::sim::systems::LinearBatteryPlugin`
  - Physical battery mass and dimensions as a dedicated link

## Key Parameters

- Wheelbase class: 500 mm (X500-class geometry)
- Battery capacity: `6.2 Ah` (6200mAh), `16.8 V` full-charge electrical model (4S LiPo)
- LiDAR: `0.1-30.0 m` range, 10 Hz update, `270 deg` horizontal scan
- Cameras: 1280x720 @ 30 Hz, horizontal FOV `1.15192 rad` (~66 deg)

## PX4 Topic Convention

The motor model plugins are set for this command endpoint:

- `/model/px4_x500_lidar_rgb/command/motor_speed`

Actuator numbers are mapped `0..3` in plugin blocks.

## Web References Used

- PX4 Gazebo model list (`x500`, `x500_depth`, `x500_lidar_2d`):
  - https://docs.px4.io/main/en/sim_gazebo_gz/
- Gazebo multicopter motor model examples (local distro file):
  - `/usr/share/gz/gz-sim8/worlds/quadcopter.sdf`
  - `/usr/share/gz/gz-sim8/worlds/multicopter_velocity_control.sdf`
- Gazebo battery plugin examples (local distro file):
  - `/usr/share/gz/gz-sim8/worlds/linear_battery_demo.sdf`
- Typical 4S 6200mAh LiPo physical spec point (dimensions, mass):
  - https://www.racepow.com/products/rc-battery-6200mah-14-8v-4s-130c-lipo-battery-with-deans-t-plug
- PX4 `x500_lidar_2d` reference behavior (Hokuyo UTM-30LX style 2D scan):
  - https://docs.px4.io/main/en/sim_gazebo_gz/
- Raspberry Pi Camera Module 3 spec (12MP Sony IMX708, ~66 deg HFOV):
  - https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf

## Notes

- If your PX4 airframe mixer uses a different motor order/spin direction, adjust `turningDirection` and `actuator_number` in `model.sdf`.
- Sensor noise and aerodynamic coefficients can be tuned further for specific hardware logs.
