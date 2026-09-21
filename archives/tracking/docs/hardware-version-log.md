# Hardware Version Log

## Odin1 firmware update - 2026-07-10

- Device: Odin1 / hawk
- Serial number: `a04d75221b1b3e53`
- USB VID:PID: `2207:0019`
- Firmware package: `odin1_firmware_update_pack_0.12.0_20260615.zip`
- Firmware payload: `odin1_firmware_update_0.12.0.tar.gz`
- Update tool: `odin1_firmware_update_tool_0.4.8_amd64`
- Update host: local x86_64 workstation
- Update log: `/tmp/odin1_fw_pack/firmware_update_sudo_20260710_151126.log`

Before update:

- SoC app version: `0.11.1`
- Daemon version: `0.6.0`
- Install version: `0.11.3`
- MCU version: `1.6.2`
- Kernel version: `4.10.0`

After update:

- SoC app version: `0.12.0`
- Daemon version: `0.6.1`
- Install version: `0.12.0`
- MCU version: `1.5.2`
- Kernel version: `4.10.0`

Post-update RK3588 test:

- ROS driver: `odin_ros_driver` / `host_sdk_sample`, driver version `0.11.0`
- Required firmware version reported by driver: `0.12.0`
- Driver-read versions on RK3588:
  - Kernel version: `V4.10.0`
  - MCU version: `V1.5.2`
  - SoC version: `V0.12.0`
  - Daemon_proc version: `V0.6.1`
  - SLAM version: `V0.12.0`
- `/raw/odom/odin` verified with `ros2 topic hz`: about `10 Hz`
- `/raw/odom/odin` verified with `ros2 topic echo --once`

Notes:

- Firmware update was started while the device was already attached, but the tool waited for a hotplug event. Replugging Odin on the local workstation caused the tool to proceed.
- A transient `LIBUSB_TRANSFER_ERROR` appeared during device reboot/flashing, followed by a successful reconnect and version readback.
- On RK3588 the device still enumerated through a USB2 path (`bcdUSB 2.10`), but `strict_usb3.0_check` was disabled and Odin odometry published successfully after the firmware update.
