# pico_recorder/msg/BleFrame.msg
# One BLE (ESP32-side) frame forwarded from PICO over TCP.
# header.stamp encodes the PICO frame-header ts_ms (sec = ts_ms/1000, nanosec = (ts_ms%1000)*1_000_000).
std_msgs/Header header
uint32 esp32_ts          # ESP32 millisecond clock (from wire payload bytes 0..3)
uint8[] data             # sensor_data (wire payload bytes 4..end), raw
