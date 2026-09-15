# PICO Recorder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build self-contained ROS2 ament_cmake package `pico_recorder` in `/home/eai/project/pico_project/pico_reciver_data/` that bridges PICO 4U TCP stream to ROS2 topics and records to HDF5, without touching `/home/eai/project/exoskeleton_v2/`.

**Architecture:** Two C++ nodes — `pico_bridge_node` (TCP→ROS topics) and `pico_recorder_node` (topics→HDF5 via local simplified DataCollector). BLE uses a custom `BleFrame.msg` to carry `ts_ms` (in `header`) + `esp32_ts` + raw bytes. 30Hz collection grid aligns streams in HDF5. Keyboard `a`/`s`/`q` + `std_srvs/Trigger` services both start/stop recording.

**Tech Stack:** C++17, rclcpp, HDF5 C API, Eigen3, rosidl for custom message, gtest for unit tests, Python for a mock PICO server used in integration smoke tests.

**Spec reference:** `docs/superpowers/specs/2026-04-16-pico-recorder-design.md`

---

## File Structure

```
pico_reciver_data/                             ← ament_cmake package root
├── CMakeLists.txt                             ← new
├── package.xml                                 ← new
├── README.md                                   ← new
├── PICO_Streaming_Guide.md                     ← keep (protocol reference)
├── receiver.py                                 ← keep (protocol reference)
├── msg/
│   └── BleFrame.msg                            ← new
├── include/pico_recorder/
│   ├── pico_frame.hpp                          ← new (frame header parser, header-only)
│   └── data_collector.hpp                      ← new (copied+simplified from inference_cpp)
├── src/
│   ├── data_collector.cpp                      ← new (copied+simplified)
│   ├── pico_bridge_node.cpp                    ← new
│   └── pico_recorder_node.cpp                  ← new
├── launch/
│   └── start_pico_recorder.launch.py           ← new
├── scripts/
│   └── mock_pico_server.py                     ← new (fake PICO server for integration tests)
├── test/
│   ├── test_pico_frame.cpp                     ← new (gtest: frame parser)
│   └── test_data_collector.cpp                 ← new (gtest: collector roundtrip)
├── docs/superpowers/specs/
│   └── 2026-04-16-pico-recorder-design.md      ← exists
└── docs/superpowers/plans/
    └── 2026-04-16-pico-recorder.md             ← this file

/home/eai/project/pico_project/ws/              ← mini colcon workspace (new)
└── src/pico_recorder -> ../../pico_reciver_data ← symlink
```

---

## Task 1: Package scaffolding + mini workspace

**Files:**
- Create: `/home/eai/project/pico_project/pico_reciver_data/package.xml`
- Create: `/home/eai/project/pico_project/pico_reciver_data/CMakeLists.txt`
- Create: `/home/eai/project/pico_project/ws/src/pico_recorder` (symlink)

- [ ] **Step 1: Write `package.xml` declaring the `pico_recorder` package**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pico_recorder</name>
  <version>0.1.0</version>
  <description>Standalone PICO 4U TCP→ROS bridge and HDF5 recorder.</description>
  <maintainer email="eai@local">eai</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_srvs</depend>
  <depend>builtin_interfaces</depend>

  <build_depend>eigen</build_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Write minimal `CMakeLists.txt` (no targets yet — skeleton only)**

```cmake
cmake_minimum_required(VERSION 3.8)
project(pico_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(HDF5 REQUIRED COMPONENTS C)

ament_package()
```

- [ ] **Step 3: Create mini workspace and symlink this package into it**

```bash
mkdir -p /home/eai/project/pico_project/ws/src
ln -s /home/eai/project/pico_project/pico_reciver_data \
      /home/eai/project/pico_project/ws/src/pico_recorder
ls -la /home/eai/project/pico_project/ws/src/
```

Expected: listing shows `pico_recorder -> /home/eai/project/pico_project/pico_reciver_data`.

- [ ] **Step 4: Build empty skeleton to verify package discovery**

```bash
cd /home/eai/project/pico_project/ws
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/iron/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
colcon build --packages-select pico_recorder
```

Expected: `Finished <<< pico_recorder`, no errors. If your ROS2 distro isn't humble/iron/jazzy, source whatever `/opt/ros/*/setup.bash` you use.

- [ ] **Step 5: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git init 2>/dev/null || true
git add package.xml CMakeLists.txt
git commit -m "feat: scaffold pico_recorder ament_cmake package"
```

---

## Task 2: Custom message `BleFrame.msg`

**Files:**
- Create: `msg/BleFrame.msg`
- Modify: `CMakeLists.txt` (add `rosidl_generate_interfaces`)

- [ ] **Step 1: Write the message definition**

```
# pico_recorder/msg/BleFrame.msg
# One BLE (ESP32-side) frame forwarded from PICO over TCP.
# header.stamp encodes the PICO frame-header ts_ms (sec = ts_ms/1000, nanosec = (ts_ms%1000)*1_000_000).
std_msgs/Header header
uint32 esp32_ts          # ESP32 millisecond clock (from wire payload bytes 0..3)
uint8[] data             # sensor_data (wire payload bytes 4..end), raw
```

- [ ] **Step 2: Add rosidl generation to `CMakeLists.txt`** (insert before `ament_package()`)

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BleFrame.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)
```

- [ ] **Step 3: Build and verify the message is generated**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
source install/setup.bash
ros2 interface show pico_recorder/msg/BleFrame
```

Expected: output shows the three fields from the .msg file.

- [ ] **Step 4: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add msg/BleFrame.msg CMakeLists.txt
git commit -m "feat: add BleFrame custom message"
```

---

## Task 3: Frame parser (header-only) + gtest

**Files:**
- Create: `include/pico_recorder/pico_frame.hpp`
- Create: `test/test_pico_frame.cpp`
- Modify: `CMakeLists.txt` (add gtest target)

- [ ] **Step 1: Write the failing test FIRST**

```cpp
// test/test_pico_frame.cpp
#include <gtest/gtest.h>
#include <cstring>
#include "pico_recorder/pico_frame.hpp"

using pico_recorder::FrameHeader;
using pico_recorder::parse_frame_header;
using pico_recorder::HEADER_SIZE;

TEST(PicoFrame, ParsesValidHeader) {
    uint8_t buf[HEADER_SIZE];
    buf[0] = 0xAB;                        // magic
    buf[1] = 0x03;                        // type = pose_left
    int64_t ts_ms = 123456;
    std::memcpy(buf + 2, &ts_ms, 8);
    uint32_t payload_len = 28;
    std::memcpy(buf + 10, &payload_len, 4);

    FrameHeader hdr;
    ASSERT_TRUE(parse_frame_header(buf, hdr));
    EXPECT_EQ(hdr.magic, 0xAB);
    EXPECT_EQ(hdr.type, 0x03);
    EXPECT_EQ(hdr.ts_ms, 123456);
    EXPECT_EQ(hdr.payload_len, 28u);
}

TEST(PicoFrame, RejectsBadMagic) {
    uint8_t buf[HEADER_SIZE] = {0x00};
    FrameHeader hdr;
    EXPECT_FALSE(parse_frame_header(buf, hdr));
}

TEST(PicoFrame, RejectsOversizedPayload) {
    uint8_t buf[HEADER_SIZE];
    buf[0] = 0xAB;
    buf[1] = 0x01;
    int64_t ts = 0;
    std::memcpy(buf + 2, &ts, 8);
    uint32_t big = 20 * 1024 * 1024;      // 20 MB — exceeds 10 MB cap
    std::memcpy(buf + 10, &big, 4);

    FrameHeader hdr;
    EXPECT_FALSE(parse_frame_header(buf, hdr));
}
```

- [ ] **Step 2: Write the parser header to make the tests compile (but still fail)**

```cpp
// include/pico_recorder/pico_frame.hpp
#pragma once
#include <cstdint>
#include <cstring>

namespace pico_recorder {

constexpr int HEADER_SIZE = 1 + 1 + 8 + 4;         // 14 bytes
constexpr uint8_t FRAME_MAGIC = 0xAB;
constexpr uint32_t MAX_PAYLOAD_BYTES = 10 * 1024 * 1024;

// Frame type IDs — mirror receiver.py / PICO_Streaming_Guide.md.
constexpr uint8_t TYPE_CAM_LEFT    = 0x01;
constexpr uint8_t TYPE_CAM_RIGHT   = 0x02;
constexpr uint8_t TYPE_POSE_LEFT   = 0x03;
constexpr uint8_t TYPE_POSE_RIGHT  = 0x04;
constexpr uint8_t TYPE_POSE_HEAD   = 0x05;
constexpr uint8_t TYPE_BLE_LEFT    = 0x10;
constexpr uint8_t TYPE_BLE_RIGHT   = 0x11;

struct FrameHeader {
    uint8_t  magic;
    uint8_t  type;
    int64_t  ts_ms;
    uint32_t payload_len;
};

// Parse 14-byte little-endian header. Returns false on bad magic or oversized payload.
inline bool parse_frame_header(const uint8_t* buf, FrameHeader& out) {
    out.magic = buf[0];
    out.type  = buf[1];
    std::memcpy(&out.ts_ms,       buf + 2, 8);
    std::memcpy(&out.payload_len, buf + 10, 4);
    if (out.magic != FRAME_MAGIC) return false;
    if (out.payload_len > MAX_PAYLOAD_BYTES) return false;
    return true;
}

// Decode 7×float32 little-endian pose payload into pos[3] + quat[4] (xyzw).
// Returns false if payload_len < 28.
inline bool parse_pose_payload(const uint8_t* payload, size_t len,
                                float pos[3], float quat_xyzw[4]) {
    if (len < 28) return false;
    std::memcpy(pos,        payload,      12);
    std::memcpy(quat_xyzw,  payload + 12, 16);
    return true;
}

// Split a BLE payload [4B esp32_ts | sensor_bytes] → esp32_ts + (data_ptr, data_len).
// Returns false if payload is shorter than 4 bytes.
inline bool split_ble_payload(const uint8_t* payload, size_t len,
                               uint32_t& esp32_ts,
                               const uint8_t*& data_ptr, size_t& data_len) {
    if (len < 4) return false;
    std::memcpy(&esp32_ts, payload, 4);
    data_ptr = payload + 4;
    data_len = len - 4;
    return true;
}

}  // namespace pico_recorder
```

- [ ] **Step 3: Add gtest target to `CMakeLists.txt`** (insert before `ament_package()`)

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pico_frame test/test_pico_frame.cpp)
  target_include_directories(test_pico_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()
```

- [ ] **Step 4: Build + run tests**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
colcon test --packages-select pico_recorder --event-handlers console_direct+
```

Expected: 3 tests pass (`ParsesValidHeader`, `RejectsBadMagic`, `RejectsOversizedPayload`).

- [ ] **Step 5: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add include/pico_recorder/pico_frame.hpp test/test_pico_frame.cpp CMakeLists.txt
git commit -m "feat: add pico frame parser with unit tests"
```

---

## Task 4: DataCollector header

**Files:**
- Create: `include/pico_recorder/data_collector.hpp`

- [ ] **Step 1: Write the simplified header (based on `inference_cpp` with PICO extensions)**

```cpp
// include/pico_recorder/data_collector.hpp
#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace pico_recorder {

struct FrameData {
    Eigen::VectorXd data;
    double msg_timestamp;
    double recv_timestamp;
    int64_t ts_ms = 0;               // PICO frame header ts_ms, original value
};

struct ImageFrameData {
    std::vector<uint8_t> bytes;       // JPEG for cameras, raw payload for BLE
    double msg_timestamp;
    double recv_timestamp;
    int64_t ts_ms = 0;
    uint32_t esp32_ts = 0;            // Only used by BLE; cameras leave at 0.
};

struct DatasetConfig {
    int size;                         // doubles per sample (0 for image datasets)
    std::string description;
    bool required;
    double max_age;                   // <=0 → fall back to global default
};

struct DatasetStats {
    double cur_latency = 0.0;
    double avg_latency = 0.0;
    double max_latency = 0.0;
    double cur_delta_time = 0.0;
    double avg_delta_time = 0.0;
    double max_delta_time = 0.0;
    int stale_count = 0;
    int missing_count = 0;
};

struct CollectResult {
    bool success;
    std::string message;
    int frame_index;
    double time_cost;
    std::vector<std::string> stale_datasets;
    std::vector<std::string> missing_datasets;
};

class DataCollector {
public:
    using LogFn = std::function<void(const std::string&)>;

    DataCollector(const std::string& data_dir,
                  double collection_frequency,
                  int max_queue_size,
                  double max_age,
                  const std::string& task_type,
                  LogFn on_error = {}, LogFn on_warn = {}, LogFn on_info = {});

    void register_dataset(const std::string& name, int size,
                          const std::string& description,
                          bool required, double max_age);
    void register_image_dataset(const std::string& name,
                                 const std::string& description,
                                 bool required, double max_age);

    void add_data(const std::string& name, const Eigen::VectorXd& data,
                  double msg_timestamp, int64_t ts_ms);
    void add_image_data(const std::string& name,
                         const std::vector<uint8_t>& bytes,
                         double msg_timestamp,
                         int64_t ts_ms,
                         uint32_t esp32_ts = 0);

    CollectResult collect_frame();
    void update_stats();

    std::pair<bool, std::string> start_collection();
    std::pair<bool, std::string> stop_collection();

    bool is_collecting() const { return is_collecting_; }
    struct Status { bool is_collecting; int num_frames; double duration; };
    Status status() const;

    const std::map<std::string, DatasetStats>& dataset_stats() const { return dataset_stats_; }
    const std::map<std::string, DatasetConfig>& configs() const { return configs_; }
    int total_frames() const { return total_frames_; }
    int dropped_frames() const { return dropped_frames_; }

private:
    void save_hdf5();

    // Config
    std::string data_dir_;
    double collection_frequency_;
    int max_queue_size_;
    double max_age_;
    std::string task_type_;
    LogFn on_error_, on_warn_, on_info_;

    // Datasets
    std::map<std::string, DatasetConfig> configs_;
    std::map<std::string, std::deque<FrameData>> buffers_;
    std::map<std::string, std::mutex> locks_;
    std::map<std::string, DatasetStats> dataset_stats_;
    std::vector<std::string> dataset_order_;

    std::set<std::string> image_datasets_;
    std::map<std::string, std::deque<ImageFrameData>> image_buffers_;
    std::vector<std::string> image_dataset_order_;

    // Collection state
    bool is_collecting_;
    double collection_start_time_;
    int total_frames_;
    int dropped_frames_;
    int consecutive_stale_count_;
    static constexpr int max_consecutive_stale_ = 50;

    // Collected frames (aligned by 30Hz grid)
    std::vector<std::map<std::string, Eigen::VectorXd>> collected_frames_;
    std::vector<std::map<std::string, std::vector<uint8_t>>> collected_image_frames_;
    std::vector<std::map<std::string, int64_t>> collected_ts_ms_;
    std::vector<std::map<std::string, uint32_t>> collected_esp32_ts_;
    std::vector<std::map<std::string, double>> collected_msg_ts_;
    std::vector<std::map<std::string, double>> collected_recv_ts_;
    std::vector<double> collect_times_;

    std::mutex global_lock_;
};

}  // namespace pico_recorder
```

- [ ] **Step 2: Commit (no build yet — .cpp comes next)**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add include/pico_recorder/data_collector.hpp
git commit -m "feat: add DataCollector header with ts_ms/esp32_ts fields"
```

---

## Task 5: DataCollector — constructor + registration (TDD)

**Files:**
- Create: `test/test_data_collector.cpp`
- Create: `src/data_collector.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

```cpp
// test/test_data_collector.cpp
#include <gtest/gtest.h>
#include <filesystem>
#include "pico_recorder/data_collector.hpp"

using pico_recorder::DataCollector;

static std::string tmp_data_dir() {
    auto p = std::filesystem::temp_directory_path() / "pico_recorder_test";
    std::filesystem::create_directories(p);
    return p.string();
}

TEST(DataCollector, RegistersNumericAndImageDatasets) {
    DataCollector dc(tmp_data_dir(), 30.0, 100, 0.5, "test");
    dc.register_dataset("pose_head", 7, "Head pose", true, -1.0);
    dc.register_image_dataset("cam_left", "Left cam", true, -1.0);

    const auto& cfgs = dc.configs();
    ASSERT_TRUE(cfgs.count("pose_head"));
    ASSERT_TRUE(cfgs.count("cam_left"));
    EXPECT_EQ(cfgs.at("pose_head").size, 7);
    EXPECT_EQ(cfgs.at("cam_left").size, 0);  // image = size 0
    EXPECT_TRUE(cfgs.at("pose_head").required);
}
```

- [ ] **Step 2: Write the minimal .cpp to make it compile + pass**

```cpp
// src/data_collector.cpp
#include "pico_recorder/data_collector.hpp"

#include <hdf5.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <set>
#include <sstream>

namespace pico_recorder {

static double now_seconds() {
    auto tp = std::chrono::system_clock::now();
    return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

DataCollector::DataCollector(const std::string& data_dir,
                              double collection_frequency,
                              int max_queue_size,
                              double max_age,
                              const std::string& task_type,
                              LogFn on_error, LogFn on_warn, LogFn on_info)
    : data_dir_(data_dir),
      collection_frequency_(collection_frequency),
      max_queue_size_(max_queue_size),
      max_age_(max_age),
      task_type_(task_type),
      on_error_(on_error ? on_error : [](const std::string& m){ fprintf(stderr, "[ERROR] %s\n", m.c_str()); }),
      on_warn_ (on_warn  ? on_warn  : [](const std::string& m){ fprintf(stderr, "[WARN]  %s\n", m.c_str()); }),
      on_info_ (on_info  ? on_info  : [](const std::string& m){ printf("[INFO]  %s\n", m.c_str()); }),
      is_collecting_(false),
      collection_start_time_(0.0),
      total_frames_(0),
      dropped_frames_(0),
      consecutive_stale_count_(0) {
    std::filesystem::create_directories(data_dir);
}

void DataCollector::register_dataset(const std::string& name, int size,
                                      const std::string& description,
                                      bool required, double max_age) {
    configs_[name] = DatasetConfig{size, description, required, max_age};
    buffers_[name] = {};
    dataset_stats_[name] = DatasetStats{};
    dataset_order_.push_back(name);
}

void DataCollector::register_image_dataset(const std::string& name,
                                            const std::string& description,
                                            bool required, double max_age) {
    configs_[name] = DatasetConfig{0, description, required, max_age};
    buffers_[name] = {};
    dataset_stats_[name] = DatasetStats{};
    image_datasets_.insert(name);
    image_buffers_[name] = {};
    image_dataset_order_.push_back(name);
    dataset_order_.push_back(name);
}

}  // namespace pico_recorder
```

- [ ] **Step 3: Add DataCollector + test target to `CMakeLists.txt`** (inside/after existing BUILD_TESTING block)

```cmake
# Library target for DataCollector (reusable by node + test)
add_library(data_collector STATIC
  src/data_collector.cpp
)
target_include_directories(data_collector PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${HDF5_INCLUDE_DIRS}
)
target_link_libraries(data_collector
  Eigen3::Eigen
  ${HDF5_C_LIBRARIES}
)

if(BUILD_TESTING)
  ament_add_gtest(test_data_collector test/test_data_collector.cpp)
  target_link_libraries(test_data_collector data_collector)
endif()
```

- [ ] **Step 4: Build + run tests**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
colcon test --packages-select pico_recorder --event-handlers console_direct+
```

Expected: `test_data_collector` passes. `test_pico_frame` still passes.

- [ ] **Step 5: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add src/data_collector.cpp test/test_data_collector.cpp CMakeLists.txt
git commit -m "feat: DataCollector constructor + register (with tests)"
```

---

## Task 6: DataCollector — add_data / add_image_data (TDD)

**Files:**
- Modify: `test/test_data_collector.cpp`
- Modify: `src/data_collector.cpp`

- [ ] **Step 1: Add the failing tests** (append after existing tests)

```cpp
TEST(DataCollector, AddDataStoresTsMs) {
    DataCollector dc(tmp_data_dir(), 30.0, 100, 0.5, "test");
    dc.register_dataset("pose_head", 7, "Head pose", true, -1.0);

    Eigen::VectorXd v(7);
    v << 1, 2, 3, 0, 0, 0, 1;
    dc.add_data("pose_head", v, 1234.567, /*ts_ms=*/42);

    // Indirect check: collect_frame will pick this up — exercise via start+collect+stop roundtrip
    // Here we just ensure no crash; full roundtrip tested in Task 7.
    SUCCEED();
}

TEST(DataCollector, AddImageDataStoresEsp32Ts) {
    DataCollector dc(tmp_data_dir(), 30.0, 100, 0.5, "test");
    dc.register_image_dataset("hand_left", "BLE left", false, -1.0);

    std::vector<uint8_t> bytes = {0x01, 0x02, 0x03};
    dc.add_image_data("hand_left", bytes, 99.0, /*ts_ms=*/7, /*esp32_ts=*/123);
    SUCCEED();
}
```

- [ ] **Step 2: Implement `add_data` and `add_image_data`** (append to `src/data_collector.cpp`)

```cpp
void DataCollector::add_data(const std::string& name, const Eigen::VectorXd& data,
                              double msg_timestamp, int64_t ts_ms) {
    auto it = configs_.find(name);
    if (it == configs_.end()) return;

    double recv_timestamp = now_seconds();
    if (msg_timestamp <= 0.0) msg_timestamp = recv_timestamp;

    FrameData fd{data, msg_timestamp, recv_timestamp, ts_ms};

    std::lock_guard<std::mutex> lk(locks_[name]);
    auto& buf = buffers_[name];
    buf.push_back(std::move(fd));
    if (static_cast<int>(buf.size()) > max_queue_size_)
        buf.pop_front();
}

void DataCollector::add_image_data(const std::string& name,
                                    const std::vector<uint8_t>& bytes,
                                    double msg_timestamp,
                                    int64_t ts_ms,
                                    uint32_t esp32_ts) {
    if (!image_datasets_.count(name)) return;

    double recv_timestamp = now_seconds();
    if (msg_timestamp <= 0.0) msg_timestamp = recv_timestamp;

    ImageFrameData fd{bytes, msg_timestamp, recv_timestamp, ts_ms, esp32_ts};

    std::lock_guard<std::mutex> lk(locks_[name]);
    auto& buf = image_buffers_[name];
    buf.push_back(std::move(fd));
    if (static_cast<int>(buf.size()) > max_queue_size_)
        buf.pop_front();
}
```

- [ ] **Step 3: Build + run tests**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
colcon test --packages-select pico_recorder --event-handlers console_direct+
```

Expected: all 5 tests pass.

- [ ] **Step 4: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add test/test_data_collector.cpp src/data_collector.cpp
git commit -m "feat: DataCollector add_data / add_image_data"
```

---

## Task 7: DataCollector — start / stop / collect_frame (TDD)

**Files:**
- Modify: `test/test_data_collector.cpp`
- Modify: `src/data_collector.cpp`

- [ ] **Step 1: Write the failing test**

```cpp
TEST(DataCollector, CollectFrameCapturesLatest) {
    DataCollector dc(tmp_data_dir(), 30.0, 100, 10.0, "test");
    dc.register_dataset("pose_head", 7, "Head pose", true, -1.0);
    dc.register_image_dataset("cam_left", "Left cam", false, -1.0);

    auto [ok, _] = dc.start_collection();
    ASSERT_TRUE(ok);

    Eigen::VectorXd v(7); v << 1,2,3,4,5,6,7;
    dc.add_data("pose_head", v, 0.0, 100);
    std::vector<uint8_t> img = {0xFF, 0xD8};
    dc.add_image_data("cam_left", img, 0.0, 101);

    auto res = dc.collect_frame();
    EXPECT_TRUE(res.success) << res.message;
    EXPECT_EQ(res.frame_index, 0);

    auto st = dc.status();
    EXPECT_TRUE(st.is_collecting);
    EXPECT_EQ(st.num_frames, 1);

    // Stop without save (small hack: we haven't written save_hdf5 yet, but stop_collection
    // will call it. Use status check after a second collect_frame instead — no stop here.)
    // Note: full save test is in Task 8.
}
```

- [ ] **Step 2: Implement `start_collection`, `stop_collection` (stub save), `collect_frame`, `update_stats`, `status`** (append to `src/data_collector.cpp`)

```cpp
std::pair<bool, std::string> DataCollector::start_collection() {
    std::lock_guard<std::mutex> lk(global_lock_);
    if (is_collecting_) return {false, "Already collecting"};

    is_collecting_ = true;
    collection_start_time_ = now_seconds();
    collected_frames_.clear();
    collected_image_frames_.clear();
    collected_ts_ms_.clear();
    collected_esp32_ts_.clear();
    collected_msg_ts_.clear();
    collected_recv_ts_.clear();
    collect_times_.clear();
    total_frames_ = 0;
    dropped_frames_ = 0;
    consecutive_stale_count_ = 0;

    for (const auto& name : dataset_order_)
        dataset_stats_[name] = DatasetStats{};

    std::ostringstream oss;
    oss << "Started collecting at " << collection_frequency_ << "Hz";
    on_info_(oss.str());
    return {true, oss.str()};
}

std::pair<bool, std::string> DataCollector::stop_collection() {
    std::lock_guard<std::mutex> lk(global_lock_);
    if (!is_collecting_) return {false, "Not collecting"};
    is_collecting_ = false;

    if (collected_frames_.empty() && collected_image_frames_.empty())
        return {false, "No data collected"};

    try {
        save_hdf5();
    } catch (const std::exception& e) {
        return {false, std::string("Save failed: ") + e.what()};
    }

    int n = static_cast<int>(std::max(collected_frames_.size(),
                                       collected_image_frames_.size()));
    double dur = now_seconds() - collection_start_time_;
    double freq = dur > 0 ? n / dur : 0.0;

    std::ostringstream oss;
    oss << "Saved " << n << " frames (" << freq << "Hz) to " << data_dir_;
    on_info_(oss.str());

    collected_frames_.clear();
    collected_image_frames_.clear();
    collected_ts_ms_.clear();
    collected_esp32_ts_.clear();
    collected_msg_ts_.clear();
    collected_recv_ts_.clear();
    collect_times_.clear();
    return {true, oss.str()};
}

CollectResult DataCollector::collect_frame() {
    if (!is_collecting_)
        return {false, "Not collecting", 0, 0.0, {}, {}};

    const double start_time = now_seconds();
    const double collect_time = start_time;

    std::map<std::string, Eigen::VectorXd> frame_data;
    std::map<std::string, std::vector<uint8_t>> frame_imgs;
    std::map<std::string, int64_t> frame_ts_ms;
    std::map<std::string, uint32_t> frame_esp32;
    std::map<std::string, double> frame_msg_ts;
    std::map<std::string, double> frame_recv_ts;
    std::vector<std::string> stale, missing;

    for (const auto& name : dataset_order_) {
        const auto& cfg = configs_[name];

        if (image_datasets_.count(name)) {
            std::lock_guard<std::mutex> lk(locks_[name]);
            auto& buf = image_buffers_[name];
            if (buf.empty()) {
                missing.push_back(name);
                dataset_stats_[name].missing_count++;
                if (cfg.required) {
                    is_collecting_ = false;
                    return {false, "Required image missing: " + name, 0, 0.0, {}, {name}};
                }
                continue;
            }
            const auto& f = buf.back();
            double dt = collect_time - f.recv_timestamp;
            double lat = f.recv_timestamp - f.msg_timestamp;
            auto& s = dataset_stats_[name];
            s.cur_latency = lat;  s.max_latency = std::max(s.max_latency, lat);
            s.avg_latency = 0.1*lat + 0.9*s.avg_latency;
            s.cur_delta_time = dt; s.max_delta_time = std::max(s.max_delta_time, dt);
            s.avg_delta_time = 0.1*dt + 0.9*s.avg_delta_time;

            double eff_age = cfg.max_age > 0 ? cfg.max_age : max_age_;
            if (dt > eff_age) { stale.push_back(name); s.stale_count++; }

            frame_imgs[name]     = f.bytes;
            frame_ts_ms[name]    = f.ts_ms;
            frame_esp32[name]    = f.esp32_ts;
            frame_msg_ts[name]   = f.msg_timestamp;
            frame_recv_ts[name]  = f.recv_timestamp;

            if (buf.size() > 1) buf.erase(buf.begin(), buf.end() - 1);
            continue;
        }

        std::lock_guard<std::mutex> lk(locks_[name]);
        auto& buf = buffers_[name];
        if (buf.empty()) {
            missing.push_back(name);
            dataset_stats_[name].missing_count++;
            if (cfg.required) {
                is_collecting_ = false;
                return {false, "Required data missing: " + name, 0, 0.0, {}, {name}};
            }
            continue;
        }
        const auto& f = buf.back();
        double dt = collect_time - f.recv_timestamp;
        double lat = f.recv_timestamp - f.msg_timestamp;
        auto& s = dataset_stats_[name];
        s.cur_latency = lat; s.max_latency = std::max(s.max_latency, lat);
        s.avg_latency = 0.1*lat + 0.9*s.avg_latency;
        s.cur_delta_time = dt; s.max_delta_time = std::max(s.max_delta_time, dt);
        s.avg_delta_time = 0.1*dt + 0.9*s.avg_delta_time;

        double eff_age = cfg.max_age > 0 ? cfg.max_age : max_age_;
        if (dt > eff_age) {
            stale.push_back(name);
            s.stale_count++;
            if (cfg.required) {
                consecutive_stale_count_++;
                dropped_frames_++;
                if (consecutive_stale_count_ >= max_consecutive_stale_) {
                    is_collecting_ = false;
                    return {false, "Stopped: " + name + " stale too long", 0, 0.0, stale, {}};
                }
                return {false, "Skipping stale frame: " + name, 0, 0.0, stale, {}};
            }
        }

        frame_data[name]    = f.data;
        frame_ts_ms[name]   = f.ts_ms;
        frame_msg_ts[name]  = f.msg_timestamp;
        frame_recv_ts[name] = f.recv_timestamp;

        if (buf.size() > 1) buf.erase(buf.begin(), buf.end() - 1);
    }

    consecutive_stale_count_ = 0;

    {
        std::lock_guard<std::mutex> lk(global_lock_);
        if (is_collecting_) {
            collected_frames_.push_back(std::move(frame_data));
            collected_image_frames_.push_back(std::move(frame_imgs));
            collected_ts_ms_.push_back(std::move(frame_ts_ms));
            collected_esp32_ts_.push_back(std::move(frame_esp32));
            collected_msg_ts_.push_back(std::move(frame_msg_ts));
            collected_recv_ts_.push_back(std::move(frame_recv_ts));
            collect_times_.push_back(collect_time);
            total_frames_++;
        }
    }

    int idx = static_cast<int>(collected_frames_.size()) - 1;
    return {true, "OK", idx, now_seconds() - start_time, stale, missing};
}

void DataCollector::update_stats() {
    const double now = now_seconds();
    for (const auto& name : dataset_order_) {
        std::lock_guard<std::mutex> lk(locks_[name]);
        auto& s = dataset_stats_[name];
        if (image_datasets_.count(name)) {
            auto& buf = image_buffers_[name];
            if (buf.empty()) { s.missing_count++; continue; }
            const auto& f = buf.back();
            s.cur_latency = f.recv_timestamp - f.msg_timestamp;
            s.cur_delta_time = now - f.recv_timestamp;
            continue;
        }
        auto& buf = buffers_[name];
        if (buf.empty()) { s.missing_count++; continue; }
        const auto& f = buf.back();
        s.cur_latency = f.recv_timestamp - f.msg_timestamp;
        s.cur_delta_time = now - f.recv_timestamp;
    }
}

DataCollector::Status DataCollector::status() const {
    return Status{
        is_collecting_,
        is_collecting_ ? static_cast<int>(collected_frames_.size())
                       : 0,
        is_collecting_ ? now_seconds() - collection_start_time_ : 0.0,
    };
}

// Stub to be filled in next task:
void DataCollector::save_hdf5() {
    on_warn_("save_hdf5 not yet implemented — stub does nothing");
}
```

- [ ] **Step 3: Build + run tests**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
colcon test --packages-select pico_recorder --event-handlers console_direct+
```

Expected: all 6 tests pass.

- [ ] **Step 4: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add test/test_data_collector.cpp src/data_collector.cpp
git commit -m "feat: DataCollector collect_frame + start/stop (save stubbed)"
```

---

## Task 8: DataCollector — `save_hdf5` with per-stream subgroups (TDD)

**Files:**
- Modify: `test/test_data_collector.cpp`
- Modify: `src/data_collector.cpp`

- [ ] **Step 1: Add the failing roundtrip test**

```cpp
TEST(DataCollector, SaveHdf5RoundTrip) {
    std::string dir = tmp_data_dir() + "/roundtrip";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    DataCollector dc(dir, 30.0, 100, 10.0, "pico_test");
    dc.register_dataset("pose_head", 7, "Head pose", true, -1.0);
    dc.register_image_dataset("cam_left", "Left cam JPEG", false, -1.0);
    dc.register_image_dataset("hand_left", "Left BLE raw", false, -1.0);

    dc.start_collection();

    Eigen::VectorXd v(7); v << 0.1, 0.2, 0.3, 0, 0, 0, 1;
    dc.add_data("pose_head", v, 1000.0, 111);
    dc.add_image_data("cam_left", {0xFF, 0xD8, 0xAA}, 1000.0, 111);
    dc.add_image_data("hand_left", {0x01, 0x00, 0x00, 0x00, 0x42}, 1000.0, 111, /*esp32_ts=*/1);

    auto res = dc.collect_frame();
    ASSERT_TRUE(res.success) << res.message;

    auto [ok, msg] = dc.stop_collection();
    ASSERT_TRUE(ok) << msg;

    // Find the saved file
    std::string hdf5_dir = dir + "/hdf5";
    std::vector<std::filesystem::path> files;
    for (auto& p : std::filesystem::directory_iterator(hdf5_dir))
        if (p.path().extension() == ".hdf5") files.push_back(p.path());
    ASSERT_EQ(files.size(), 1u);

    // Open and verify structure via HDF5 C API
    hid_t file = H5Fopen(files[0].c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    ASSERT_GE(file, 0);
    EXPECT_GE(H5Lexists(file, "/states/pose_head/pos", H5P_DEFAULT), 1);
    EXPECT_GE(H5Lexists(file, "/states/pose_head/rot_xyzw", H5P_DEFAULT), 1);
    EXPECT_GE(H5Lexists(file, "/states/pose_head/ts_ms", H5P_DEFAULT), 1);
    EXPECT_GE(H5Lexists(file, "/states/pose_head/recv_stamp", H5P_DEFAULT), 1);
    EXPECT_GE(H5Lexists(file, "/observations/images/cam_left/jpeg", H5P_DEFAULT), 1);
    EXPECT_GE(H5Lexists(file, "/observations/images/cam_left/ts_ms", H5P_DEFAULT), 1);
    EXPECT_GE(H5Lexists(file, "/observations/images/hand_left/data", H5P_DEFAULT), 1);
    EXPECT_GE(H5Lexists(file, "/observations/images/hand_left/esp32_ts", H5P_DEFAULT), 1);
    EXPECT_GE(H5Lexists(file, "/timestamp/collect_time", H5P_DEFAULT), 1);
    H5Fclose(file);
}
```

Also add `#include <hdf5.h>` to the top of the test file.

- [ ] **Step 2: Replace the stub `save_hdf5` with the real implementation** (in `src/data_collector.cpp`, delete the stub and append)

```cpp
namespace {

void write_f64_2d(hid_t g, const char* name,
                  const std::vector<std::vector<double>>& data, int cols) {
    if (data.empty()) return;
    hsize_t dims[2] = {data.size(), static_cast<hsize_t>(cols)};
    hid_t space = H5Screate_simple(2, dims, nullptr);
    std::vector<double> flat(dims[0] * dims[1], std::nan(""));
    for (hsize_t r = 0; r < dims[0]; ++r)
        for (hsize_t c = 0; c < dims[1] && c < data[r].size(); ++c)
            flat[r * dims[1] + c] = data[r][c];
    hid_t ds = H5Dcreate2(g, name, H5T_IEEE_F64LE, space,
                           H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(ds, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
    H5Dclose(ds); H5Sclose(space);
}

void write_f64_1d(hid_t g, const char* name, const std::vector<double>& data) {
    if (data.empty()) return;
    hsize_t dims[1] = {data.size()};
    hid_t space = H5Screate_simple(1, dims, nullptr);
    hid_t ds = H5Dcreate2(g, name, H5T_IEEE_F64LE, space,
                           H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(ds, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.data());
    H5Dclose(ds); H5Sclose(space);
}

void write_i64_1d(hid_t g, const char* name, const std::vector<int64_t>& data) {
    if (data.empty()) return;
    hsize_t dims[1] = {data.size()};
    hid_t space = H5Screate_simple(1, dims, nullptr);
    hid_t ds = H5Dcreate2(g, name, H5T_STD_I64LE, space,
                           H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(ds, H5T_NATIVE_LLONG, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.data());
    H5Dclose(ds); H5Sclose(space);
}

void write_u32_1d(hid_t g, const char* name, const std::vector<uint32_t>& data) {
    if (data.empty()) return;
    hsize_t dims[1] = {data.size()};
    hid_t space = H5Screate_simple(1, dims, nullptr);
    hid_t ds = H5Dcreate2(g, name, H5T_STD_U32LE, space,
                           H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(ds, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.data());
    H5Dclose(ds); H5Sclose(space);
}

void write_vlen_u8(hid_t g, const char* name,
                   const std::vector<std::vector<uint8_t>>& frames,
                   const std::string& format,
                   const std::string& description) {
    if (frames.empty()) return;
    hid_t vlen = H5Tvlen_create(H5T_NATIVE_UINT8);
    hsize_t dims[1] = {frames.size()};
    hid_t space = H5Screate_simple(1, dims, nullptr);

    hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
    hsize_t chunk = std::min(frames.size(), size_t{64});
    if (chunk > 0) {
        H5Pset_chunk(dcpl, 1, &chunk);
        H5Pset_deflate(dcpl, 1);
    }
    hid_t ds = H5Dcreate2(g, name, vlen, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);

    std::vector<hvl_t> v(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        v[i].len = frames[i].size();
        v[i].p = frames[i].empty() ? nullptr : const_cast<uint8_t*>(frames[i].data());
    }
    H5Dwrite(ds, vlen, H5S_ALL, H5S_ALL, H5P_DEFAULT, v.data());

    // Attributes
    hid_t aspace = H5Screate(H5S_SCALAR);
    hid_t atype = H5Tcopy(H5T_C_S1);
    H5Tset_size(atype, format.size());
    hid_t a = H5Acreate2(ds, "format", atype, aspace, H5P_DEFAULT, H5P_DEFAULT);
    H5Awrite(a, atype, format.c_str()); H5Aclose(a);
    H5Tset_size(atype, description.size());
    a = H5Acreate2(ds, "description", atype, aspace, H5P_DEFAULT, H5P_DEFAULT);
    H5Awrite(a, atype, description.c_str()); H5Aclose(a);
    H5Tclose(atype); H5Sclose(aspace);

    H5Dclose(ds); H5Pclose(dcpl); H5Sclose(space); H5Tclose(vlen);
}

}  // namespace

void DataCollector::save_hdf5() {
    std::string hdf5_dir = data_dir_ + "/hdf5";
    std::filesystem::create_directories(hdf5_dir);

    long ts = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::string path = hdf5_dir + "/pico_" + std::to_string(ts) + ".hdf5";

    hid_t file = H5Fcreate(path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    if (file < 0) {
        on_error_("Failed to create HDF5 file: " + path);
        return;
    }

    const int n = static_cast<int>(std::max(collected_frames_.size(),
                                             collected_image_frames_.size()));
    const double duration = now_seconds() - collection_start_time_;

    // Root attrs
    {
        hid_t aspace = H5Screate(H5S_SCALAR);
        auto write_d = [&](const char* k, double v) {
            hid_t a = H5Acreate2(file, k, H5T_IEEE_F64LE, aspace, H5P_DEFAULT, H5P_DEFAULT);
            H5Awrite(a, H5T_NATIVE_DOUBLE, &v); H5Aclose(a);
        };
        auto write_i = [&](const char* k, int v) {
            hid_t a = H5Acreate2(file, k, H5T_STD_I32LE, aspace, H5P_DEFAULT, H5P_DEFAULT);
            H5Awrite(a, H5T_NATIVE_INT, &v); H5Aclose(a);
        };
        hid_t atype = H5Tcopy(H5T_C_S1);
        H5Tset_size(atype, task_type_.size());
        hid_t a = H5Acreate2(file, "task_type", atype, aspace, H5P_DEFAULT, H5P_DEFAULT);
        H5Awrite(a, atype, task_type_.c_str()); H5Aclose(a); H5Tclose(atype);

        write_d("collection_frequency", collection_frequency_);
        write_d("actual_frequency", duration > 0 ? n / duration : 0.0);
        write_d("max_age", max_age_);
        write_d("duration", duration);
        write_d("collection_start_time", collection_start_time_);
        write_i("num_frames", n);
        write_i("dropped_frames", dropped_frames_);
        H5Sclose(aspace);
    }

    // /states/<name>/{pos, rot_xyzw, ts_ms, recv_stamp}
    hid_t states = H5Gcreate2(file, "states", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    for (const auto& name : dataset_order_) {
        if (image_datasets_.count(name)) continue;
        const auto& cfg = configs_[name];
        hid_t g = H5Gcreate2(states, name.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

        std::vector<std::vector<double>> pos(n, std::vector<double>(3, std::nan("")));
        std::vector<std::vector<double>> rot(n, std::vector<double>(4, std::nan("")));
        std::vector<int64_t> ts_ms(n, 0);
        std::vector<double>  recv(n, 0.0);

        bool has_rot = (cfg.size >= 7);
        for (int i = 0; i < n && i < static_cast<int>(collected_frames_.size()); ++i) {
            auto it = collected_frames_[i].find(name);
            if (it != collected_frames_[i].end()) {
                const auto& v = it->second;
                for (int j = 0; j < 3 && j < v.size(); ++j) pos[i][j] = v[j];
                if (has_rot) for (int j = 0; j < 4 && (3 + j) < v.size(); ++j) rot[i][j] = v[3 + j];
            }
            auto it_ts = collected_ts_ms_[i].find(name);
            if (it_ts != collected_ts_ms_[i].end()) ts_ms[i] = it_ts->second;
            auto it_rv = collected_recv_ts_[i].find(name);
            if (it_rv != collected_recv_ts_[i].end()) recv[i] = it_rv->second;
        }

        write_f64_2d(g, "pos", pos, 3);
        if (has_rot) write_f64_2d(g, "rot_xyzw", rot, 4);
        write_i64_1d(g, "ts_ms", ts_ms);
        write_f64_1d(g, "recv_stamp", recv);
        H5Gclose(g);
    }
    H5Gclose(states);

    // /observations/images/<name>/{jpeg|data, ts_ms, esp32_ts?, recv_stamp}
    if (!image_dataset_order_.empty()) {
        hid_t obs = H5Gcreate2(file, "observations", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        hid_t img = H5Gcreate2(obs, "images", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

        for (const auto& name : image_dataset_order_) {
            hid_t g = H5Gcreate2(img, name.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

            std::vector<std::vector<uint8_t>> bytes(n);
            std::vector<int64_t>  ts_ms(n, 0);
            std::vector<uint32_t> esp32(n, 0);
            std::vector<double>   recv(n, 0.0);
            bool any_esp32 = false;

            for (int i = 0; i < n && i < static_cast<int>(collected_image_frames_.size()); ++i) {
                auto it = collected_image_frames_[i].find(name);
                if (it != collected_image_frames_[i].end()) bytes[i] = it->second;
                auto it_ts = collected_ts_ms_[i].find(name);
                if (it_ts != collected_ts_ms_[i].end()) ts_ms[i] = it_ts->second;
                auto it_es = collected_esp32_ts_[i].find(name);
                if (it_es != collected_esp32_ts_[i].end()) {
                    esp32[i] = it_es->second;
                    if (esp32[i] != 0) any_esp32 = true;
                }
                auto it_rv = collected_recv_ts_[i].find(name);
                if (it_rv != collected_recv_ts_[i].end()) recv[i] = it_rv->second;
            }

            // Heuristic: "cam_*" saves as jpeg; anything else (e.g. "hand_*") saves as data.
            bool is_cam = (name.rfind("cam", 0) == 0);
            write_vlen_u8(g,
                is_cam ? "jpeg" : "data",
                bytes,
                is_cam ? "jpeg" : "raw",
                configs_[name].description);

            write_i64_1d(g, "ts_ms", ts_ms);
            if (any_esp32) write_u32_1d(g, "esp32_ts", esp32);
            write_f64_1d(g, "recv_stamp", recv);
            H5Gclose(g);
        }

        H5Gclose(img); H5Gclose(obs);
    }

    // /timestamp/collect_time
    {
        hid_t g = H5Gcreate2(file, "timestamp", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        write_f64_1d(g, "collect_time", collect_times_);
        H5Gclose(g);
    }

    H5Fclose(file);
    on_info_("HDF5 saved: " + path);
}
```

- [ ] **Step 3: Build + run all tests**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
colcon test --packages-select pico_recorder --event-handlers console_direct+
```

Expected: all 7 tests pass. HDF5 file created under `/tmp/pico_recorder_test/roundtrip/hdf5/`.

- [ ] **Step 4: Spot-check the HDF5 file manually**

```bash
python3 -c "
import h5py, sys
for f in __import__('glob').glob('/tmp/pico_recorder_test/roundtrip/hdf5/*.hdf5'):
    print(f)
    with h5py.File(f, 'r') as h:
        def show(name, obj):
            print(' ', name, obj.shape if hasattr(obj, 'shape') else '(group)')
        h.visititems(show)
        print('  attrs:', dict(h.attrs))
"
```

Expected: pose_head datasets shape (1,3) and (1,4); cam_left/jpeg and hand_left/data shapes (1,); ts_ms values = 111.

- [ ] **Step 5: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add src/data_collector.cpp test/test_data_collector.cpp
git commit -m "feat: DataCollector save_hdf5 with per-stream subgroups"
```

---

## Task 9: Bridge node — TCP client + publishers

**Files:**
- Create: `src/pico_bridge_node.cpp`
- Modify: `CMakeLists.txt` (add executable + link)

- [ ] **Step 1: Write the bridge node**

```cpp
// src/pico_bridge_node.cpp
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pico_recorder/msg/ble_frame.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "pico_recorder/pico_frame.hpp"

using namespace std::chrono_literals;
namespace pf = pico_recorder;

class PicoBridgeNode : public rclcpp::Node {
public:
    PicoBridgeNode() : Node("pico_bridge") {
        host_ = declare_parameter<std::string>("host", "127.0.0.1");
        port_ = declare_parameter<int>("port", 9999);

        auto sensor_qos = rclcpp::SensorDataQoS();
        cam_l_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/pico/cam_left/compressed", sensor_qos);
        cam_r_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/pico/cam_right/compressed", sensor_qos);

        pose_h_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pico/pose/head", 10);
        pose_l_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pico/pose/left_hand", 10);
        pose_r_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pico/pose/right_hand", 10);

        ble_l_pub_ = create_publisher<pico_recorder::msg::BleFrame>("/pico/ble/left", 10);
        ble_r_pub_ = create_publisher<pico_recorder::msg::BleFrame>("/pico/ble/right", 10);

        running_ = true;
        thread_ = std::thread(&PicoBridgeNode::recv_loop, this);

        RCLCPP_INFO(get_logger(), "PicoBridge connecting to %s:%d ...", host_.c_str(), port_);
    }

    ~PicoBridgeNode() override {
        running_ = false;
        if (sock_ >= 0) ::shutdown(sock_, SHUT_RDWR);
        if (thread_.joinable()) thread_.join();
        if (sock_ >= 0) ::close(sock_);
    }

private:
    static builtin_interfaces::msg::Time ts_from_ms(int64_t ts_ms) {
        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(ts_ms / 1000);
        t.nanosec = static_cast<uint32_t>((ts_ms % 1000) * 1'000'000);
        return t;
    }

    bool recv_exact(int fd, uint8_t* buf, size_t n) {
        size_t got = 0;
        while (got < n) {
            ssize_t r = ::recv(fd, buf + got, n - got, 0);
            if (r <= 0) return false;
            got += static_cast<size_t>(r);
        }
        return true;
    }

    int connect_once() {
        int s = ::socket(AF_INET, SOCK_STREAM, 0);
        if (s < 0) return -1;
        int one = 1;
        ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port_));
        ::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);
        if (::connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(s);
            return -1;
        }
        return s;
    }

    void publish_pose(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
                     int64_t ts_ms, const uint8_t* payload, size_t len) {
        float pos[3], q[4];
        if (!pf::parse_pose_payload(payload, len, pos, q)) return;
        geometry_msgs::msg::PoseStamped m;
        m.header.stamp = ts_from_ms(ts_ms);
        m.header.frame_id = "pico";
        m.pose.position.x = pos[0];
        m.pose.position.y = pos[1];
        m.pose.position.z = pos[2];
        m.pose.orientation.x = q[0];
        m.pose.orientation.y = q[1];
        m.pose.orientation.z = q[2];
        m.pose.orientation.w = q[3];
        pub->publish(m);
    }

    void publish_cam(rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub,
                     int64_t ts_ms, std::vector<uint8_t>&& jpeg) {
        sensor_msgs::msg::CompressedImage m;
        m.header.stamp = ts_from_ms(ts_ms);
        m.header.frame_id = "pico";
        m.format = "jpeg";
        m.data = std::move(jpeg);
        pub->publish(m);
    }

    void publish_ble(rclcpp::Publisher<pico_recorder::msg::BleFrame>::SharedPtr pub,
                     int64_t ts_ms, const uint8_t* payload, size_t len) {
        uint32_t esp32_ts;
        const uint8_t* data_ptr;
        size_t data_len;
        if (!pf::split_ble_payload(payload, len, esp32_ts, data_ptr, data_len)) return;

        pico_recorder::msg::BleFrame m;
        m.header.stamp = ts_from_ms(ts_ms);
        m.header.frame_id = "pico";
        m.esp32_ts = esp32_ts;
        m.data.assign(data_ptr, data_ptr + data_len);
        pub->publish(m);
    }

    void recv_loop() {
        while (running_ && rclcpp::ok()) {
            sock_ = connect_once();
            if (sock_ < 0) {
                RCLCPP_WARN(get_logger(), "Connect to %s:%d failed, retrying in 2s",
                            host_.c_str(), port_);
                std::this_thread::sleep_for(2s);
                continue;
            }
            RCLCPP_INFO(get_logger(), "Connected to %s:%d", host_.c_str(), port_);

            while (running_ && rclcpp::ok()) {
                uint8_t header[pf::HEADER_SIZE];
                if (!recv_exact(sock_, header, pf::HEADER_SIZE)) break;
                pf::FrameHeader h;
                if (!pf::parse_frame_header(header, h)) {
                    RCLCPP_WARN(get_logger(), "Bad header (magic=0x%02X), resyncing", h.magic);
                    continue;
                }
                std::vector<uint8_t> payload(h.payload_len);
                if (h.payload_len > 0 && !recv_exact(sock_, payload.data(), h.payload_len)) break;

                switch (h.type) {
                    case pf::TYPE_CAM_LEFT:
                        publish_cam(cam_l_pub_, h.ts_ms, std::move(payload)); break;
                    case pf::TYPE_CAM_RIGHT:
                        publish_cam(cam_r_pub_, h.ts_ms, std::move(payload)); break;
                    case pf::TYPE_POSE_LEFT:
                        publish_pose(pose_l_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_POSE_RIGHT:
                        publish_pose(pose_r_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_POSE_HEAD:
                        publish_pose(pose_h_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_BLE_LEFT:
                        publish_ble(ble_l_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_BLE_RIGHT:
                        publish_ble(ble_r_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    default:
                        RCLCPP_WARN(get_logger(), "Unknown type 0x%02X, skipping", h.type);
                }
            }

            RCLCPP_WARN(get_logger(), "Disconnected; reconnecting in 2s");
            ::close(sock_);
            sock_ = -1;
            std::this_thread::sleep_for(2s);
        }
    }

    std::string host_;
    int port_ = 9999;
    std::atomic<bool> running_{false};
    int sock_ = -1;
    std::thread thread_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cam_l_pub_, cam_r_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_h_pub_, pose_l_pub_, pose_r_pub_;
    rclcpp::Publisher<pico_recorder::msg::BleFrame>::SharedPtr ble_l_pub_, ble_r_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicoBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 2: Add bridge executable to `CMakeLists.txt`** (insert before `if(BUILD_TESTING)`)

```cmake
add_executable(pico_bridge_node src/pico_bridge_node.cpp)
target_include_directories(pico_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(pico_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  builtin_interfaces
)
# Link the generated BleFrame message interface into this target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(pico_bridge_node "${cpp_typesupport_target}")

install(TARGETS pico_bridge_node DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 3: Build**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
```

Expected: build succeeds, `install/pico_recorder/lib/pico_recorder/pico_bridge_node` exists.

- [ ] **Step 4: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add src/pico_bridge_node.cpp CMakeLists.txt
git commit -m "feat: pico_bridge_node TCP→ROS publisher"
```

---

## Task 10: Recorder node — subscriptions + DataCollector integration

**Files:**
- Create: `src/pico_recorder_node.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the recorder node**

```cpp
// src/pico_recorder_node.cpp
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pico_recorder/msg/ble_frame.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "pico_recorder/data_collector.hpp"

namespace pr = pico_recorder;

class PicoRecorderNode : public rclcpp::Node {
public:
    PicoRecorderNode() : Node("pico_recorder") {
        std::string data_dir = declare_parameter<std::string>("data_dir", "/tmp/pico");
        double freq          = declare_parameter<double>("collection_frequency", 30.0);
        double max_age       = declare_parameter<double>("max_age", 0.2);
        std::string task     = declare_parameter<std::string>("task_type", "pico_collection");

        dc_ = std::make_unique<pr::DataCollector>(
            data_dir, freq, /*max_queue_size=*/200, max_age, task,
            [this](const std::string& m){ RCLCPP_ERROR(get_logger(), "%s", m.c_str()); },
            [this](const std::string& m){ RCLCPP_WARN (get_logger(), "%s", m.c_str()); },
            [this](const std::string& m){ RCLCPP_INFO (get_logger(), "%s", m.c_str()); });

        dc_->register_dataset("pose_head",       7, "Head 6DOF (pos3 + quat4 xyzw)",       true,  -1.0);
        dc_->register_dataset("pose_left_hand",  7, "Left controller 6DOF",                true,  -1.0);
        dc_->register_dataset("pose_right_hand", 7, "Right controller 6DOF",               true,  -1.0);
        dc_->register_image_dataset("cam_left",  "PICO left camera JPEG",                  true,  -1.0);
        dc_->register_image_dataset("cam_right", "PICO right camera JPEG",                 true,  -1.0);
        dc_->register_image_dataset("hand_left",
            "Left BLE raw payload [4B esp32_ts | sensor_bytes]",                           false, -1.0);
        dc_->register_image_dataset("hand_right",
            "Right BLE raw payload [4B esp32_ts | sensor_bytes]",                          false, -1.0);

        auto sqos = rclcpp::SensorDataQoS();
        cam_l_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/pico/cam_left/compressed", sqos,
            [this](sensor_msgs::msg::CompressedImage::SharedPtr m){ on_cam("cam_left", m); });
        cam_r_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/pico/cam_right/compressed", sqos,
            [this](sensor_msgs::msg::CompressedImage::SharedPtr m){ on_cam("cam_right", m); });

        pose_h_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pico/pose/head", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m){ on_pose("pose_head", m); });
        pose_l_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pico/pose/left_hand", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m){ on_pose("pose_left_hand", m); });
        pose_r_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pico/pose/right_hand", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m){ on_pose("pose_right_hand", m); });

        ble_l_sub_ = create_subscription<pico_recorder::msg::BleFrame>(
            "/pico/ble/left", 10,
            [this](pico_recorder::msg::BleFrame::SharedPtr m){ on_ble("hand_left", m); });
        ble_r_sub_ = create_subscription<pico_recorder::msg::BleFrame>(
            "/pico/ble/right", 10,
            [this](pico_recorder::msg::BleFrame::SharedPtr m){ on_ble("hand_right", m); });

        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "/pico/start_collect",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                auto [ok, msg] = dc_->start_collection();
                resp->success = ok; resp->message = msg;
            });
        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "/pico/stop_collect",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                auto [ok, msg] = dc_->stop_collection();
                resp->success = ok; resp->message = msg;
            });

        auto period = std::chrono::duration<double>(1.0 / freq);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() {
                if (dc_->is_collecting()) dc_->collect_frame();
                else                      dc_->update_stats();
            });

        running_ = true;
        kb_thread_ = std::thread(&PicoRecorderNode::keyboard_loop, this);

        RCLCPP_INFO(get_logger(),
            "pico_recorder ready. Press 'a' to start, 's' to stop+save, 'q' to quit. data_dir=%s",
            data_dir.c_str());
    }

    ~PicoRecorderNode() override {
        running_ = false;
        if (kb_thread_.joinable()) kb_thread_.join();
    }

private:
    static int64_t ts_ms_from_stamp(const builtin_interfaces::msg::Time& st) {
        return static_cast<int64_t>(st.sec) * 1000 + st.nanosec / 1'000'000;
    }
    static double secs_from_stamp(const builtin_interfaces::msg::Time& st) {
        return static_cast<double>(st.sec) + st.nanosec * 1e-9;
    }

    void on_pose(const std::string& name, geometry_msgs::msg::PoseStamped::SharedPtr m) {
        Eigen::VectorXd v(7);
        v << m->pose.position.x, m->pose.position.y, m->pose.position.z,
             m->pose.orientation.x, m->pose.orientation.y,
             m->pose.orientation.z, m->pose.orientation.w;
        dc_->add_data(name, v, secs_from_stamp(m->header.stamp), ts_ms_from_stamp(m->header.stamp));
    }

    void on_cam(const std::string& name, sensor_msgs::msg::CompressedImage::SharedPtr m) {
        dc_->add_image_data(name, m->data,
                             secs_from_stamp(m->header.stamp),
                             ts_ms_from_stamp(m->header.stamp),
                             0);
    }

    void on_ble(const std::string& name, pico_recorder::msg::BleFrame::SharedPtr m) {
        // Reconstruct the original PICO wire payload: [4B esp32_ts LE][sensor_bytes]
        std::vector<uint8_t> raw;
        raw.reserve(4 + m->data.size());
        raw.push_back(static_cast<uint8_t>(m->esp32_ts & 0xFF));
        raw.push_back(static_cast<uint8_t>((m->esp32_ts >> 8) & 0xFF));
        raw.push_back(static_cast<uint8_t>((m->esp32_ts >> 16) & 0xFF));
        raw.push_back(static_cast<uint8_t>((m->esp32_ts >> 24) & 0xFF));
        raw.insert(raw.end(), m->data.begin(), m->data.end());
        dc_->add_image_data(name, raw,
                             secs_from_stamp(m->header.stamp),
                             ts_ms_from_stamp(m->header.stamp),
                             m->esp32_ts);
    }

    void keyboard_loop() {
        if (!isatty(STDIN_FILENO)) {
            RCLCPP_WARN(get_logger(), "stdin not a tty — keyboard disabled (use ROS services)");
            return;
        }
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (running_ && rclcpp::ok()) {
            fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
            timeval tv{0, 100000};
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
                char c;
                if (::read(STDIN_FILENO, &c, 1) == 1) {
                    c = static_cast<char>(std::tolower(c));
                    if (c == 'a') {
                        auto [ok, msg] = dc_->start_collection();
                        RCLCPP_INFO(get_logger(), "%s", msg.c_str());
                    } else if (c == 's') {
                        auto [ok, msg] = dc_->stop_collection();
                        if (ok) RCLCPP_INFO(get_logger(), "%s", msg.c_str());
                        else    RCLCPP_WARN(get_logger(), "%s", msg.c_str());
                    } else if (c == 'q') {
                        running_ = false;
                        rclcpp::shutdown();
                        break;
                    }
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    std::unique_ptr<pr::DataCollector> dc_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr cam_l_sub_, cam_r_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   pose_h_sub_, pose_l_sub_, pose_r_sub_;
    rclcpp::Subscription<pico_recorder::msg::BleFrame>::SharedPtr       ble_l_sub_, ble_r_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> running_{false};
    std::thread kb_thread_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicoRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 2: Add recorder executable to `CMakeLists.txt`** (next to bridge target)

```cmake
add_executable(pico_recorder_node src/pico_recorder_node.cpp)
target_include_directories(pico_recorder_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(pico_recorder_node data_collector)
ament_target_dependencies(pico_recorder_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  builtin_interfaces
)
target_link_libraries(pico_recorder_node "${cpp_typesupport_target}")

install(TARGETS pico_recorder_node DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 3: Build**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
```

Expected: build succeeds, both executables installed under `install/pico_recorder/lib/pico_recorder/`.

- [ ] **Step 4: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add src/pico_recorder_node.cpp CMakeLists.txt
git commit -m "feat: pico_recorder_node subscribers + services + keyboard"
```

---

## Task 11: Launch file + install

**Files:**
- Create: `launch/start_pico_recorder.launch.py`
- Modify: `CMakeLists.txt` (install launch)

- [ ] **Step 1: Write the launch file**

```python
# launch/start_pico_recorder.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument("host", default_value="127.0.0.1")
    port_arg = DeclareLaunchArgument("port", default_value="9999")
    data_dir_arg = DeclareLaunchArgument("data_dir", default_value="/tmp/pico")
    freq_arg = DeclareLaunchArgument("collection_frequency", default_value="30.0")
    max_age_arg = DeclareLaunchArgument("max_age", default_value="0.2")

    bridge = Node(
        package="pico_recorder",
        executable="pico_bridge_node",
        name="pico_bridge",
        output="screen",
        parameters=[{
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
        }],
    )
    recorder = Node(
        package="pico_recorder",
        executable="pico_recorder_node",
        name="pico_recorder",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "data_dir": LaunchConfiguration("data_dir"),
            "collection_frequency": LaunchConfiguration("collection_frequency"),
            "max_age": LaunchConfiguration("max_age"),
        }],
    )
    return LaunchDescription([host_arg, port_arg, data_dir_arg, freq_arg, max_age_arg,
                              bridge, recorder])
```

- [ ] **Step 2: Install launch directory via `CMakeLists.txt`** (append before `ament_package()`)

```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

- [ ] **Step 3: Build + verify launch resolves**

```bash
cd /home/eai/project/pico_project/ws
colcon build --packages-select pico_recorder
source install/setup.bash
ros2 launch pico_recorder start_pico_recorder.launch.py --show-args
```

Expected: prints the five launch arguments.

- [ ] **Step 4: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add launch/start_pico_recorder.launch.py CMakeLists.txt
git commit -m "feat: add launch file for bridge+recorder"
```

---

## Task 12: Mock PICO server + integration smoke test

**Files:**
- Create: `scripts/mock_pico_server.py`

- [ ] **Step 1: Write the mock server**

```python
#!/usr/bin/env python3
"""
Mock PicoStreamingServer for integration testing.

Listens on 127.0.0.1:9999 and pushes fake frames in the official wire format.
Useful when the real PICO isn't connected.

Frame layout: [0xAB][type:1B][ts_ms:8B LE i64][payload_len:4B LE u32][payload]
"""
import argparse
import math
import socket
import struct
import time


def make_header(frame_type: int, ts_ms: int, payload_len: int) -> bytes:
    return struct.pack("<BBqI", 0xAB, frame_type, ts_ms, payload_len)


def tiny_jpeg() -> bytes:
    # Minimal valid-looking "JPEG" — a real JPEG SOI+EOI so cv2.imdecode returns None cleanly.
    # (Integration tests don't need a decodable image, just bytes that travel end-to-end.)
    return b"\xFF\xD8\xFF\xD9" + b"FAKE_JPEG"


def pose_payload(t: float, offset: float) -> bytes:
    return struct.pack(
        "<7f",
        math.sin(t),       math.cos(t),       offset,
        0.0, 0.0, 0.0, 1.0,
    )


def ble_payload(ts_ms: int, payload_byte: int) -> bytes:
    return struct.pack("<I", ts_ms) + bytes([payload_byte, payload_byte ^ 0xFF])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9999)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Run for this many seconds (0 = forever)")
    args = ap.parse_args()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(1)
    print(f"[mock] listening on 127.0.0.1:{args.port}")

    while True:
        conn, addr = srv.accept()
        print(f"[mock] client connected: {addr}")
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        start = time.time()
        frame_idx = 0
        try:
            while args.duration == 0 or (time.time() - start) < args.duration:
                ts_ms = int((time.time() - start) * 1000)
                t = ts_ms / 1000.0

                # Cameras (0x01, 0x02)
                for ftype in (0x01, 0x02):
                    jp = tiny_jpeg()
                    conn.sendall(make_header(ftype, ts_ms, len(jp)) + jp)

                # Poses (0x03, 0x04, 0x05)
                for ftype, off in ((0x03, 0.0), (0x04, 1.0), (0x05, 2.0)):
                    p = pose_payload(t, off)
                    conn.sendall(make_header(ftype, ts_ms, len(p)) + p)

                # BLE (0x10, 0x11)
                for ftype, byte in ((0x10, 0xA0), (0x11, 0xB0)):
                    p = ble_payload(ts_ms, byte + (frame_idx & 0x0F))
                    conn.sendall(make_header(ftype, ts_ms, len(p)) + p)

                frame_idx += 1
                time.sleep(1.0 / args.fps)
        except (BrokenPipeError, ConnectionResetError):
            print("[mock] client disconnected")
        finally:
            conn.close()
            if args.duration > 0:
                break

    srv.close()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Mark script executable**

```bash
chmod +x /home/eai/project/pico_project/pico_reciver_data/scripts/mock_pico_server.py
```

- [ ] **Step 3: Run an end-to-end smoke test**

Open 3 terminals:

Terminal 1 (mock PICO):
```bash
python3 /home/eai/project/pico_project/pico_reciver_data/scripts/mock_pico_server.py --fps 30 --duration 30
```

Terminal 2 (bridge + recorder):
```bash
cd /home/eai/project/pico_project/ws
source install/setup.bash
ros2 launch pico_recorder start_pico_recorder.launch.py data_dir:=/tmp/pico_smoke
# In this terminal, type 'a' then wait ~3s then type 's'
```

Terminal 3 (sanity-check the output):
```bash
ls -la /tmp/pico_smoke/hdf5/
python3 -c "
import glob, h5py
f = sorted(glob.glob('/tmp/pico_smoke/hdf5/*.hdf5'))[-1]
print('file:', f)
with h5py.File(f, 'r') as h:
    def show(name, obj):
        shape = getattr(obj, 'shape', None)
        print(f'  {name} {shape}')
    h.visititems(show)
    print('attrs:', dict(h.attrs))
"
```

Expected: file exists, `/states/pose_head/pos` has shape `(~90, 3)` (3s @ 30Hz), `/observations/images/cam_left/jpeg` has shape `(~90,)`, and `/observations/images/hand_left/esp32_ts` exists.

- [ ] **Step 4: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add scripts/mock_pico_server.py
git commit -m "test: add mock PICO server for integration smoke test"
```

---

## Task 13: README

**Files:**
- Create: `README.md`

- [ ] **Step 1: Write README**

```markdown
# pico_recorder

Self-contained ROS2 ament_cmake package for recording PICO 4U data streams to HDF5.

## What it does
- `pico_bridge_node`: TCP client for `PicoStreamingServer`, publishes 7 ROS2 topics.
- `pico_recorder_node`: subscribes to those topics, records HDF5 on demand.

## Prerequisites
- ROS2 (tested on Humble/Iron/Jazzy).
- `libhdf5-dev`, `libeigen3-dev`, `python3-h5py` (for post-hoc inspection).
- `adb` on PATH.

## Build (method 2 — mini workspace)

```bash
mkdir -p ~/project/pico_project/ws/src
ln -s ~/project/pico_project/pico_reciver_data ~/project/pico_project/ws/src/pico_recorder
cd ~/project/pico_project/ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select pico_recorder
```

## Run

```bash
# Terminal 1 — adb port forward
adb forward tcp:9999 tcp:9999

# Terminal 2 — launch
cd ~/project/pico_project/ws
source install/setup.bash
ros2 launch pico_recorder start_pico_recorder.launch.py data_dir:=/tmp/pico
```

In the recorder terminal, press **`a`** to start, **`s`** to stop and save, **`q`** to quit.
Or drive it with services:

```bash
ros2 service call /pico/start_collect std_srvs/srv/Trigger
ros2 service call /pico/stop_collect  std_srvs/srv/Trigger
```

## Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/pico/cam_left/compressed`  | `sensor_msgs/CompressedImage` | JPEG passthrough |
| `/pico/cam_right/compressed` | `sensor_msgs/CompressedImage` | JPEG passthrough |
| `/pico/pose/head`            | `geometry_msgs/PoseStamped`   | 6DOF, `header.stamp` encodes PICO `ts_ms` |
| `/pico/pose/left_hand`       | `geometry_msgs/PoseStamped`   | |
| `/pico/pose/right_hand`      | `geometry_msgs/PoseStamped`   | |
| `/pico/ble/left`             | `pico_recorder/msg/BleFrame`  | `esp32_ts` + raw sensor bytes |
| `/pico/ble/right`            | `pico_recorder/msg/BleFrame`  | |

## HDF5 layout

See `docs/superpowers/specs/2026-04-16-pico-recorder-design.md` section 4.

## Testing without a real PICO

```bash
python3 scripts/mock_pico_server.py --fps 30 --duration 60
```

Then run the launch command above in another terminal.

## References
- `PICO_Streaming_Guide.md` — wire protocol reference from the Unity side.
- `receiver.py` — original Python reference receiver.
```

- [ ] **Step 2: Commit**

```bash
cd /home/eai/project/pico_project/pico_reciver_data
git add README.md
git commit -m "docs: add README"
```

---

## Self-Review Summary

**Spec coverage (mapped to tasks):**

| Spec section | Task(s) |
|--------------|---------|
| §2 Architecture (two-node split) | T9, T10 |
| §3.1 pico_bridge_node | T9 |
| §3.2 data_collector simplified | T4–T8 |
| §3.3 pico_recorder_node | T10 |
| §3.4 trigger interface (keyboard + service) | T10 |
| §3.5 launch file | T11 |
| §4 HDF5 layout (per-stream subgroups, ts_ms, esp32_ts) | T8 |
| §5 timestamp semantics | T9 (bridge stamp encode), T10 (recorder decode), T8 (save) |
| §6 file layout | all tasks |
| §7 build/run | T1, T11, T13 |
| §8 testing strategy | T3 (unit), T5–T8 (unit), T12 (integration) |
| `BleFrame.msg` | T2 |

**Placeholder scan:** No TBDs, no "add error handling", every code block complete. ✓

**Type consistency:** `FrameData`/`ImageFrameData` fields declared in T4 header match T5–T8 usages. `collected_ts_ms_` / `collected_esp32_ts_` / `collected_recv_ts_` all declared in T4 and populated in T7 `collect_frame`, consumed in T8 `save_hdf5`. `cpp_typesupport_target` declared in T9 and reused in T10. ✓

**Scope:** Single package, single spec, testable end-to-end. ✓

---

Plan complete and saved to `/home/eai/project/pico_project/pico_reciver_data/docs/superpowers/plans/2026-04-16-pico-recorder.md`.
