/*
 * Wuji SDK — C API
 *
 * THIS FILE IS AUTO-GENERATED. DO NOT EDIT.
 */

#ifndef WUJI_SDK_H
#define WUJI_SDK_H

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define WUJI_HAND_2_JOINT_COUNT 20
#define WUJI_JOINT_ONLINE(mask, i) (((mask) >> (i)) & 1u)


/**
 * Use in `WujiConnectTarget.kind` to connect by serial number.
 * `value` must point to a NUL-terminated serial-number string.
 */
#define WUJI_CONNECT_TARGET_KIND_SN 0

/**
 * Use in `WujiConnectTarget.kind` to connect by network address.
 * `value` must point to a NUL-terminated address string.
 */
#define WUJI_CONNECT_TARGET_KIND_ADDR 1

/**
 * Device type, known at scan time. Held by `WujiDiscovered.device_id`.
 *
 * An unrecognized or future device type maps to `Unknown`.
 */
enum WujiDeviceType
#ifdef __cplusplus
  : uint8_t
#endif // __cplusplus
 {
  /**
   * Unknown, or not recognized by this SDK build.
   */
  WUJI_DEVICE_TYPE_UNKNOWN = 0,
  /**
   * Wuji Glove tactile glove.
   */
  WUJI_DEVICE_TYPE_WUJI_GLOVE = 1,
  /**
   * Wuji Hand 2 dexterous hand.
   */
  WUJI_DEVICE_TYPE_WUJI_HAND_2 = 2,
  /**
   * Wuji Hand (first generation).
   */
  WUJI_DEVICE_TYPE_WUJI_HAND = 3,
};
#ifndef __cplusplus
typedef uint8_t WujiDeviceType;
#endif // __cplusplus

enum WujiFrameKind
#ifdef __cplusplus
  : uint8_t
#endif // __cplusplus
 {
  WUJI_FRAME_KIND_OK = 0,
  WUJI_FRAME_KIND_LAG = 1,
  WUJI_FRAME_KIND_END = 2,
  WUJI_FRAME_KIND_ERROR = 3,
};
#ifndef __cplusplus
typedef uint8_t WujiFrameKind;
#endif // __cplusplus

enum WujiGloveCalibrationState
#ifdef __cplusplus
  : uint8_t
#endif // __cplusplus
 {
  WUJI_GLOVE_CALIBRATION_STATE_UNKNOWN = 0,
  WUJI_GLOVE_CALIBRATION_STATE_WAITING_MOVEMENT = 1,
  WUJI_GLOVE_CALIBRATION_STATE_WAITING_STABLE = 2,
  WUJI_GLOVE_CALIBRATION_STATE_COLLECTING = 3,
  WUJI_GLOVE_CALIBRATION_STATE_DONE = 4,
};
#ifndef __cplusplus
typedef uint8_t WujiGloveCalibrationState;
#endif // __cplusplus

/**
 * Which Wuji hand model the qpos targets. Mirrors the SDK `HandModel`.
 *
 * `wuji_retarget_session_create` takes the discriminant as `int32_t` and
 * validates it — an out-of-range value from C would be UB as a Rust enum.
 * The enum exists to provide the `WUJI_HAND_MODEL_*` constants
 * (force-included via cbindgen.toml).
 */
enum WujiHandModel
#ifdef __cplusplus
  : int32_t
#endif // __cplusplus
 {
  WUJI_HAND_MODEL_WUJI_HAND = 0,
  WUJI_HAND_MODEL_WUJI_HAND2 = 1,
};
#ifndef __cplusplus
typedef int32_t WujiHandModel;
#endif // __cplusplus

/**
 * Handedness used by Wuji device APIs.
 */
typedef enum WujiHandedness {
  WUJI_HANDEDNESS_LEFT = 0,
  WUJI_HANDEDNESS_RIGHT = 1,
} WujiHandedness;

enum WujiResourceScope
#ifdef __cplusplus
  : uint8_t
#endif // __cplusplus
 {
  WUJI_RESOURCE_SCOPE_DEVICE = 0,
  WUJI_RESOURCE_SCOPE_GLOBAL = 1,
};
#ifndef __cplusplus
typedef uint8_t WujiResourceScope;
#endif // __cplusplus

enum WujiStatus
#ifdef __cplusplus
  : int32_t
#endif // __cplusplus
 {
  WUJI_STATUS_OK = 0,
  WUJI_STATUS_ERR_INVALID_ARG = -1,
  WUJI_STATUS_ERR_NOT_INITIALIZED = -2,
  WUJI_STATUS_ERR_NOT_FOUND = -3,
  WUJI_STATUS_ERR_TIMEOUT = -4,
  WUJI_STATUS_ERR_PROTOCOL = -5,
  WUJI_STATUS_ERR_DISCONNECTED = -6,
  WUJI_STATUS_ERR_BUFFER_TOO_SMALL = -7,
  /**
   * Operation not supported by this build, platform, or device.
   */
  WUJI_STATUS_ERR_UNSUPPORTED = -8,
  WUJI_STATUS_ERR_CANCELLED = -9,
  /**
   * Input data is present but failed validation (e.g. a corrupt or malformed
   * user-data bundle: bad zip, unparseable/wrong-`kind` manifest, sha256
   * mismatch, unsafe zip entry path, or size limit exceeded).
   */
  WUJI_STATUS_ERR_INVALID_DATA = -10,
  /**
   * An algorithm reported an error; call `wuji_last_error()` for details.
   */
  WUJI_STATUS_ERR_ALGORITHM = -11,
  WUJI_STATUS_ERR_INTERNAL = -99,
};
#ifndef __cplusplus
typedef int32_t WujiStatus;
#endif // __cplusplus

/**
 * One-shot calibration state.
 */
typedef enum WujiTactileCalibrationState {
  WUJI_TACTILE_CALIBRATION_STATE_COLLECT = 0,
  WUJI_TACTILE_CALIBRATION_STATE_POSE_OK = 1,
  WUJI_TACTILE_CALIBRATION_STATE_CHECK = 2,
  WUJI_TACTILE_CALIBRATION_STATE_TRAIN = 3,
  WUJI_TACTILE_CALIBRATION_STATE_INSTALL = 4,
  WUJI_TACTILE_CALIBRATION_STATE_VERIFY = 5,
  WUJI_TACTILE_CALIBRATION_STATE_DONE = 6,
  WUJI_TACTILE_CALIBRATION_STATE_UNKNOWN = 255,
} WujiTactileCalibrationState;

/**
 * Feedback callback action.
 */
typedef enum WujiTactileCallbackAction {
  WUJI_TACTILE_CALLBACK_ACTION_CONTINUE = 0,
  WUJI_TACTILE_CALLBACK_ACTION_ABORT = 1,
} WujiTactileCallbackAction;

/**
 * Prompt request kind.
 */
typedef enum WujiTactilePromptKind {
  WUJI_TACTILE_PROMPT_KIND_POSE_READY = 0,
  WUJI_TACTILE_PROMPT_KIND_POSE_REVIEW = 1,
  WUJI_TACTILE_PROMPT_KIND_UNKNOWN = 255,
} WujiTactilePromptKind;

/**
 * Response from the C prompt callback.
 */
typedef enum WujiTactilePromptResponse {
  WUJI_TACTILE_PROMPT_RESPONSE_PROCEED = 0,
  WUJI_TACTILE_PROMPT_RESPONSE_RETRY = 1,
  WUJI_TACTILE_PROMPT_RESPONSE_ABORT = 2,
} WujiTactilePromptResponse;

/**
 * Tactile runtime state reported by `wuji_hand_2_tactile_status`.
 *
 * Mirrors the firmware wire-protocol `TactileState`: `Calibrating` while a
 * runtime calibration is in progress, `Ready` once complete.
 */
typedef enum WujiTactileState {
  WUJI_TACTILE_STATE_READY = 0,
  WUJI_TACTILE_STATE_CALIBRATING = 1,
} WujiTactileState;

/**
 * Tactile sensor model reported by `wuji_hand_2_tactile_status`.
 *
 * Mirrors the firmware wire-protocol `TactileType`: `Standard` is a normal
 * fingertip sensor, `Thumb` a 40-point thumb node.
 */
typedef enum WujiTactileType {
  WUJI_TACTILE_TYPE_STANDARD = 0,
  WUJI_TACTILE_TYPE_THUMB = 1,
} WujiTactileType;

enum WujiTransportType
#ifdef __cplusplus
  : uint8_t
#endif // __cplusplus
 {
  WUJI_TRANSPORT_TYPE_UDP = 0,
  WUJI_TRANSPORT_TYPE_USB = 1,
  WUJI_TRANSPORT_TYPE_ZENOH = 2,
};
#ifndef __cplusplus
typedef uint8_t WujiTransportType;
#endif // __cplusplus

typedef struct WujiRetargetSession WujiRetargetSession;

typedef struct WujiDevice WujiDevice;

typedef struct WujiGloveCalibrationSession WujiGloveCalibrationSession;

typedef struct WujiHandJointCommandPublisher WujiHandJointCommandPublisher;

typedef struct WujiJointCommandPublisher WujiJointCommandPublisher;

typedef struct WujiPub WujiPub;

typedef struct WujiRealtimeController WujiRealtimeController;

typedef struct WujiSub WujiSub;

/**
 * Common frame header for timestamped data
 */
typedef struct WujiFrameHeader {
  uint32_t seq;
  uint64_t timestamp_us;
  /**
   * NUL-terminated UTF-8 string. At most 31 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 31 bytes.
   */
  char frame_id[32];
} WujiFrameHeader;

/**
 * Quaternion (f32 precision). Used by `WujiPose` and `WujiFrameTransform`.
 */
typedef struct WujiQuaternionF32 {
  float x;
  float y;
  float z;
  float w;
} WujiQuaternionF32;

/**
 * Position and orientation in 3D space
 */
typedef struct WujiPose {
  float position[3];
  struct WujiQuaternionF32 orientation;
} WujiPose;

/**
 * EMF-derived pose with confidence score
 */
typedef struct WujiEmfPose {
  struct WujiPose pose;
  float confidence;
} WujiEmfPose;

/**
 * Array of EMF poses
 */
typedef struct WujiEmfPoseArray {
  struct WujiFrameHeader header;
  struct WujiEmfPose *poses;
  size_t poses_len;
} WujiEmfPoseArray;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiEmfPoseArray`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiEmfPoseArraySubCallback)(WujiFrameKind kind,
                                            const struct WujiEmfPoseArray *frame,
                                            void *user_data);

/**
 * Fingertip pose with confidence
 */
typedef struct WujiFingertipPose {
  struct WujiPose pose;
  float confidence;
} WujiFingertipPose;

/**
 * Array of fingertip poses
 */
typedef struct WujiFingertipPoses {
  struct WujiFrameHeader header;
  struct WujiFingertipPose *poses;
  size_t poses_len;
} WujiFingertipPoses;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiFingertipPoses`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiFingertipPosesSubCallback)(WujiFrameKind kind,
                                              const struct WujiFingertipPoses *frame,
                                              void *user_data);

/**
 * Fingertip sensor data stream (pure value payload, interpreted per info.format)
 */
typedef struct WujiFingertipSensorData {
  struct WujiFrameHeader header;
  uint32_t info_digest;
  uint8_t *data;
  size_t data_len;
} WujiFingertipSensorData;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiFingertipSensorData`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiFingertipSensorDataSubCallback)(WujiFrameKind kind,
                                                   const struct WujiFingertipSensorData *frame,
                                                   void *user_data);

/**
 * A single coordinate frame transform
 */
typedef struct WujiFrameTransform {
  uint64_t timestamp_us;
  /**
   * NUL-terminated UTF-8 string. At most 31 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 31 bytes.
   */
  char parent_frame_id[32];
  /**
   * NUL-terminated UTF-8 string. At most 31 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 31 bytes.
   */
  char child_frame_id[32];
  float translation[3];
  struct WujiQuaternionF32 rotation;
} WujiFrameTransform;

/**
 * Collection of coordinate frame transforms
 */
typedef struct WujiFrameTransforms {
  struct WujiFrameTransform *transforms;
  size_t transforms_len;
} WujiFrameTransforms;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiFrameTransforms`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiFrameTransformsSubCallback)(WujiFrameKind kind,
                                               const struct WujiFrameTransforms *frame,
                                               void *user_data);

/**
 * Joint angles for a single finger
 */
typedef struct WujiFingerJointAngles {
  double angles[5];
  double confidence;
} WujiFingerJointAngles;

/**
 * Joint angles for all fingers
 */
typedef struct WujiHandJointAngles {
  struct WujiFrameHeader header;
  struct WujiFingerJointAngles *fingers;
  size_t fingers_len;
} WujiHandJointAngles;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiHandJointAngles`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiHandJointAnglesSubCallback)(WujiFrameKind kind,
                                               const struct WujiHandJointAngles *frame,
                                               void *user_data);

/**
 * ROS sensor_msgs/JointState-style joint state; position always present, velocity / effort optional (length 0 = channel not provided). Joint order is finger-major ({left,right}_finger{1..5}_joint{1..4}).
 */
typedef struct WujiHandJointStates {
  struct WujiFrameHeader header;
  double *position;
  size_t position_len;
  double *velocity;
  size_t velocity_len;
  double *effort;
  size_t effort_len;
} WujiHandJointStates;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiHandJointStates`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiHandJointStatesSubCallback)(WujiFrameKind kind,
                                               const struct WujiHandJointStates *frame,
                                               void *user_data);

/**
 * A single skeleton joint with pose and confidence
 */
typedef struct WujiSkeletonJoint {
  /**
   * NUL-terminated UTF-8 string. At most 31 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 31 bytes.
   */
  char name[32];
  struct WujiPose pose;
  float confidence;
} WujiSkeletonJoint;

/**
 * Hand skeleton with MediaPipe landmark joints
 */
typedef struct WujiHandSkeleton {
  struct WujiFrameHeader header;
  struct WujiSkeletonJoint *joints;
  size_t joints_len;
} WujiHandSkeleton;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiHandSkeleton`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiHandSkeletonSubCallback)(WujiFrameKind kind,
                                            const struct WujiHandSkeleton *frame,
                                            void *user_data);

/**
 * Quaternion (f64 precision). Used by `WujiImuData`.
 */
typedef struct WujiQuaternionF64 {
  double x;
  double y;
  double z;
  double w;
} WujiQuaternionF64;

/**
 * 3D vector with f64 components
 */
typedef struct WujiVector3F64 {
  double x;
  double y;
  double z;
} WujiVector3F64;

/**
 * IMU sensor data with orientation, angular velocity, and linear acceleration
 */
typedef struct WujiImuData {
  struct WujiFrameHeader header;
  struct WujiQuaternionF64 orientation;
  double orientation_covariance[9];
  struct WujiVector3F64 angular_velocity;
  double angular_velocity_covariance[9];
  struct WujiVector3F64 linear_acceleration;
  double linear_acceleration_covariance[9];
} WujiImuData;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiImuData`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiImuDataSubCallback)(WujiFrameKind kind,
                                       const struct WujiImuData *frame,
                                       void *user_data);

/**
 * Single joint diagnostics (status word / current / bus voltage / temperature / error code / bus comm quality). error_code_current is this joint's active device fault code (u16, 0 = no active error); pass it to describe_error for name/severity/cause/resolution.
 */
typedef struct WujiJointDiagnosticsEntry {
  uint8_t nid;
  uint32_t status_word;
  float current;
  float vbus_v_fb;
  float mcu_temp_c_fb;
  uint16_t error_code_current;
  uint8_t comm_response_rate_pct;
  uint32_t comm_timeout_total;
} WujiJointDiagnosticsEntry;

/**
 * Frame-level communication summary — SDK-local E2E stream/RPC stats + firmware comm_diag snapshot digest.
 */
typedef struct WujiHand2CommSummary {
  uint16_t age_ms;
  uint8_t tactile_online_mask;
  uint32_t e2e_received;
  uint32_t e2e_lost;
  uint16_t e2e_reordered;
  uint16_t e2e_duplicates;
  uint16_t e2e_window_loss_x100;
  uint32_t rpc_total;
  uint16_t rpc_retries;
  uint16_t rpc_timeouts;
  uint16_t comm_get_failures;
  uint8_t tactile_response_rate_pct[5];
  uint32_t tactile_timeout_total[5];
  uint32_t sdk_dropped;
} WujiHand2CommSummary;

/**
 * Whole-hand joint diagnostics frame; variable-length, online joints only (look up by `nid`). Carries a frame-level communication summary.
 */
typedef struct WujiJointDiagnosticsFrame {
  struct WujiFrameHeader header;
  uint8_t num_joints;
  struct WujiJointDiagnosticsEntry *joints;
  size_t joints_len;
  struct WujiHand2CommSummary comm;
} WujiJointDiagnosticsFrame;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiJointDiagnosticsFrame`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiJointDiagnosticsFrameSubCallback)(WujiFrameKind kind,
                                                     const struct WujiJointDiagnosticsFrame *frame,
                                                     void *user_data);

/**
 * Single joint state (position / velocity / effort).
 */
typedef struct WujiJointStateEntry {
  uint8_t nid;
  float position;
  float velocity;
  float effort;
} WujiJointStateEntry;

/**
 * Whole-hand joint state frame (position/velocity/effort per joint); variable-length, online joints only (look up by `nid`).
 */
typedef struct WujiJointStateFrame {
  struct WujiFrameHeader header;
  uint8_t num_joints;
  struct WujiJointStateEntry *joints;
  size_t joints_len;
} WujiJointStateFrame;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiJointStateFrame`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiJointStateFrameSubCallback)(WujiFrameKind kind,
                                               const struct WujiJointStateFrame *frame,
                                               void *user_data);

/**
 * Description of a single field in a point cloud
 */
typedef struct WujiPointField {
  /**
   * NUL-terminated UTF-8 string. At most 31 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 31 bytes.
   */
  char name[32];
  uint32_t offset;
  uint8_t type_;
} WujiPointField;

/**
 * Point cloud data
 */
typedef struct WujiPointCloud {
  struct WujiFrameHeader header;
  /**
   * NUL-terminated UTF-8 string. At most 63 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 63 bytes.
   */
  char frame_id[64];
  uint32_t point_stride;
  struct WujiPointField *fields;
  size_t fields_len;
  uint8_t *data;
  size_t data_len;
} WujiPointCloud;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiPointCloud`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiPointCloudSubCallback)(WujiFrameKind kind,
                                          const struct WujiPointCloud *frame,
                                          void *user_data);

/**
 * NC baseline binary contact detection output (24x31 row-major; 1.0=contact, 0.0=no contact, -1.0=invalid/masked taxel)
 */
typedef struct WujiTactileBinary {
  struct WujiFrameHeader header;
  float *data;
  size_t data_len;
} WujiTactileBinary;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiTactileBinary`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiTactileBinarySubCallback)(WujiFrameKind kind,
                                             const struct WujiTactileBinary *frame,
                                             void *user_data);

/**
 * Processed tactile sensor frame
 */
typedef struct WujiTactileFrame {
  struct WujiFrameHeader header;
  float *data;
  size_t data_len;
} WujiTactileFrame;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiTactileFrame`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiTactileFrameSubCallback)(WujiFrameKind kind,
                                            const struct WujiTactileFrame *frame,
                                            void *user_data);

/**
 * Single tactile pressure frame (24x32 f32, flat ~3 KB)
 */
typedef struct WujiTactileGloveFrame {
  uint8_t handedness;
  uint32_t sequence;
  uint32_t timestamp_ms;
  float *pressure;
  size_t pressure_len;
} WujiTactileGloveFrame;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiTactileGloveFrame`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiTactileGloveFrameSubCallback)(WujiFrameKind kind,
                                                 const struct WujiTactileGloveFrame *frame,
                                                 void *user_data);

/**
 * Health/availability of the paired tactile glove sub-module
 */
typedef struct WujiTactileGloveStatus {
  uint8_t state;
  uint32_t last_frame_age_ms;
} WujiTactileGloveStatus;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiTactileGloveStatus`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiTactileGloveStatusSubCallback)(WujiFrameKind kind,
                                                  const struct WujiTactileGloveStatus *frame,
                                                  void *user_data);

/**
 * NC baseline continuous contact signal (24x31 row-major signed residual actual-μ-bias; -1.0=invalid/masked taxel)
 */
typedef struct WujiTactileResidual {
  struct WujiFrameHeader header;
  float *data;
  size_t data_len;
} WujiTactileResidual;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiTactileResidual`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiTactileResidualSubCallback)(WujiFrameKind kind,
                                               const struct WujiTactileResidual *frame,
                                               void *user_data);

/**
 * Tactile data grouped by finger zones
 */
typedef struct WujiTactileZones {
  struct WujiFrameHeader header;
  float *palm;
  size_t palm_len;
  float *thumb;
  size_t thumb_len;
  float *index;
  size_t index_len;
  float *middle;
  size_t middle_len;
  float *ring;
  size_t ring_len;
  float *pinky;
  size_t pinky_len;
} WujiTactileZones;

/**
 * Typed callback for the generated typed-subscribe wrappers that yield `WujiTactileZones`.
 *
 * `frame` is non-NULL only when `kind == WUJI_FRAME_KIND_OK`; it points to a
 * stack-allocated struct valid only for the duration of this call (the wrapper
 * frees its heap fields immediately after this fn returns).
 */
typedef void (*WujiTactileZonesSubCallback)(WujiFrameKind kind,
                                            const struct WujiTactileZones *frame,
                                            void *user_data);

/**
 * Hardware version (Major.Minor.Patch). All-zero = unprovisioned at factory.
 */
typedef struct WujiHwVersion {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
} WujiHwVersion;

/**
 * Tactile glove diagnostics snapshot
 */
typedef struct WujiTactileGloveDiagnostics {
  uint32_t uptime_ms;
  uint64_t frame_count;
  uint32_t crc_err_count;
  uint32_t dropout_count;
  uint32_t usb_reset_count;
} WujiTactileGloveDiagnostics;

/**
 * Tactile glove identity (TBIM SN, hw/fw revs)
 */
typedef struct WujiTactileGloveDeviceInfo {
  /**
   * NUL-terminated UTF-8 string. At most 31 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 31 bytes.
   */
  char serial[32];
  uint8_t hw_revision[4];
  uint8_t fw_version[4];
} WujiTactileGloveDeviceInfo;

/**
 * Per transfer-type stats
 */
typedef struct WujiTransferStats {
  uint32_t tx_total;
  uint32_t timeout_total;
  uint16_t tx_per_sec;
  uint8_t timeout_rate_pct;
} WujiTransferStats;

/**
 * Per-node diagnostics
 */
typedef struct WujiNodeDiagnostics {
  uint8_t node_type;
  uint8_t online;
  uint32_t ms_since_last_response;
  uint32_t request_total;
  uint32_t response_ok_total;
  uint32_t timeout_total;
  uint8_t response_rate_pct;
  uint16_t request_per_sec;
} WujiNodeDiagnostics;

/**
 * Per-finger L2/L3/per-node diagnostics
 */
typedef struct WujiFingerCommunicationDiagnostics {
  uint32_t tx_frame_total;
  uint32_t rx_frame_total;
  uint32_t tx_byte_total;
  uint32_t rx_byte_total;
  uint32_t crc_error_total;
  uint32_t frame_format_error_total;
  uint32_t uart_hw_error_total;
  uint16_t tx_frame_per_sec;
  uint16_t rx_frame_per_sec;
  uint32_t tx_kbps;
  uint32_t rx_kbps;
  uint16_t error_per_sec;
  struct WujiTransferStats transfer_stats[5];
  struct WujiNodeDiagnostics nodes[5];
} WujiFingerCommunicationDiagnostics;

/**
 * Whole-hand 1Hz aggregated diagnostics across 5 fingers
 */
typedef struct WujiHandCommunicationDiagnostics {
  struct WujiFingerCommunicationDiagnostics fingers[5];
} WujiHandCommunicationDiagnostics;

typedef struct WujiInitOptions {
  int32_t log_level;
} WujiInitOptions;

typedef struct WujiParam {
  const char *path;
  uint8_t readable;
  uint8_t writable;
} WujiParam;

typedef struct WujiTopic {
  const char *path;
  WujiResourceScope scope;
} WujiTopic;

/**
 * Result of a single time-sync round-trip. Plain data — no heap fields, so no
 * `*_free` is needed.
 */
typedef struct WujiTimeSyncResult {
  /**
   * Absolute clock offset in microseconds. `device_time = uptime_us + offset_us`.
   */
  int64_t offset_us;
  /**
   * Round-trip time in microseconds. Smaller is more precise.
   */
  int64_t round_trip_us;
  /**
   * UTC microseconds at the moment of measurement (NTP T4).
   */
  uint64_t synced_at_us;
} WujiTimeSyncResult;

typedef struct WujiGloveCalibrationOptions {
  bool skip_constraints;
  double timeout_s;
} WujiGloveCalibrationOptions;

typedef struct WujiGloveCalibrationMetric {
  bool has_label;
  char *label;
  bool has_finger;
  char *finger;
  bool has_finger_b;
  char *finger_b;
  bool has_value;
  double value;
  bool has_min;
  double min;
  bool has_max;
  double max;
  bool has_error;
  double error;
  bool has_unit;
  char *unit;
  bool has_hint;
  char *hint;
} WujiGloveCalibrationMetric;

typedef struct WujiGloveCalibrationFeedback {
  bool has_step_index;
  uint32_t step_index;
  bool has_step_total;
  uint32_t step_total;
  bool has_step_name;
  char *step_name;
  WujiGloveCalibrationState state;
  double progress;
  bool has_hold_elapsed;
  double hold_elapsed;
  bool has_hold_target;
  double hold_target;
  bool has_collect_elapsed;
  double collect_elapsed;
  bool has_collect_target;
  double collect_target;
  bool has_frames_collected;
  uint32_t frames_collected;
  bool has_variance_ok;
  bool variance_ok;
  bool has_constraints_ok;
  bool constraints_ok;
  bool has_variance;
  double variance;
  bool has_variance_target;
  double variance_target;
  struct WujiGloveCalibrationMetric *metrics;
  size_t metrics_len;
  char **hints;
  size_t hints_len;
} WujiGloveCalibrationFeedback;

/**
 * @brief WUJI GLOVE IK calibration feedback callback.
 *
 * @note `feedback` is borrowed and valid only for the duration of the callback.
 * Copy any fields that must outlive the call.
 * @note The callback runs on the calibration worker thread. Do not call
 * `wuji_glove_calibration_wait`, `wuji_glove_calibration_try_finish`, or
 * `wuji_glove_calibration_session_free` for the same session from inside this
 * callback. `wuji_glove_calibration_cancel` is non-blocking and may be called
 * from the callback.
 */
typedef void (*WujiGloveCalibrationFeedbackCallback)(const struct WujiGloveCalibrationFeedback*,
                                                     void*);

typedef struct WujiFrameCount {
  char *pose_name;
  uint32_t frames;
} WujiFrameCount;

typedef struct WujiUserInfo {
  char *user_id;
  char *display_name;
  char *description;
  char *external_id;
  bool is_default;
  char *created_at;
  char *updated_at;
} WujiUserInfo;

typedef struct WujiGloveCalibrationResult {
  uint32_t poses_collected;
  struct WujiFrameCount *frames_per_pose;
  size_t frames_per_pose_len;
  enum WujiHandedness handedness;
  /**
   * SDK-allocated local filesystem path. Treat it as an opaque result value;
   * do not infer or depend on its parent-directory layout.
   */
  char *calibrated_urdf;
  struct WujiUserInfo sdk_user;
} WujiGloveCalibrationResult;

/**
 * Per-joint diagnostics snapshot (mirrors Python `WujiHandJointDiagnostics`).
 */
typedef struct WujiHandJointDiagnostics {
  /**
   * CAN bus voltage at this joint (volts).
   */
  float bus_voltage;
  /**
   * Joint controller temperature (°C).
   */
  float temperature;
  /**
   * Firmware error-code **bitfield** (one bit per fault condition). 0 = no fault.
   *
   * This is a first-generation-hand encoding of its own — unrelated to the
   * fault codes carried by `WujiJointDiagnosticsEntry::error_code_current`,
   * and not decodable by `wuji_hand_2_describe_error`.
   */
  uint32_t error_code;
} WujiHandJointDiagnostics;

/**
 * Structured description of one joint fault code, mirroring the
 * firmware-authored catalog exposed by Python's `WujiHand2.describe_error`.
 *
 * All string fields are NUL-terminated, fixed-size and UTF-8 (the firmware
 * catalog text is Chinese). Buffers are sized to fit the current catalog with
 * headroom; a future entry exceeding a buffer is left empty rather than
 * truncated.
 */
typedef struct WujiErrorInfo {
  /**
   * The raw 16-bit fault code, as reported by the device in
   * `error_code_current`.
   *
   * Firmware lays the hex digits out as `0xSCNN`: `S` encodes how the joint
   * reacts (warning / deferred stop / immediate stop / fatal), `C` how the
   * fault clears (auto / manual / needs a reset), and `NN` is a two-digit
   * index within that group. Do not decode the digits yourself — read
   * `severity` and `clear_policy` below, which carry the same information as
   * plain strings and stay correct if the layout ever changes. Treat the code
   * itself as an opaque identifier: log it, show it, quote it in a report.
   */
  uint16_t code;
  /**
   * Test-injection-only code (excluded from user-facing docs).
   */
  bool is_test;
  /**
   * Symbolic name, e.g. "Overcurrent".
   */
  char name[64];
  /**
   * "Warning" | "DeferredStop" | "ImmediateStop" | "Fatal".
   */
  char severity[24];
  /**
   * "AutoClear" | "ManualClear" | "NonClearable".
   */
  char clear_policy[24];
  /**
   * Short human-readable description.
   */
  char desc[160];
  /**
   * Likely cause.
   */
  char cause[320];
  /**
   * Suggested resolution.
   */
  char resolution[256];
} WujiErrorInfo;

/**
 * Fingertip sensor metadata (one per link; format JSON describes data payload layout)
 */
typedef struct WujiFingertipSensorInfo {
  struct WujiFrameHeader header;
  uint32_t digest;
  /**
   * NUL-terminated UTF-8 string. At most 31 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 31 bytes.
   */
  char model[32];
  uint16_t device_type;
  float rate_hz;
  /**
   * NUL-terminated UTF-8 string. At most 2047 content bytes; the last slot is always the NUL terminator. Truncated if the source exceeds 2047 bytes.
   */
  char format[2048];
} WujiFingertipSensorInfo;

/**
 * Single joint command — position + velocity + effort.
 */
typedef struct WujiJointCommand {
  float position;
  float velocity;
  float effort;
} WujiJointCommand;

/**
 * Low-pass filter config passed to `wuji_hand_realtime_controller_open`.
 */
typedef struct WujiLowPass {
  /**
   * Cutoff frequency in Hz.
   */
  double cutoff_hz;
} WujiLowPass;

typedef struct WujiConnectOptions {
  /**
   * Timeout in milliseconds (default 1000).
   */
  uint32_t timeout_ms;
  /**
   * Number of retries (default 3).
   */
  uint32_t retry_count;
  /**
   * Allow multiple SDK instances to connect to the same device (default true).
   */
  bool enable_bridge;
  /**
   * Background time-sync interval in milliseconds (default 30000). Used when
   * `auto_time_sync_interval_enabled` is true. Must be >= 100 when enabled.
   */
  uint64_t auto_time_sync_interval_ms;
  /**
   * Set false to disable the background task; equivalent to Python
   * `ConnectOptions.auto_time_sync_interval_ms=None`. The first sync inside
   * `connect()` still runs.
   */
  bool auto_time_sync_interval_enabled;
} WujiConnectOptions;

typedef struct WujiDiscovered {
  /**
   * NUL-terminated UTF-8 string. At most 63 content bytes; the last slot
   * is always the NUL terminator. Truncated if the source exceeds 63 bytes.
   */
  char serial_number[64];
  /**
   * Human-readable device type / model string: `"WujiGlove"`, `"WujiHand2"`,
   * `"WujiHand"`, or `"Unknown"`. NUL-terminated UTF-8; at most 31 content
   * bytes, the last slot is always the NUL terminator.
   */
  char model[32];
  /**
   * Device type — one of the `WujiDeviceType` values; compare against the
   * `WUJI_DEVICE_TYPE_*` constants. `WUJI_DEVICE_TYPE_UNKNOWN` if the type
   * was not reported at scan time.
   */
  WujiDeviceType device_id;
  WujiTransportType transport;
  /**
   * NUL-terminated UTF-8 string. At most 63 content bytes; the last slot
   * is always the NUL terminator. Truncated if the source exceeds 63 bytes.
   */
  char address[64];
} WujiDiscovered;

typedef struct WujiConnectTarget {
  /**
   * Must be one of the `WUJI_CONNECT_TARGET_KIND_*` constants
   * (`_SN` = 0, `_ADDR` = 1). Other values cause `wuji_connect` to
   * return `WUJI_STATUS_ERR_INVALID_ARG`.
   */
  uint8_t kind;
  const char *value;
} WujiConnectTarget;

/**
 * C-side callback signature.
 *
 * Lifetime contract:
 * - `data` is valid only for the duration of this call.
 * - `kind == WUJI_FRAME_KIND_END` or `WUJI_FRAME_KIND_ERROR` is terminal;
 *   no further callbacks will fire after one of these is delivered.
 * - `kind == WUJI_FRAME_KIND_LAG` carries `lag_count`; not terminal —
 *   the stream continues.
 * - `user_data` is the verbatim pointer passed to `wuji_sub_open*`.
 * - Passing a NULL `cb` to `wuji_sub_open*` returns
 *   `WUJI_STATUS_ERR_INVALID_ARG` without opening any subscription.
 *
 * Threading contract:
 * - Fires on a dedicated SDK worker thread — one per subscription, never the
 *   caller's thread.
 * - Calls for a single subscription are serialized and delivered in order (at
 *   most one in-flight); blocking here back-pressures that stream and may
 *   surface as `WUJI_FRAME_KIND_LAG`.
 * - Different subscriptions run on different threads and may fire
 *   concurrently — synchronize any state shared across their callbacks.
 * - Other (thread-safe) `wuji_*` calls are allowed from inside the callback,
 *   but never call `wuji_sub_close` on this same subscription from its own
 *   callback: the close path joins the worker thread and would self-deadlock.
 */
typedef void (*WujiSubCallback)(WujiFrameKind kind,
                                const uint8_t *data,
                                size_t len,
                                uint64_t timestamp_us,
                                uint64_t lag_count,
                                void *user_data);

/**
 * Options for blocking tactile calibration.
 */
typedef struct WujiTactileCalibrationOptions {
  /**
   * Recording duration per pose in seconds; must be > 0.
   */
  float seconds_per_pose;
  /**
   * Training epochs; must be >= 1.
   */
  uint32_t epochs;
  bool install;
  /**
   * If false, `sensitivity` is ignored.
   */
  bool has_sensitivity;
  double sensitivity;
  /**
   * Overall timeout in seconds; must be > 0.
   */
  double timeout_s;
} WujiTactileCalibrationOptions;

/**
 * Feedback emitted by blocking tactile calibration. Pointer fields are valid
 * only for the duration of the feedback callback.
 */
typedef struct WujiTactileCalibrationFeedback {
  enum WujiTactileCalibrationState state_kind;
  const char *state;
  uint32_t step_index;
  uint32_t step_total;
  const char *step_name;
  float collect_elapsed;
  float collect_target;
  uint32_t frames_collected;
  uint32_t epoch;
  uint32_t epoch_total;
  float best_val;
} WujiTactileCalibrationFeedback;

/**
 * Optional feedback callback. Return a `WujiTactileCallbackAction` value
 * (`0` continue, `1` abort) as `uint32_t`; any other value is treated as
 * abort. A fixed-width integer return keeps an out-of-range value from
 * becoming an invalid Rust enum discriminant.
 */
typedef uint32_t (*WujiTactileFeedbackCallback)(const struct WujiTactileCalibrationFeedback *event,
                                                void *user_data);

/**
 * Borrowed prompt request. Pointer fields are valid only for the duration of
 * the prompt callback.
 */
typedef struct WujiTactilePromptRequest {
  enum WujiTactilePromptKind kind;
  uint32_t step_index;
  uint32_t step_total;
  const char *step_name;
  float seconds_per_pose;
  uint32_t paired_frames;
  const char *const *warnings;
  size_t warnings_len;
} WujiTactilePromptRequest;

/**
 * Optional pose prompt callback for ready/review gates. Return a
 * `WujiTactilePromptResponse` value (`0` proceed, `1` retry, `2` abort) as
 * `uint32_t`; any other value is treated as abort.
 */
typedef uint32_t (*WujiTactilePosePromptCallback)(const struct WujiTactilePromptRequest *request,
                                                  void *user_data);

/**
 * Optional callbacks for blocking tactile calibration.
 */
typedef struct WujiTactileCalibrationCallbacks {
  WujiTactileFeedbackCallback on_feedback;
  void *feedback_userdata;
  WujiTactilePosePromptCallback on_pose_prompt;
  void *prompt_userdata;
} WujiTactileCalibrationCallbacks;

/**
 * Pose-name to frame-count pair in `WujiTactileCalibrationSummary`.
 */
typedef struct WujiTactilePoseFrames {
  char *pose;
  uint32_t frames;
} WujiTactilePoseFrames;

/**
 * Summary returned by tactile calibration. Nested arrays and strings are
 * SDK-allocated and released by `wuji_tactile_calibration_summary_free`.
 */
typedef struct WujiTactileCalibrationSummary {
  size_t poses_collected;
  struct WujiTactilePoseFrames *frames_per_pose;
  size_t frames_per_pose_len;
  uint32_t epochs;
  float best_val_loss;
  size_t alive_taxels;
  float dead_pct;
  bool installed;
  char *model_dir;
  bool has_verified_alive_taxels;
  size_t verified_alive_taxels;
} WujiTactileCalibrationSummary;

typedef struct WujiUserUpdate {
  const char *display_name;
  const char *description;
  const char *external_id;
  bool clear_description;
  bool clear_external_id;
} WujiUserUpdate;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * @brief Subscribe to the `emf_poses` stream (schema `WujiEmfPoseArray`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_emf_poses(struct WujiDevice *dev,
                                          WujiEmfPoseArraySubCallback cb,
                                          void *user_data,
                                          struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `tip_poses` stream (schema `WujiFingertipPoses`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_tip_poses(struct WujiDevice *dev,
                                          WujiFingertipPosesSubCallback cb,
                                          void *user_data,
                                          struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `fingertip_index_data` stream (schema `WujiFingertipSensorData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_2_subscribe_fingertip_index_data(struct WujiDevice *dev,
                                                      WujiFingertipSensorDataSubCallback cb,
                                                      void *user_data,
                                                      struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `fingertip_middle_data` stream (schema `WujiFingertipSensorData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_2_subscribe_fingertip_middle_data(struct WujiDevice *dev,
                                                       WujiFingertipSensorDataSubCallback cb,
                                                       void *user_data,
                                                       struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `fingertip_pinky_data` stream (schema `WujiFingertipSensorData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_2_subscribe_fingertip_pinky_data(struct WujiDevice *dev,
                                                      WujiFingertipSensorDataSubCallback cb,
                                                      void *user_data,
                                                      struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `fingertip_ring_data` stream (schema `WujiFingertipSensorData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_2_subscribe_fingertip_ring_data(struct WujiDevice *dev,
                                                     WujiFingertipSensorDataSubCallback cb,
                                                     void *user_data,
                                                     struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `fingertip_thumb_data` stream (schema `WujiFingertipSensorData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_2_subscribe_fingertip_thumb_data(struct WujiDevice *dev,
                                                      WujiFingertipSensorDataSubCallback cb,
                                                      void *user_data,
                                                      struct WujiSub **out_sub);

/**
 * @brief Subscribe to the global `tf` stream (schema `WujiFrameTransforms`).
 *
 * Carries cross-device merged data, so no device handle is required.
 *
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_subscribe_tf(WujiFrameTransformsSubCallback cb,
                             void *user_data,
                             struct WujiSub **out_sub);

/**
 * @brief Subscribe to the global `tf_static` stream (schema `WujiFrameTransforms`).
 *
 * Carries cross-device merged data, so no device handle is required.
 *
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_subscribe_tf_static(WujiFrameTransformsSubCallback cb,
                                    void *user_data,
                                    struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `hand_joint_angles` stream (schema `WujiHandJointAngles`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_hand_joint_angles(struct WujiDevice *dev,
                                                  WujiHandJointAnglesSubCallback cb,
                                                  void *user_data,
                                                  struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `joint_states` stream (schema `WujiHandJointStates`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_subscribe_joint_states(struct WujiDevice *dev,
                                            WujiHandJointStatesSubCallback cb,
                                            void *user_data,
                                            struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `hand_skeleton` stream (schema `WujiHandSkeleton`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_hand_skeleton(struct WujiDevice *dev,
                                              WujiHandSkeletonSubCallback cb,
                                              void *user_data,
                                              struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_data_index` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_data_index(struct WujiDevice *dev,
                                               WujiImuDataSubCallback cb,
                                               void *user_data,
                                               struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_data_middle` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_data_middle(struct WujiDevice *dev,
                                                WujiImuDataSubCallback cb,
                                                void *user_data,
                                                struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_data_palm` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_data_palm(struct WujiDevice *dev,
                                              WujiImuDataSubCallback cb,
                                              void *user_data,
                                              struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_data_pinky` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_data_pinky(struct WujiDevice *dev,
                                               WujiImuDataSubCallback cb,
                                               void *user_data,
                                               struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_data_ring` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_data_ring(struct WujiDevice *dev,
                                              WujiImuDataSubCallback cb,
                                              void *user_data,
                                              struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_data_thumb` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_data_thumb(struct WujiDevice *dev,
                                               WujiImuDataSubCallback cb,
                                               void *user_data,
                                               struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_index` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_index(struct WujiDevice *dev,
                                          WujiImuDataSubCallback cb,
                                          void *user_data,
                                          struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_middle` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_middle(struct WujiDevice *dev,
                                           WujiImuDataSubCallback cb,
                                           void *user_data,
                                           struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_palm` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_palm(struct WujiDevice *dev,
                                         WujiImuDataSubCallback cb,
                                         void *user_data,
                                         struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_pinky` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_pinky(struct WujiDevice *dev,
                                          WujiImuDataSubCallback cb,
                                          void *user_data,
                                          struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_ring` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_ring(struct WujiDevice *dev,
                                         WujiImuDataSubCallback cb,
                                         void *user_data,
                                         struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu_thumb` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_imu_thumb(struct WujiDevice *dev,
                                          WujiImuDataSubCallback cb,
                                          void *user_data,
                                          struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `imu` stream (schema `WujiImuData`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_2_subscribe_imu(struct WujiDevice *dev,
                                     WujiImuDataSubCallback cb,
                                     void *user_data,
                                     struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `joint_diagnostics` stream (schema `WujiJointDiagnosticsFrame`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_2_subscribe_joint_diagnostics(struct WujiDevice *dev,
                                                   WujiJointDiagnosticsFrameSubCallback cb,
                                                   void *user_data,
                                                   struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `joint_states` stream (schema `WujiJointStateFrame`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_2_subscribe_joint_states(struct WujiDevice *dev,
                                              WujiJointStateFrameSubCallback cb,
                                              void *user_data,
                                              struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `tactile_point_cloud` stream (schema `WujiPointCloud`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_tactile_point_cloud(struct WujiDevice *dev,
                                                    WujiPointCloudSubCallback cb,
                                                    void *user_data,
                                                    struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `tactile_binary` stream (schema `WujiTactileBinary`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_tactile_binary(struct WujiDevice *dev,
                                               WujiTactileBinarySubCallback cb,
                                               void *user_data,
                                               struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `tactile` stream (schema `WujiTactileFrame`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_tactile(struct WujiDevice *dev,
                                        WujiTactileFrameSubCallback cb,
                                        void *user_data,
                                        struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `tactile_pressure_frame` stream (schema `WujiTactileGloveFrame`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_subscribe_tactile_pressure_frame(struct WujiDevice *dev,
                                                      WujiTactileGloveFrameSubCallback cb,
                                                      void *user_data,
                                                      struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `tactile_status` stream (schema `WujiTactileGloveStatus`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_hand_subscribe_tactile_status(struct WujiDevice *dev,
                                              WujiTactileGloveStatusSubCallback cb,
                                              void *user_data,
                                              struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `tactile_residual` stream (schema `WujiTactileResidual`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_tactile_residual(struct WujiDevice *dev,
                                                 WujiTactileResidualSubCallback cb,
                                                 void *user_data,
                                                 struct WujiSub **out_sub);

/**
 * @brief Subscribe to the `tactile_zones` stream (schema `WujiTactileZones`).
 *
 * @param dev Valid device handle returned by `wuji_connect`.
 * @param cb Non-NULL typed callback invoked by the subscription worker.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status on invalid input / subscription failure.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_glove_subscribe_tactile_zones(struct WujiDevice *dev,
                                              WujiTactileZonesSubCallback cb,
                                              void *user_data,
                                              struct WujiSub **out_sub);

/**
 * @brief Free heap allocations owned by a `WujiHwVersion` and zero out its pointers.
 *
 * @param frame May be NULL, or must point to a `WujiHwVersion` produced by the matching decoder.
 * @note This function is idempotent and NULL-safe.
 */
void wuji_hw_version_free(struct WujiHwVersion *_frame);

/**
 * @brief Free heap allocations owned by a `WujiTactileGloveDiagnostics` and zero out its pointers.
 *
 * @param frame May be NULL, or must point to a `WujiTactileGloveDiagnostics` produced by the matching decoder.
 * @note This function is idempotent and NULL-safe.
 */
void wuji_tactile_glove_diagnostics_free(struct WujiTactileGloveDiagnostics *_frame);

/**
 * @brief Free heap allocations owned by a `WujiTactileGloveDeviceInfo` and zero out its pointers.
 *
 * @param frame May be NULL, or must point to a `WujiTactileGloveDeviceInfo` produced by the matching decoder.
 * @note This function is idempotent and NULL-safe.
 */
void wuji_tactile_glove_device_info_free(struct WujiTactileGloveDeviceInfo *_frame);

/**
 * @brief Free heap allocations owned by a `WujiHandCommunicationDiagnostics` and zero out its pointers.
 *
 * @param frame May be NULL, or must point to a `WujiHandCommunicationDiagnostics` produced by the matching decoder.
 * @note This function is idempotent and NULL-safe.
 */
void wuji_hand_communication_diagnostics_free(struct WujiHandCommunicationDiagnostics *frame);

/**
 * @brief Typed GET for `"serial_number"`.
 * @note Buffer-fill: writes a NUL-terminated string. Returns ErrBufferTooSmall (and sets *needed to the required size incl. NUL, if non-NULL) when buf is NULL or too small; never truncates. Two-call: pass buf=NULL,buf_len=0 to query *needed.
 */
WujiStatus wuji_glove_get_sn(struct WujiDevice *dev,
                             char *buf,
                             size_t buf_len,
                             size_t *needed);

/**
 * @brief Typed GET for `"firmware_version"`.
 * @note Buffer-fill: writes a NUL-terminated string. Returns ErrBufferTooSmall (and sets *needed to the required size incl. NUL, if non-NULL) when buf is NULL or too small; never truncates. Two-call: pass buf=NULL,buf_len=0 to query *needed.
 */
WujiStatus wuji_glove_get_version(struct WujiDevice *dev,
                                  char *buf,
                                  size_t buf_len,
                                  size_t *needed);

/**
 * @brief Typed GET for `"ip_address"`.
 * @note Buffer-fill: writes a NUL-terminated string. Returns ErrBufferTooSmall (and sets *needed to the required size incl. NUL, if non-NULL) when buf is NULL or too small; never truncates. Two-call: pass buf=NULL,buf_len=0 to query *needed.
 */
WujiStatus wuji_glove_get_ip(struct WujiDevice *dev,
                             char *buf,
                             size_t buf_len,
                             size_t *needed);

/**
 * @brief Typed GET for `"data_port"`.
 */
WujiStatus wuji_glove_get_port(struct WujiDevice *dev, uint16_t *out);

/**
 * @brief Typed GET for `"hand_side"`.
 * @note Buffer-fill: writes a NUL-terminated string. Returns ErrBufferTooSmall (and sets *needed to the required size incl. NUL, if non-NULL) when buf is NULL or too small; never truncates. Two-call: pass buf=NULL,buf_len=0 to query *needed.
 */
WujiStatus wuji_glove_get_hand_side(struct WujiDevice *dev,
                                    char *buf,
                                    size_t buf_len,
                                    size_t *needed);

/**
 * @brief Typed GET for `"input_voltage"`.
 */
WujiStatus wuji_hand_get_input_voltage(struct WujiDevice *dev, float *out);

/**
 * @brief Typed GET for `"temperature"`.
 */
WujiStatus wuji_hand_get_temperature(struct WujiDevice *dev, float *out);

/**
 * @brief Typed GET for `"handedness"`.
 */
WujiStatus wuji_hand_get_handedness(struct WujiDevice *dev, uint8_t *out);

/**
 * @brief Typed GET for `"firmware_version"`.
 */
WujiStatus wuji_hand_get_firmware_version(struct WujiDevice *dev, uint32_t *out);

/**
 * @brief Typed GET for `"product_sn"`.
 * @note Buffer-fill: writes a NUL-terminated string. Returns ErrBufferTooSmall (and sets *needed to the required size incl. NUL, if non-NULL) when buf is NULL or too small; never truncates. Two-call: pass buf=NULL,buf_len=0 to query *needed.
 */
WujiStatus wuji_hand_get_product_sn(struct WujiDevice *dev,
                                    char *buf,
                                    size_t buf_len,
                                    size_t *needed);

/**
 * @brief Typed GET for `"tactile.device_info"`.
 * @note On success, `*out` owns heap fields; release them with `wuji_tactile_glove_device_info_free`. Do not free on a non-OK status.
 */
WujiStatus wuji_hand_get_tactile_device_info(struct WujiDevice *dev,
                                             struct WujiTactileGloveDeviceInfo *out);

/**
 * @brief Typed GET for `"tactile.handedness"`.
 */
WujiStatus wuji_hand_get_tactile_handedness(struct WujiDevice *dev, uint8_t *out);

/**
 * @brief Typed GET for `"tactile.diagnostics"`.
 * @note On success, `*out` owns heap fields; release them with `wuji_tactile_glove_diagnostics_free`. Do not free on a non-OK status.
 */
WujiStatus wuji_hand_get_tactile_diagnostics(struct WujiDevice *dev,
                                             struct WujiTactileGloveDiagnostics *out);

/**
 * @brief Typed GET for `"ip_address"`.
 * @note Buffer-fill: writes a NUL-terminated string. Returns ErrBufferTooSmall (and sets *needed to the required size incl. NUL, if non-NULL) when buf is NULL or too small; never truncates. Two-call: pass buf=NULL,buf_len=0 to query *needed.
 */
WujiStatus wuji_hand_2_get_ip(struct WujiDevice *dev,
                              char *buf,
                              size_t buf_len,
                              size_t *needed);

/**
 * @brief Typed GET for `"hw_version"`.
 * @note On success, `*out` owns heap fields; release them with `wuji_hw_version_free`. Do not free on a non-OK status.
 */
WujiStatus wuji_hand_2_get_hw_version(struct WujiDevice *dev,
                                      struct WujiHwVersion *out);

/**
 * @brief Typed GET for `"comm_diag/hand"`.
 * @note On success, `*out` owns heap fields; release them with `wuji_hand_communication_diagnostics_free`. Do not free on a non-OK status.
 */
WujiStatus wuji_hand_2_get_comm_diag(struct WujiDevice *dev,
                                     struct WujiHandCommunicationDiagnostics *out);

/**
 * @brief Typed SET for `"ip_address"`.
 */
WujiStatus wuji_glove_set_ip(struct WujiDevice *dev, const char *value);

/**
 * @brief Typed SET for `"data_port"`.
 */
WujiStatus wuji_glove_set_port(struct WujiDevice *dev, uint16_t value);

/**
 * @brief Typed SET for `"ip_address"`.
 */
WujiStatus wuji_hand_2_set_ip(struct WujiDevice *dev, const char *value);

/**
 * @brief Execute the `"reboot"` action.
 */
WujiStatus wuji_glove_reboot(struct WujiDevice *dev);

/**
 * @brief Execute the `"reboot"` action.
 */
WujiStatus wuji_hand_2_reboot(struct WujiDevice *dev);

/**
 * @brief Initialize the SDK.
 *
 * Idempotent: subsequent calls return `WUJI_STATUS_OK`. Passing NULL opts is allowed
 * and uses defaults (`log_level = Info`).
 *
 * @param opts Optional pointer to initialization options.
 * @return `WUJI_STATUS_OK`.
 */
WujiStatus wuji_init(const struct WujiInitOptions *opts);

/**
 * @brief Gracefully drain and shut down.
 *
 * Currently a no-op.
 */
void wuji_shutdown(void);

/**
 * @brief Return a static NUL-terminated semver string like `"0.10.0"`.
 *
 * @return Pointer to static process-lifetime string storage. Do not free.
 */
const char *wuji_version(void);

/**
 * @brief Disconnect a device.
 *
 * The handle remains valid after disconnect; call `wuji_dev_release` to free it.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_dev_disconnect(struct WujiDevice *dev);

/**
 * @brief Release a device handle.
 *
 * Does not disconnect; call `wuji_dev_disconnect` first if needed.
 *
 * @param dev Device handle returned by `wuji_connect`, or NULL.
 * @note NULL-safe.
 */
void wuji_dev_release(struct WujiDevice *dev);

/**
 * @brief Get a parameter value as raw bytes.
 *
 * The returned payload is the raw protocol byte representation for `path`.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param path Non-NULL NUL-terminated parameter path.
 * @param out_buf Non-NULL writable pointer that receives a heap-allocated byte buffer.
 * @param out_len Non-NULL writable pointer that receives the byte length.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Free `*out_buf` with `wuji_buffer_free(*out_buf, *out_len)`.
 */
WujiStatus wuji_dev_get(struct WujiDevice *dev,
                        const char *path,
                        uint8_t **out_buf,
                        size_t *out_len);

/**
 * @brief Execute an ACTION-type resource (no payload, no return value).
 *
 * Used for EXEC resources such as `reboot`.
 * This is a low-level public primitive that the generated semantic wrappers
 * (e.g. `wuji_hand_2_reboot`) build on; prefer the device-specific wrapper
 * when one exists for your action.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param path Non-NULL NUL-terminated action path.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_dev_execute(struct WujiDevice *dev, const char *path);

/**
 * @brief Set a resource by raw byte payload.
 *
 * `buf` must contain the raw protocol byte representation expected by `path`.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param path Non-NULL NUL-terminated parameter path.
 * @param buf Pointer to `len` payload bytes. May be NULL only when `len == 0`.
 * @param len Number of bytes available at `buf`.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_dev_set(struct WujiDevice *dev, const char *path, const uint8_t *buf, size_t len);

/**
 * @brief Free a raw byte buffer returned by `wuji_dev_get`.
 *
 * @param buf Buffer pointer returned by `wuji_dev_get`, or NULL.
 * @param len Byte length returned by `wuji_dev_get`.
 * @note NULL-safe.
 */
void wuji_buffer_free(uint8_t *buf, size_t len);

/**
 * @brief List all get/set parameters for a device.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param out Non-NULL writable pointer that receives a parameter array.
 * @param out_n Non-NULL writable pointer that receives the array length.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Free `*out` with `wuji_param_array_free(*out, *out_n)`.
 */
WujiStatus wuji_dev_list_params(struct WujiDevice *dev, struct WujiParam **out, size_t *out_n);

/**
 * @brief Free a parameter array returned by `wuji_dev_list_params`.
 *
 * Also frees each per-entry `path` string.
 *
 * @param arr Array pointer returned by `wuji_dev_list_params`, or NULL.
 * @param n Array length returned by `wuji_dev_list_params`.
 * @note NULL-safe.
 */
void wuji_param_array_free(struct WujiParam *arr, size_t n);

/**
 * @brief List all subscribe/publish topics for a device.
 *
 * Global-scope topics are excluded; use `wuji_list_global_topics` for those.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param out Non-NULL writable pointer that receives a topic array.
 * @param out_n Non-NULL writable pointer that receives the array length.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Free `*out` with `wuji_topic_array_free(*out, *out_n)`.
 */
WujiStatus wuji_dev_list_topics(struct WujiDevice *dev, struct WujiTopic **out, size_t *out_n);

/**
 * @brief Fill `buf` with the device's serial number.
 *
 * Writes a NUL-terminated string when `buf_len > 0`. If `buf_len == 0`, returns
 * the size required excluding the NUL terminator.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param buf Writable output buffer, or NULL only when `buf_len == 0`.
 * @param buf_len Number of bytes available at `buf`.
 * @return Bytes written excluding NUL, required size when `buf_len == 0`, or `-1` on error.
 */
int32_t wuji_dev_serial_number(struct WujiDevice *dev, char *buf, size_t buf_len);

/**
 * @brief Fill `buf` with the device alias / device name.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param buf Writable output buffer, or NULL only when `buf_len == 0`.
 * @param buf_len Number of bytes available at `buf`.
 * @return Same contract as `wuji_dev_serial_number`.
 */
int32_t wuji_dev_device_name(struct WujiDevice *dev, char *buf, size_t buf_len);

/**
 * @brief Fill `buf` with the device model string.
 *
 * The model is a hex-formatted device identifier such as `"0x1234"`; a free-form
 * model name is not yet exposed.
 *
 * If the device hasn't completed its handshake yet, writes the empty
 * string and returns 0 (rather than `-1`); this matches the semantics of
 * the other best-effort metadata accessors.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param buf Writable output buffer, or NULL only when `buf_len == 0`.
 * @param buf_len Number of bytes available at `buf`.
 * @return Same contract as `wuji_dev_serial_number`.
 */
int32_t wuji_dev_model(struct WujiDevice *dev, char *buf, size_t buf_len);

/**
 * @brief Return the device's transport type.
 *
 * On NULL handle, returns `WUJI_TRANSPORT_TYPE_UDP` (the default) and records a
 * last error. Callers that need to distinguish NULL-handle from genuine UDP
 * should check via `wuji_dev_is_connected` first.
 *
 * @param dev Device handle returned by `wuji_connect`, or NULL.
 * @return Transport type enum value.
 */
WujiTransportType wuji_dev_transport(struct WujiDevice *dev);

/**
 * @brief Return whether the device is currently connected.
 *
 * @param dev Device handle returned by `wuji_connect`, or NULL.
 * @return 1 if connected, 0 otherwise. NULL handle returns 0 and sets `last_error`.
 */
uint8_t wuji_dev_is_connected(struct WujiDevice *dev);

/**
 * @brief Return the last error message for the current thread.
 *
 * @return Pointer to a thread-local C string, or NULL if no error is recorded.
 * The pointer is valid only until the next `wuji_*` call on the same thread.
 * Do not free it.
 */
const char *wuji_last_error(void);

/**
 * @brief Set WujiGlove tactile contact sensitivity.
 *
 * This SDK-local setting affects `tactile_binary` thresholding immediately and
 * persists for the active SDK user. Values must be finite and positive.
 *
 * @param dev Non-NULL WujiGlove device handle returned by `wuji_connect`.
 * @param sensitivity Contact sensitivity multiplier; values above 1 are more sensitive.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_glove_set_tactile_binary_sensitivity(struct WujiDevice *dev, double sensitivity);

/**
 * @brief Get the custom hand URDF path for WujiGlove online IK.
 *
 * Mirrors Python `glove.hand_model_path().get()`.
 *
 * @param dev Non-NULL WujiGlove device handle returned by `wuji_connect`.
 * @param buf Writable output buffer, or NULL to query the required size.
 * @param buf_len Number of bytes available at `buf`.
 * @param needed Optional out pointer; receives required size including NUL.
 * @return `WUJI_STATUS_OK` on success, or an error status. Returns
 * `WUJI_STATUS_ERR_BUFFER_TOO_SMALL` when `buf` is NULL or too small.
 */
WujiStatus wuji_glove_get_hand_model_path(struct WujiDevice *dev,
                                          char *buf,
                                          size_t buf_len,
                                          size_t *needed);

/**
 * @brief Set a custom hand URDF path for WujiGlove online IK.
 *
 * Mirrors Python `glove.hand_model_path().set(path)`. The path is validated
 * (non-empty, file readable) and stored in the device's SDK parameter store;
 * online IK streams (`hand_joint_angles`, `tip_poses`, `hand_skeleton`)
 * reload the hand model through the existing SDK calibration-generation
 * mechanism. An unreadable path fails immediately instead of being stored.
 *
 * The default SDK user does not support a custom hand model path: its online
 * IK always uses the built-in default URDF. Calling this function while the
 * default SDK user is active fails with an explanatory error (see
 * `wuji_last_error`); switch to a named SDK user first.
 *
 * @param dev Non-NULL WujiGlove device handle returned by `wuji_connect`.
 * @param urdf_path Non-NULL NUL-terminated UTF-8 path to a readable URDF file.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_glove_set_hand_model_path(struct WujiDevice *dev, const char *urdf_path);

/**
 * @brief Trigger a single time-sync round-trip on a WujiGlove device.
 *
 * Blocking NTP-style measurement; mirrors Python `WujiGlove.sync_time()`. The call
 * is serialized with the SDK's background time-sync task. On success `*out` is
 * filled; on error `*out` is left untouched.
 *
 * @param dev Non-NULL WujiGlove device handle returned by `wuji_connect`.
 * @param out Non-NULL; receives the measured offset / round-trip / timestamp.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_glove_sync_time(struct WujiDevice *dev, struct WujiTimeSyncResult *out);

/**
 * @brief Fill calibration options with defaults.
 */
WujiStatus wuji_glove_calibration_options_default(struct WujiGloveCalibrationOptions *out_options);

/**
 * @brief Start WUJI GLOVE IK calibration asynchronously.
 * @param options Optional; NULL uses the SDK default calibration options.
 */
WujiStatus wuji_glove_calibration_start(struct WujiDevice *dev,
                                        const struct WujiGloveCalibrationOptions *options,
                                        WujiGloveCalibrationFeedbackCallback callback,
                                        void *user_data,
                                        struct WujiGloveCalibrationSession **out_session);

/**
 * @brief Poll whether an asynchronous calibration session has finished.
 *
 * @note Must not be called from the same session's feedback callback.
 * @note When `*out_done` is false, `out_result` is not written and must not
 *       be read or passed to a result cleanup function.
 * @note Only read `out_result` when this function returns `WUJI_STATUS_OK`.
 */
WujiStatus wuji_glove_calibration_try_finish(struct WujiGloveCalibrationSession *session,
                                             bool *out_done,
                                             struct WujiGloveCalibrationResult *out_result);

/**
 * @brief Wait for an asynchronous calibration session to finish.
 *
 * @note Must not be called from the same session's feedback callback.
 */
WujiStatus wuji_glove_calibration_wait(struct WujiGloveCalibrationSession *session,
                                       struct WujiGloveCalibrationResult *out_result);

/**
 * @brief Request cooperative cancellation of an asynchronous calibration session.
 */
WujiStatus wuji_glove_calibration_cancel(struct WujiGloveCalibrationSession *session);

/**
 * @brief Free a calibration session handle.
 *
 * @note Do not call this from the same session's feedback callback. If a
 * callback needs to stop calibration, call `wuji_glove_calibration_cancel`
 * and release the session after the callback returns. Calls from the same
 * session's callback are ignored and record `wuji_last_error()`.
 */
void wuji_glove_calibration_session_free(struct WujiGloveCalibrationSession *session);

/**
 * @brief Run WUJI GLOVE IK calibration synchronously.
 */
WujiStatus wuji_glove_calibrate(struct WujiDevice *dev,
                                const struct WujiGloveCalibrationOptions *options,
                                WujiGloveCalibrationFeedbackCallback callback,
                                void *user_data,
                                struct WujiGloveCalibrationResult *out_result);

/**
 * @brief Free heap fields held by a calibration result.
 */
void wuji_glove_calibration_result_free(struct WujiGloveCalibrationResult *result);

/**
 * @brief Enable all 20 joint motors. NULL dev → ErrInvalidArg; unsupported build/device → ErrUnsupported.
 */
WujiStatus wuji_hand_enable(struct WujiDevice *dev);

/**
 * @brief Disable all 20 joint motors.
 */
WujiStatus wuji_hand_disable(struct WujiDevice *dev);

/**
 * @brief Clear fault flags on all 20 joints.
 */
WujiStatus wuji_hand_clear_all_faults(struct WujiDevice *dev);

/**
 * @brief Set effort limit (amps) for all 20 joints.
 */
WujiStatus wuji_hand_set_all_effort_limit(struct WujiDevice *dev, float amps);

/**
 * @brief Read effort limit (amps) for all 20 joints into `out[20]` (finger-major).
 */
WujiStatus wuji_hand_get_all_effort_limit(struct WujiDevice *dev, float *out);

/**
 * @brief Read per-joint diagnostics for all 20 joints into `out[20]` (finger-major).
 */
WujiStatus wuji_hand_get_all_diagnostics(struct WujiDevice *dev,
                                         struct WujiHandJointDiagnostics *out);

/**
 * @brief Read firmware soft limits (radians) for all 20 joints. upper/lower each receive 20 floats.
 */
WujiStatus wuji_hand_get_soft_limits(struct WujiDevice *dev, float *upper, float *lower);

/**
 * @brief Joint label for index 0..19, e.g. "index_joint2". Buffer-fill; no device.
 * buf=NULL → size query (*needed = bytes incl NUL, ErrBufferTooSmall). idx>=20 → ErrInvalidArg.
 */
WujiStatus wuji_hand_joint_label(uint8_t joint_index, char *buf, size_t buf_len, size_t *needed);

/**
 * @brief Finger name for index 0..4: thumb/index/middle/ring/pinky. No device.
 */
WujiStatus wuji_hand_finger_name(uint8_t finger_index, char *buf, size_t buf_len, size_t *needed);

/**
 * @brief Blocking one-shot read of all-joint actual positions (radians).
 * @param positions Non-NULL; receives 20 floats (finger-major). velocity/effort
 *   are not provided by this read (use the joint_states stream for those).
 *   NULL dev/out → ErrInvalidArg; unsupported build/device → ErrUnsupported;
 *   unexpected joint count from the device → ErrProtocol.
 */
WujiStatus wuji_hand_read_joint_state(struct WujiDevice *dev, float *positions);

/**
 * @brief Connect to a WujiHand by USB serial number (convenience over `wuji_connect` with kind=SN).
 *
 * Mirrors Python `Hand.connect_sn(sn, device_name="wuji_hand")`.
 *
 * @param sn Non-NULL NUL-terminated USB serial number string.
 * @param device_name_or_null Device alias for routing; NULL defaults to `"wuji_hand"`.
 * @param out Non-NULL writable pointer that receives an opaque device handle.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Release `*out` with `wuji_dev_release`.
 */
WujiStatus wuji_hand_connect_sn(const char *sn,
                                const char *device_name_or_null,
                                struct WujiDevice **out);

/**
 * @brief Whether a tactile glove is currently attached (consumes one frame
 * from the `tactile_status` stream; reflects runtime unplug, not just
 * connect-time pairing).
 * @param out Non-NULL; receives true iff status state == Healthy.
 */
WujiStatus wuji_hand_is_tactile_attached(struct WujiDevice *dev, bool *out);

/**
 * @brief Joint label for index 0..19, e.g. "thumb_S1". Buffer-fill; no device.
 */
WujiStatus wuji_hand_2_joint_label(uint8_t joint_index, char *buf, size_t buf_len, size_t *needed);

/**
 * @brief Finger name for index 0..4 ("thumb".."pinky"). Buffer-fill; no device.
 */
WujiStatus wuji_hand_2_finger_name(uint8_t finger_index, char *buf, size_t buf_len, size_t *needed);

/**
 * @brief Map a (bus_id, node_id) to the firmware joint id. Pure; no device.
 *
 * The joint id is `bus_id * 5 + (node_id - 1)` — the firmware node-id space
 * (4 joints + 1 tactile slot per bus, so the value is sparse, e.g. bus=4,
 * node=4 -> 23), NOT the dense 0..19 flat joint index used by `joint_label`.
 *
 * @param bus_id   CAN bus index.
 * @param node_id  1-based node index on that bus; MUST be >= 1.
 * @return The firmware joint id, or `0xFF` if `node_id == 0` (invalid — message
 *   in `wuji_last_error`; the Python wrapper raises `ValueError` instead).
 */
uint8_t wuji_hand_2_joint_id_from_bus_node(uint8_t bus_id, uint8_t node_id);

/**
 * @brief Enable joint motors.
 *
 * @param dev Non-NULL device handle.
 * @param mask_or_null NULL = enable the whole hand (ROOT broadcast). Otherwise a
 *   pointer to a flat-20 array of 0/1 bytes (joint index 0..19): joints whose
 *   byte is non-zero are enabled per-joint, the rest are left untouched.
 */
WujiStatus wuji_hand_2_enable(struct WujiDevice *dev, const uint8_t *mask_or_null);

/**
 * @brief Disable joint motors.
 *
 * @param dev Non-NULL device handle.
 * @param mask_or_null NULL = disable the whole hand; else a flat-20 array of 0/1
 *   bytes selecting joints to disable (see `wuji_hand_2_enable`).
 */
WujiStatus wuji_hand_2_disable(struct WujiDevice *dev, const uint8_t *mask_or_null);

/**
 * @brief Clear fault flags on joints.
 *
 * @param dev Non-NULL device handle.
 * @param mask_or_null NULL = clear the whole hand; else a flat-20 array of 0/1
 *   bytes selecting joints to clear (see `wuji_hand_2_enable`).
 */
WujiStatus wuji_hand_2_clear_fault(struct WujiDevice *dev, const uint8_t *mask_or_null);

/**
 * @brief Set the user origin (zero) at the current physical position.
 *
 * Runs the firmware offset (does not write Flash).
 *
 * @param dev Non-NULL device handle.
 * @param mask_or_null NULL = set origin on the whole hand; else a flat-20 array
 *   of 0/1 bytes selecting joints (see `wuji_hand_2_enable`).
 */
WujiStatus wuji_hand_2_set_origin(struct WujiDevice *dev, const uint8_t *mask_or_null);

/**
 * @brief Clear the user origin (offset → 0).
 *
 * @param dev Non-NULL device handle.
 * @param mask_or_null NULL = clear origin on the whole hand; else a flat-20
 *   array of 0/1 bytes selecting joints (see `wuji_hand_2_enable`).
 */
WujiStatus wuji_hand_2_clear_origin(struct WujiDevice *dev, const uint8_t *mask_or_null);

/**
 * @brief Emergency-stop all joints (hardware-level hard stop). Whole-hand only.
 */
WujiStatus wuji_hand_2_emergency_stop(struct WujiDevice *dev);

/**
 * @brief Runtime-calibrate all five fingertip tactile sensors to a fresh zero
 *   baseline. Whole-hand only. Mirrors Python `WujiHand2.tactile_calibrate()`.
 *
 * Triggers the fingers in fixed order (thumb → index → middle → ring → pinky).
 * **Fail-fast**: the first finger whose calibrate fails (e.g. its tactile node
 * is offline) aborts and returns that error; the remaining fingers are not
 * triggered. **All sensor surfaces MUST be unloaded when called.** Fire-and-
 * forget: `WUJI_STATUS_OK` means every finger's command was enqueued, not that
 * calibration has completed.
 */
WujiStatus wuji_hand_2_tactile_calibrate(struct WujiDevice *dev);

/**
 * @brief Query a finger's tactile model and runtime state (blocking).
 *
 * Mirrors Python `WujiHand2.tactile_status(finger)`: triggers a QueryStatus on
 * the finger's tactile slave and polls until a fresh state is observed (up to
 * ~1s worst case). Use after `wuji_hand_2_tactile_calibrate` to confirm
 * calibration finished (`out_state` back to `ready`).
 *
 * @param dev        Non-NULL WujiHand2 device handle.
 * @param finger     0=thumb, 1=index, 2=middle, 3=ring, 4=pinky.
 * @param out_model  Non-NULL; receives the `WujiTactileType` on success.
 * @param out_state  Non-NULL; receives the `WujiTactileState` on success.
 * @return `WUJI_STATUS_OK` on success, or an error status. On error the out
 *         params are untouched; `WUJI_STATUS_ERR_TIMEOUT` means no fresh state
 *         arrived in time.
 */
WujiStatus wuji_hand_2_tactile_status(struct WujiDevice *dev,
                                      uint8_t finger,
                                      enum WujiTactileType *out_model,
                                      enum WujiTactileState *out_state);

/**
 * @brief Look up the firmware-authored description of a device fault code.
 *
 * Mirrors Python's `WujiHand2.describe_error`: fills `out` with the code's
 * name / severity / clear policy / description / cause / resolution. Static —
 * needs no device connection.
 *
 * Accepts the device fault codes carried by
 * `WujiJointDiagnosticsEntry::error_code_current`, and the same values surfaced
 * by the error log and calibration results in the Python SDK. Calibration also
 * reports a separate encoder-only error byte there; that one is a different,
 * unrelated enum and must not be passed here.
 *
 * @param code The fault code to describe.
 * @param out  Non-NULL `WujiErrorInfo` to fill (zeroed first).
 * @return `WUJI_STATUS_OK` if the code is known; `WUJI_STATUS_ERR_NOT_FOUND` if
 *   the code is unknown; `WUJI_STATUS_ERR_INVALID_ARG` if `out` is NULL.
 */
WujiStatus wuji_hand_2_describe_error(uint16_t code, struct WujiErrorInfo *out);

/**
 * @brief Read a fingertip's self-describing sensor info.
 *
 * Mirrors Python's `WujiHand2.get_fingertip_info`: drives the chunked info read
 * internally (SET selector + loop GET chunk + reassemble + postcard-decode) and
 * fills `*out` with the ready `WujiFingertipSensorInfo`. The chunked-transport
 * structs are internal and never exposed. `model`/`format` are inline fixed
 * buffers, so no `*_free` is needed.
 *
 * @param dev    Non-NULL WujiHand2 device handle.
 * @param finger 0=thumb, 1=index, 2=middle, 3=ring, 4=pinky.
 * @param out    Non-NULL; receives the decoded info on success, untouched on error.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_hand_2_get_fingertip_info(struct WujiDevice *dev,
                                          uint8_t finger,
                                          struct WujiFingertipSensorInfo *out);

/**
 * @brief Set the effort (current) limit for all 20 joints to one value.
 *
 * @param amps  Effort limit in amperes (applied to every joint).
 */
WujiStatus wuji_hand_2_set_all_effort_limit(struct WujiDevice *dev, float amps);

/**
 * @brief Set the effort (current) limit per joint. `limits` is a flat-20 array
 * (joint scan index 0..19, finger-major). Mirrors the Python per-joint form
 * `effort_limit().set([...])`; use `wuji_hand_2_set_all_effort_limit` to apply
 * a single value to every joint.
 *
 * @param limits Pointer to >= 20 floats (per-joint effort limit, amperes).
 */
WujiStatus wuji_hand_2_set_all_effort_limit_per_joint(struct WujiDevice *dev, const float *limits);

/**
 * @brief Set MIT kp/kd for all 20 joints. `kp`/`kd` are flat-20 arrays (joint
 * scan index 0..19, finger-major: thumb / index / middle / ring / pinky).
 * Pointers must be non-NULL and point to at least 20 floats.
 *
 * @param kp Pointer to >= 20 floats (per-joint kp).
 * @param kd Pointer to >= 20 floats (per-joint kd).
 */
WujiStatus wuji_hand_2_set_all_mit_params(struct WujiDevice *dev, const float *kp, const float *kd);

/**
 * @brief Read the device handedness (Left/Right).
 *
 * @param out  Output enum value (`WUJI_HANDEDNESS_LEFT` or `WUJI_HANDEDNESS_RIGHT`).
 */
WujiStatus wuji_hand_2_get_handedness(struct WujiDevice *dev, enum WujiHandedness *out);

/**
 * @brief Read per-joint effort (current) limit (amps). `online` bit i set =
 * joint i present (out[i] valid); offline joints have out[i]=0.
 * @param out Caller array of at least WUJI_HAND_2_JOINT_COUNT (20) elements; offline slots are zeroed.
 * @param online Bitmap out-param; bit i set = joint i present. Use WUJI_JOINT_ONLINE(*online, i).
 */
WujiStatus wuji_hand_2_get_all_effort_limit(struct WujiDevice *dev,
                                            float *out,
                                            uint32_t *online);

/**
 * @brief Read MIT kp/kd for all 20 joints (flat-20: joint scan index 0..19,
 * finger-major). Offline joints have kp[i]=kd[i]=NaN and `online` bit i clear.
 * @param kp,kd Caller arrays of at least 20 floats; offline joints = NaN.
 * @param online Bitmap out-param; bit i set = joint i present. Use WUJI_JOINT_ONLINE(*online, i).
 */
WujiStatus wuji_hand_2_get_all_mit_params(struct WujiDevice *dev,
                                          float *kp,
                                          float *kd,
                                          uint32_t *online);

/**
 * @brief Count joints currently online (from communication diagnostics).
 * @param out Output: number of joints online (0..20).
 */
WujiStatus wuji_hand_2_online_joints_count(struct WujiDevice *dev, uint8_t *out);

/**
 * @brief Open a joint-command publisher on a WujiHand2 device.
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param out_pub Receives an opaque handle; close with `wuji_joint_command_publisher_close`.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_hand_2_joint_command_publish(struct WujiDevice *dev,
                                             struct WujiJointCommandPublisher **out_pub);

/**
 * @brief Send a command for all 20 joints (AoS).
 *
 * Reads exactly 20 `WujiJointCommand` elements (position + velocity + effort
 * each) and publishes them; the sequence number is auto-incremented. Use
 * `{0, 0, 0}` for joints you don't want to command.
 *
 * @param pubh Non-NULL publisher handle returned by `wuji_hand_2_joint_command_publish`.
 * @param cmds Non-NULL pointer to exactly 20 `WujiJointCommand` elements.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_joint_command_publisher_send(struct WujiJointCommandPublisher *pubh,
                                             const struct WujiJointCommand *cmds);

/**
 * @brief Close and release a joint-command publisher handle. NULL-safe.
 * @param pubh Publisher handle returned by `wuji_hand_2_joint_command_publish`, or NULL.
 */
void wuji_joint_command_publisher_close(struct WujiJointCommandPublisher *pubh);

/**
 * @brief Open a joint-command publisher on a WujiHand device.
 * @param dev Non-NULL device handle returned by `wuji_connect` or `wuji_hand_connect_sn`.
 * @param out_pub Non-NULL writable pointer that receives an opaque publisher handle.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Close `*out_pub` with `wuji_hand_joint_command_publisher_close`.
 */
WujiStatus wuji_hand_joint_command_publish(struct WujiDevice *dev,
                                           struct WujiHandJointCommandPublisher **out_pub);

/**
 * @brief Send a command for all 20 joints (AoS). Reads exactly 20 `WujiJointCommand`
 * elements (position + velocity + effort each) and publishes them; the sequence
 * number auto-increments. Use `{0, 0, 0}` for joints you don't want to command.
 * Identical in shape to `wuji_joint_command_publisher_send` (WujiHand2).
 *
 * @param pubh Non-NULL publisher handle returned by `wuji_hand_joint_command_publish`.
 * @param cmds Non-NULL pointer to exactly 20 `WujiJointCommand` elements (finger-major).
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_hand_joint_command_publisher_send(struct WujiHandJointCommandPublisher *pubh,
                                                  const struct WujiJointCommand *cmds);

/**
 * @brief Close and release the publisher handle.
 * @param pubh Publisher handle returned by `wuji_hand_joint_command_publish`, or NULL.
 * @note NULL-safe.
 */
void wuji_hand_joint_command_publisher_close(struct WujiHandJointCommandPublisher *pubh);

/**
 * @brief Open a realtime controller on a WujiHand device.
 * @param dev Non-NULL WujiHand device handle. @param filter low-pass cutoff.
 * @param out Non-NULL; receives an opaque handle, release with `wuji_hand_realtime_controller_close`.
 * @return Ok; ErrInvalidArg (NULL dev/out); ErrUnsupported (unsupported build or device);
 *   or an error if a controller at a different cutoff is already active.
 */
WujiStatus wuji_hand_realtime_controller_open(struct WujiDevice *dev,
                                              struct WujiLowPass filter,
                                              struct WujiRealtimeController **out);

/**
 * @brief Write filtered target positions (radians, 20 finger-major floats).
 * @param ctrl Non-NULL handle. @param positions Non-NULL, exactly 20 floats.
 */
WujiStatus wuji_hand_realtime_controller_set_target_position(struct WujiRealtimeController *ctrl,
                                                             const float *positions);

/**
 * @brief Read the controller's cached actual positions into `out[20]` (finger-major).
 */
WujiStatus wuji_hand_realtime_controller_get_actual_position(struct WujiRealtimeController *ctrl,
                                                             float *out);

/**
 * @brief Read the controller's cached actual efforts (amps) into `out[20]` (finger-major).
 *
 * Like `get_actual_position` but returns per-joint effort from the same atomic
 * upstream cache. NULL ctrl/out → ErrInvalidArg; unsupported build → ErrUnsupported.
 */
WujiStatus wuji_hand_realtime_controller_get_actual_effort(struct WujiRealtimeController *ctrl,
                                                           float *out);

/**
 * @brief Close and release the controller handle (detaches on last owner). NULL-safe.
 */
void wuji_hand_realtime_controller_close(struct WujiRealtimeController *ctrl);

/**
 * @brief Return the default connection options.
 *
 * @return A `WujiConnectOptions` value matching the SDK default connection behavior.
 * @note Start from this value before overriding fields, then pass its address to `wuji_connect`.
 */
struct WujiConnectOptions wuji_connect_options_default(void);

/**
 * @brief Scan for devices on USB and UDP.
 *
 * @param out Non-NULL writable pointer that receives a device array.
 * @param out_n Non-NULL writable pointer that receives the array length.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Free `*out` with `wuji_discovered_free(*out, *out_n)`.
 */
WujiStatus wuji_scan(struct WujiDiscovered **out, size_t *out_n);

/**
 * @brief Free a device array returned by `wuji_scan`.
 *
 * @param arr Array pointer returned by `wuji_scan`, or NULL.
 * @param n Array length returned by `wuji_scan`.
 * @note NULL-safe.
 */
void wuji_discovered_free(struct WujiDiscovered *arr, size_t n);

/**
 * @brief Connect to a device.
 *
 * @param target Non-NULL connect target.
 * @param alias_or_null Non-NULL NUL-terminated device name used for routing.
 * @param opts_or_null Optional connect options; pass NULL for defaults.
 * @param out_dev Non-NULL writable pointer that receives an opaque device handle.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Release `*out_dev` with `wuji_dev_release`.
 */
WujiStatus wuji_connect(const struct WujiConnectTarget *target,
                        const char *alias_or_null,
                        const struct WujiConnectOptions *opts_or_null,
                        struct WujiDevice **out_dev);

/**
 * @brief List all global (cross-device) topics.
 *
 * @param out Non-NULL writable pointer that receives a topic array.
 * @param out_n Non-NULL writable pointer that receives the array length.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Free `*out` with `wuji_topic_array_free(*out, *out_n)`.
 */
WujiStatus wuji_list_global_topics(struct WujiTopic **out, size_t *out_n);

/**
 * @brief Free a topic array returned by topic-listing functions.
 *
 * Also frees the per-entry `path` strings.
 *
 * @param arr Array pointer returned by `wuji_list_global_topics` or `wuji_dev_list_topics`, or NULL.
 * @param n Array length returned by the listing function.
 * @note NULL-safe.
 */
void wuji_topic_array_free(struct WujiTopic *arr,
                           size_t n);

/**
 * @brief Open a publisher for the given topic path on a device.
 *
 * @param dev Non-NULL device handle returned by `wuji_connect`.
 * @param path Non-NULL NUL-terminated topic path.
 * @param out_pub Non-NULL writable pointer that receives an opaque publisher handle.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Close `*out_pub` with `wuji_pub_close`.
 */
WujiStatus wuji_pub_open(struct WujiDevice *dev, const char *path, struct WujiPub **out_pub);

/**
 * @brief Send raw bytes on an open publisher.
 *
 * `buf` must contain the raw protocol byte representation expected by the
 * path bound at `wuji_pub_open`.
 *
 * @param pubh Non-NULL publisher handle returned by `wuji_pub_open`.
 * @param buf Pointer to `len` payload bytes. May be NULL only when `len == 0`.
 * @param len Number of bytes available at `buf`.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 */
WujiStatus wuji_pub_send(struct WujiPub *pubh, const uint8_t *buf, size_t len);

/**
 * @brief Close and release a publisher handle.
 *
 * @param pubh Publisher handle returned by `wuji_pub_open`, or NULL.
 * @note NULL-safe.
 */
void wuji_pub_close(struct WujiPub *pubh);

/**
 * @brief Create a retargeting session for a hand model (builtin tuning config).
 * @param model Which hand model to target: a `WUJI_HAND_MODEL_*` constant.
 *        Unknown values are rejected with WUJI_STATUS_ERR_INVALID_ARG.
 * @param side Handedness: a `WUJI_HANDEDNESS_*` constant. Unknown values are
 *        rejected with WUJI_STATUS_ERR_INVALID_ARG.
 * @param out_session Receives the new session handle; free with
 *        wuji_retarget_session_free.
 */
WujiStatus wuji_retarget_session_create(int32_t model,
                                        int32_t side,
                                        struct WujiRetargetSession **out_session);

/**
 * @brief Retarget one frame.
 * @param session Session handle from wuji_retarget_session_create.
 * @param keypoints 63 floats (21×3, row-major xyz, MediaPipe landmark order).
 * @param qpos_out 20 floats, joint angles (radians) in firmware order.
 */
WujiStatus wuji_retarget_session_step(struct WujiRetargetSession *session,
                                      const float *keypoints,
                                      float *qpos_out);

/**
 * @brief Reset warm-start and filter state (e.g. after a tracking gap).
 * @param session Session handle from wuji_retarget_session_create.
 */
WujiStatus wuji_retarget_session_reset(struct WujiRetargetSession *session);

/**
 * @brief Free a session handle. Safe to call with NULL (no-op).
 * @param session Session handle from wuji_retarget_session_create, or NULL.
 */
void wuji_retarget_session_free(struct WujiRetargetSession *session);

/**
 * @brief Open a subscription to any topic.
 *
 * The SDK uses the topic registry to determine which backend to use:
 * - If `dev` is non-NULL and `path` is in that device's topic table
 *   → device subscribe.
 * - If `path` is in the global topic registry (e.g. `tf_static`, cross-device
 *   aggregated streams) → global subscribe; `dev` is ignored in this case.
 * - Otherwise → `WUJI_STATUS_ERR_NOT_FOUND` with a clear last_error message.
 *
 * `dev` may be NULL when subscribing to a global topic from a context that
 * doesn't hold a device handle (e.g. a monitor that just wants `tf_static`).
 *
 * On success, `*out_sub` receives an opaque handle that must be freed with
 * `wuji_sub_close`. The callback will be invoked from the worker thread for
 * every frame, every lag event, and once with `kind` equal to either
 * `WUJI_FRAME_KIND_END` or `WUJI_FRAME_KIND_ERROR` to signal termination.
 *
 * @param dev Device handle returned by `wuji_connect`, or NULL for global-only lookup.
 * @param path Non-NULL NUL-terminated topic path.
 * @param cb Non-NULL callback invoked from the subscription worker thread.
 * @param user_data Opaque caller pointer stored verbatim and passed to `cb`; may be NULL.
 * @param out_sub Non-NULL writable pointer that receives an opaque subscription handle.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Close `*out_sub` with `wuji_sub_close`.
 */
WujiStatus wuji_sub_open(struct WujiDevice *dev,
                         const char *path,
                         WujiSubCallback cb,
                         void *user_data,
                         struct WujiSub **out_sub);

/**
 * @brief Close a subscription, signalling the worker to exit and joining the thread.
 *
 * The callback will receive one final invocation with `kind` equal to
 * `WUJI_FRAME_KIND_END` before the worker terminates (unless the source
 * already emitted `WUJI_FRAME_KIND_ERROR` or `WUJI_FRAME_KIND_END`). After
 * this call returns, the callback will never fire again.
 *
 * @param sub Subscription handle returned by `wuji_sub_open`, or NULL.
 * @note NULL-safe.
 */
void wuji_sub_close(struct WujiSub *sub);

/**
 * @brief Set the device push rate (samples per second) of a subscribed stream.
 *
 * Only streams produced directly by the device support rate control. Derived
 * streams follow their source automatically, while global topics such as
 * `tf_static` are aggregated by the SDK. Calling this function on either returns
 * `WUJI_STATUS_ERR_UNSUPPORTED`. The device quantizes the request to a supported
 * rate (an integer division of the native rate) and returns the rate it applies.
 * Pass 0 to restore the device default. The rate is shared by all subscribers
 * of the stream (last-writer-wins). It resets to the device default when the
 * last subscription to the stream is closed or the device reconnects.
 *
 * @param sub Subscription handle returned by `wuji_sub_open`.
 * @param frequency_hz Target rate in Hz; 0 = device default full rate.
 * @param out_actual_hz Optional (nullable) pointer receiving the rate the
 *        device actually applied (quantized to a supported rate).
 * @return `WUJI_STATUS_OK` on success; `WUJI_STATUS_ERR_UNSUPPORTED` when the
 *         subscription is a downstream stream or a global topic such as
 *         `tf_static`, or the firmware does not support rate control; another
 *         error status on device or communication failure.
 */
WujiStatus wuji_sub_set_rate(struct WujiSub *sub, uint16_t frequency_hz, uint16_t *out_actual_hz);

/**
 * @brief Run blocking tactile calibration.
 * @note Returns `WUJI_STATUS_ERR_UNSUPPORTED` when this library was built
 * without `model-export`. Callback event pointers are borrowed for the
 * callback duration. On success, release `out` with
 * `wuji_tactile_calibration_summary_free`.
 * @note `timeout_s` is COOPERATIVE. On timeout the calibration is cancelled,
 * but this function first waits for any in-flight collect/train work and any
 * in-flight prompt/feedback callback to finish before returning. This
 * guarantees no callback is ever invoked after this function returns, so the
 * pointers you pass as `feedback_userdata`/`prompt_userdata` stay valid for
 * the whole call. The corollary: if your `on_pose_prompt` callback blocks
 * indefinitely (e.g. waiting on console input), the timeout cannot force it to
 * return and this call will wait for it.
 */
WujiStatus wuji_glove_calibrate_tactile_blocking(struct WujiDevice *dev,
                                                 const struct WujiTactileCalibrationOptions *options,
                                                 const struct WujiTactileCalibrationCallbacks *callbacks,
                                                 struct WujiTactileCalibrationSummary *out);

/**
 * @brief Free fields produced by `wuji_glove_calibrate_tactile_blocking`.
 * @note NULL-safe; does not free the `summary` struct itself.
 */
void wuji_tactile_calibration_summary_free(struct WujiTactileCalibrationSummary *summary);

/**
 * @brief Create an SDK user.
 *
 * @param display_name Non-NULL UTF-8 display name, unique in the local SDK registry.
 * @param description Optional UTF-8 description, or NULL.
 * @param external_id Optional caller-owned stable user id, or NULL; non-empty values are unique.
 * @param out_user Non-NULL; receives SDK-allocated strings on success.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Release internal strings with `wuji_user_info_free(out_user)`.
 */
WujiStatus wuji_create_user(const char *display_name,
                            const char *description,
                            const char *external_id,
                            struct WujiUserInfo *out_user);

/**
 * @brief List SDK users.
 *
 * @param out_users Non-NULL; receives a heap-allocated array.
 * @param out_len Non-NULL; receives the array length.
 * @return `WUJI_STATUS_OK` on success, or an error status.
 * @note Free with `wuji_user_info_array_free(*out_users, *out_len)`.
 */
WujiStatus wuji_list_users(struct WujiUserInfo **out_users, size_t *out_len);

/**
 * @brief Get the current SDK user.
 */
WujiStatus wuji_current_user(struct WujiUserInfo *out_user);

/**
 * @brief Switch the current SDK user and reload connected device parameter stores.
 */
WujiStatus wuji_switch_user(const char *user_id, struct WujiUserInfo *out_user);

/**
 * @brief Switch to the default SDK user.
 */
WujiStatus wuji_switch_to_default_user(struct WujiUserInfo *out_user);

/**
 * @brief Update an SDK user.
 */
WujiStatus wuji_update_user(const char *user_id,
                            const struct WujiUserUpdate *update,
                            struct WujiUserInfo *out_user);

/**
 * @brief Delete an SDK user.
 */
WujiStatus wuji_delete_user(const char *user_id);

/**
 * @brief Free internal strings held by a `WujiUserInfo`.
 *
 * @param user Pointer to a struct filled by a user API, or NULL.
 * @note NULL-safe. Frees only internal strings, not the struct allocation itself.
 */
void wuji_user_info_free(struct WujiUserInfo *user);

/**
 * @brief Free a user array returned by `wuji_list_users`.
 *
 * @param users Array pointer returned by `wuji_list_users`, or NULL.
 * @param len Array length returned by `wuji_list_users`.
 * @note NULL-safe.
 */
void wuji_user_info_array_free(struct WujiUserInfo *users, size_t len);

/**
 * @brief Export all of an SDK user's data into a portable zip bundle.
 *
 * Writes everything the user owns (hand models plus the tactile model and its
 * latest complete calibration run). Pass `user_id = ""` for the default user.
 *
 * @param user_id NUL-terminated UTF-8 SDK user id (`""` = default user).
 * @param out_zip_path NUL-terminated UTF-8 output path; must end in `.zip`.
 * @return WUJI_STATUS_OK on success; see wuji_last_error() on failure.
 */
WujiStatus wuji_user_data_export(const char *user_id, const char *out_zip_path);

/**
 * @brief Validate a user data bundle without importing it.
 *
 * Runs the full validation (zip-slip, size limits, sha256, bag contract, owner
 * id) and returns OK if the bundle is importable. Nothing is written.
 *
 * @param zip_path NUL-terminated UTF-8 path to a bundle zip.
 * @return WUJI_STATUS_OK if importable; see wuji_last_error() on failure.
 */
WujiStatus wuji_user_data_preview(const char *zip_path);

/**
 * @brief Import a user data bundle, restoring it to the bundle's owner user.
 *
 * Creates the owner SDK user if it does not exist, then writes every present
 * component, overwriting any existing files for that user. The current user is
 * not switched. Fails without writing anything if validation fails.
 *
 * @param zip_path NUL-terminated UTF-8 path to a bundle zip.
 * @return WUJI_STATUS_OK on success; see wuji_last_error() on failure.
 */
WujiStatus wuji_user_data_import(const char *zip_path);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* WUJI_SDK_H */
