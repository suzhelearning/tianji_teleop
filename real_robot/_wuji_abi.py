"""Minimal pinned wuji_sdk.h device ABI; loading is explicit and never scans."""
import ctypes as ct
from pathlib import Path

from .read_hand_serials import ConnectOptions, ConnectTarget, check


class FrameHeader(ct.Structure):
    _fields_ = [('seq', ct.c_uint32), ('timestamp_us', ct.c_uint64), ('frame_id', ct.c_char * 32)]


class JointStateEntry(ct.Structure):
    _fields_ = [('nid', ct.c_uint8), ('position', ct.c_float), ('velocity', ct.c_float), ('effort', ct.c_float)]


class JointStateFrame(ct.Structure):
    _fields_ = [('header', FrameHeader), ('num_joints', ct.c_uint8),
                ('joints', ct.POINTER(JointStateEntry)), ('joints_len', ct.c_size_t)]


class JointDiagnosticsEntry(ct.Structure):
    _fields_ = [('nid', ct.c_uint8), ('status_word', ct.c_uint32),
                ('current', ct.c_float), ('vbus_v_fb', ct.c_float), ('mcu_temp_c_fb', ct.c_float),
                ('error_code_current', ct.c_uint16), ('comm_response_rate_pct', ct.c_uint8),
                ('comm_timeout_total', ct.c_uint32)]


class Hand2CommSummary(ct.Structure):
    _fields_ = [('age_ms', ct.c_uint16), ('tactile_online_mask', ct.c_uint8),
                ('e2e_received', ct.c_uint32), ('e2e_lost', ct.c_uint32),
                ('e2e_reordered', ct.c_uint16), ('e2e_duplicates', ct.c_uint16),
                ('e2e_window_loss_x100', ct.c_uint16), ('rpc_total', ct.c_uint32),
                ('rpc_retries', ct.c_uint16), ('rpc_timeouts', ct.c_uint16),
                ('comm_get_failures', ct.c_uint16), ('tactile_response_rate_pct', ct.c_uint8 * 5),
                ('tactile_timeout_total', ct.c_uint32 * 5), ('sdk_dropped', ct.c_uint32)]


class JointDiagnosticsFrame(ct.Structure):
    _fields_ = [('header', FrameHeader), ('num_joints', ct.c_uint8),
                ('joints', ct.POINTER(JointDiagnosticsEntry)), ('joints_len', ct.c_size_t),
                ('comm', Hand2CommSummary)]


class JointCommand(ct.Structure):
    _fields_ = [('position', ct.c_float), ('velocity', ct.c_float), ('effort', ct.c_float)]


class ErrorInfo(ct.Structure):
    _fields_ = [('code', ct.c_uint16), ('is_test', ct.c_bool),
                ('name', ct.c_char * 64), ('severity', ct.c_char * 24),
                ('clear_policy', ct.c_char * 24), ('desc', ct.c_char * 160),
                ('cause', ct.c_char * 320), ('resolution', ct.c_char * 256)]


StateCallback = ct.CFUNCTYPE(None, ct.c_uint8, ct.POINTER(JointStateFrame), ct.c_void_p)
DiagnosticsCallback = ct.CFUNCTYPE(None, ct.c_uint8, ct.POINTER(JointDiagnosticsFrame), ct.c_void_p)


def load_sdk(path: Path):
    sdk = ct.CDLL(str(path.resolve()))
    pointer = ct.c_void_p
    out_pointer = ct.POINTER(pointer)
    floats = ct.POINTER(ct.c_float)
    signatures = {
        'wuji_init': ([pointer], ct.c_int32),
        'wuji_shutdown': ([], None),
        'wuji_last_error': ([], ct.c_char_p),
        'wuji_connect_options_default': ([], ConnectOptions),
        'wuji_connect': ([ct.POINTER(ConnectTarget), ct.c_char_p, ct.POINTER(ConnectOptions), out_pointer], ct.c_int32),
        'wuji_hand_2_get_handedness': ([pointer, ct.POINTER(ct.c_int)], ct.c_int32),
        'wuji_hand_2_online_joints_count': ([pointer, ct.POINTER(ct.c_uint8)], ct.c_int32),
        'wuji_hand_2_describe_error': ([ct.c_uint16, ct.POINTER(ErrorInfo)], ct.c_int32),
        'wuji_hand_2_subscribe_joint_states': ([pointer, StateCallback, pointer, out_pointer], ct.c_int32),
        'wuji_hand_2_subscribe_joint_diagnostics': ([pointer, DiagnosticsCallback, pointer, out_pointer], ct.c_int32),
        'wuji_hand_2_set_all_effort_limit': ([pointer, ct.c_float], ct.c_int32),
        'wuji_hand_2_set_all_mit_params': ([pointer, floats, floats], ct.c_int32),
        'wuji_hand_2_get_all_mit_params': ([pointer, floats, floats, ct.POINTER(ct.c_uint32)], ct.c_int32),
        'wuji_hand_2_enable': ([pointer, ct.POINTER(ct.c_uint8)], ct.c_int32),
        'wuji_hand_2_disable': ([pointer, ct.POINTER(ct.c_uint8)], ct.c_int32),
        'wuji_hand_2_joint_command_publish': ([pointer, out_pointer], ct.c_int32),
        'wuji_joint_command_publisher_send': ([pointer, ct.POINTER(JointCommand)], ct.c_int32),
        'wuji_joint_command_publisher_close': ([pointer], None),
        'wuji_sub_close': ([pointer], None),
        'wuji_dev_disconnect': ([pointer], ct.c_int32),
        'wuji_dev_release': ([pointer], None),
    }
    for name, (arguments, result) in signatures.items():
        function = getattr(sdk, name)
        function.argtypes, function.restype = arguments, result
    return sdk
