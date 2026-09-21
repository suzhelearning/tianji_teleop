/* Offline ABI fixture: includes the pinned header, links no SDK or transport. */
#include "wuji_sdk.h"
#include <string.h>

size_t hardware_abi_size(unsigned type) {
    switch (type) {
    case 0: return sizeof(WujiFrameHeader);
    case 1: return sizeof(WujiJointStateEntry);
    case 2: return sizeof(WujiJointStateFrame);
    case 3: return sizeof(WujiJointDiagnosticsEntry);
    case 4: return sizeof(WujiHand2CommSummary);
    case 5: return sizeof(WujiJointDiagnosticsFrame);
    case 6: return sizeof(WujiJointCommand);
    case 7: return sizeof(WujiConnectTarget);
    case 8: return sizeof(WujiConnectOptions);
    case 9: return sizeof(WujiErrorInfo);
    default: return 0;
    }
}

void hardware_abi_emit(WujiJointStateFrameSubCallback states,
                       WujiJointDiagnosticsFrameSubCallback diagnostics,
                       uint32_t sequence, uint32_t status, uint16_t fault) {
    WujiJointStateEntry joints[20] = {0};
    WujiJointDiagnosticsEntry entries[20] = {0};
    for (size_t i = 0; i < 20; ++i) {
        const size_t dense = 19 - i;
        const uint8_t nid = (uint8_t)((dense / 4) * 5 + dense % 4 + 1);
        joints[i].nid = nid;
        joints[i].position = (float)dense / 4.0f;
        joints[i].velocity = -0.25f;
        joints[i].effort = 0.125f;
        entries[i].nid = nid;
        entries[i].status_word = status;
        entries[i].current = 0.125f;
        entries[i].vbus_v_fb = 24.0f;
        entries[i].mcu_temp_c_fb = 30.0f;
        entries[i].error_code_current = fault;
        entries[i].comm_response_rate_pct = 100;
    }
    WujiJointStateFrame state_frame = {0};
    state_frame.header.seq = sequence;
    state_frame.header.timestamp_us = (uint64_t)sequence * 1000;
    strcpy(state_frame.header.frame_id, "right");
    state_frame.num_joints = 20;
    state_frame.joints = joints;
    state_frame.joints_len = 20;
    WujiJointDiagnosticsFrame diag_frame = {0};
    diag_frame.header = state_frame.header;
    diag_frame.num_joints = 20;
    diag_frame.joints = entries;
    diag_frame.joints_len = 20;
    states(WUJI_FRAME_KIND_OK, &state_frame, NULL);
    diagnostics(WUJI_FRAME_KIND_OK, &diag_frame, NULL);
}

int hardware_abi_commands(const WujiJointCommand *commands) {
    for (size_t i = 0; i < 20; ++i) {
        if (commands[i].position != (float)i / 4.0f ||
            commands[i].velocity != 0.0f || commands[i].effort != 0.0f) {
            return 0;
        }
    }
    return 1;
}
