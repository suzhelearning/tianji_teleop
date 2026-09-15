#ifndef ODIN_LIDAR_INNER_H_
#define ODIN_LIDAR_INNER_H_

#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

// ============================================================================
// Internal Structures (not exposed to SDK users)
// ============================================================================

/**
 * @brief Command response structure
 */
struct OdinCommandResponse {
  OdinDeviceHandle device = kInvalidDeviceHandle;  ///< Device handle
  uint16_t seq = 0;                                ///< Sequence number
  uint16_t cmd_id = 0;                             ///< Command ID
  std::vector<uint8_t> payload;                    ///< Response payload
};

/**
 * @brief Synchronous command response structure
 */
struct OdinCommandSyncResponse {
  OdinResult result;             ///< Operation result
  OdinCommandResponse response;  ///< Command response
};

/** @brief Command response callback type */
using OdinCommandCallback = void (*)(OdinResult result, const OdinCommandResponse *response,
                                     void *client_data);

}  // namespace sdk
}  // namespace odin

#endif  // ODIN_LIDAR_INNER_H_