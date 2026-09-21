#pragma once

#include <cstdint>
#include <functional>

// Forward declaration
namespace odin {
namespace sdk {
class ITransportAddress;
}
}  // namespace odin

namespace odin {
namespace sdk {

// =============================================================================
// Raw Data Callback - Protocol agnostic
// =============================================================================

/**
 * @brief Raw data callback type
 * 
 * Called when raw data is received from transport layer.
 * Protocol parsing is NOT done at this level.
 * 
 * @param data Raw data buffer
 * @param length Data length in bytes
 * @param source Source address of the data
 */
using RawDataCallback = std::function<void(const uint8_t* data, size_t length,
                                            const ITransportAddress& source)>;

// =============================================================================
// Data Capture Abstract Interface
// =============================================================================

/**
 * @brief Data capture abstract interface
 * 
 * Protocol-agnostic interface for data capture. Each instance handles one
 * data channel. Only provides raw data callback - protocol parsing is done
 * by upper layers.
 * 
 * Design:
 * - One instance per data channel
 * - Start(address, callback) to begin capture with raw data callback
 * - Stop() to end capture
 * - Protocol parsing is NOT done here - handled by device layer
 */
class ICapture {
 public:
  virtual ~ICapture() = default;

  /**
   * @brief Start data capture with raw data callback
   * @param address Transport address to bind/connect
   * @param callback Raw data callback (called for each received packet)
   * @return true if started successfully
   */
  virtual bool Start(const ITransportAddress& address, RawDataCallback callback) = 0;
  
  /**
   * @brief Stop data capture
   */
  virtual void Stop() = 0;
  
  /**
   * @brief Check if capture is running
   * @return true if capture is active
   */
  virtual bool IsRunning() const = 0;
};

}  // namespace sdk
}  // namespace odin
