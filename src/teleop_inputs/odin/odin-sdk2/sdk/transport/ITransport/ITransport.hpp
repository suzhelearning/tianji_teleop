#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace odin {
namespace sdk {

/* ============================================================================
 * Abstract Transport Interface
 * ============================================================================
 *
 * Design Goals:
 *   - Transport-agnostic communication layer
 *   - Support UDP, TCP, USB, Serial, etc.
 *   - Easy to extend for new transport types
 */

/**
 * @brief Abstract transport address base class
 *
 * Each transport type derives its own address class.
 * This allows transport-agnostic address handling.
 */
struct ITransportAddress {
  virtual ~ITransportAddress() = default;
  virtual std::string ToString() const = 0;
  virtual std::unique_ptr<ITransportAddress> Clone() const = 0;
};

/**
 * @brief Network address for UDP/TCP transports
 */
struct NetworkAddress : public ITransportAddress {
  std::string ip;
  uint16_t port = 0;

  NetworkAddress() = default;
  NetworkAddress(const std::string& ip_, uint16_t port_) : ip(ip_), port(port_) {}

  std::string ToString() const override { return ip + ":" + std::to_string(port); }

  std::unique_ptr<ITransportAddress> Clone() const override {
    return std::unique_ptr<ITransportAddress>(new NetworkAddress(ip, port));
  }
};

/**
 * @brief USB/Serial address for USB transports (reserved for USB)
 */
struct UsbAddress : public ITransportAddress {
  std::string device_path;
  uint16_t vendor_id = 0;
  uint16_t product_id = 0;

  UsbAddress() = default;
  UsbAddress(const std::string& path, uint16_t vid = 0, uint16_t pid = 0)
      : device_path(path), vendor_id(vid), product_id(pid) {}

  std::string ToString() const override { return device_path; }

  std::unique_ptr<ITransportAddress> Clone() const override {
    return std::unique_ptr<ITransportAddress>(new UsbAddress(device_path, vendor_id, product_id));
  }
};

/**
 * @brief Received data callback
 * @param data     Received data buffer
 * @param length   Data length
 * @param from     Source address (cast to specific address type if needed)
 */
using TransportReceiveCallback =
    std::function<void(const uint8_t* data, size_t length, const ITransportAddress& from)>;

/**
 * @brief Abstract transport interface
 */
class ITransport {
 public:
  virtual ~ITransport() = default;

  /**
   * @brief Open/bind the transport with local address
   * @param local_address  Local address to bind (transport-specific)
   * @return true on success
   */
  virtual bool Open(const ITransportAddress& local_address) = 0;

  /**
   * @brief Set remote target address for sending
   * @param remote_address  Remote address to send to (transport-specific)
   */
  virtual void SetRemoteTarget(const ITransportAddress& remote_address) = 0;

  /**
   * @brief Close the transport
   */
  virtual void Close() = 0;

  /**
   * @brief Check if transport is open
   */
  virtual bool IsOpen() const = 0;

  /**
   * @brief Send data to remote target (uses address set by SetRemoteTarget)
   * @param data    Data buffer to send
   * @param length  Data length
   * @return Bytes sent, or -1 on error
   */
  virtual int Send(const uint8_t* data, size_t length) = 0;

  /**
   * @brief Set receive callback (called when data arrives)
   * @param callback  Callback function
   */
  virtual void SetReceiveCallback(TransportReceiveCallback callback) = 0;

  /**
   * @brief Start receiving loop (non-blocking, spawns internal thread)
   * @return true on success
   */
  virtual bool StartReceiving() = 0;

  /**
   * @brief Stop receiving loop
   */
  virtual void StopReceiving() = 0;

  // Transport metadata
  virtual const char* GetTransportName() const = 0;
};

/**
 * @brief Factory function type for creating transports
 */
using TransportFactory = std::function<std::unique_ptr<ITransport>()>;

}  // namespace sdk
}  // namespace odin
