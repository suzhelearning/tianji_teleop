#pragma once

#include "IHotplugListener/IHotplugListener.h"
#include <memory>

namespace odin {
namespace sdk {

/**
 * @brief Internal factory for creating hotplug listeners
 * 
 * This is an internal SDK class, not exposed in public API.
 * Provides abstraction for creating different types of hotplug listeners
 * based on the connection type (network, USB, etc.)
 */
class HotplugListenerFactory {
 public:
  /**
   * @brief Create the default hotplug listener for current platform
   * @return Unique pointer to the created listener
   */
  static std::unique_ptr<IHotplugListener> CreateDefault();

  // Prevent instantiation
  HotplugListenerFactory() = delete;
};

}  // namespace sdk
}  // namespace odin
