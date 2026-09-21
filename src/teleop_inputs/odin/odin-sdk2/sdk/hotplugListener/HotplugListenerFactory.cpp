#include "HotplugListenerFactory.h"
#include "CompositeHotplugListener/CompositeHotplugListener.h"
#include "UdpHotplugListener/UdpHotplugListener.h"

namespace odin {
namespace sdk {

std::unique_ptr<IHotplugListener> HotplugListenerFactory::CreateDefault() {
  auto composite = std::unique_ptr<CompositeHotplugListener>(new CompositeHotplugListener());
  
  // Add UDP listener for network devices
  composite->AddListener(std::unique_ptr<IHotplugListener>(new UdpHotplugListener()));
  
  // TODO: Add USB listener for USB devices when implemented
  // composite->AddListener(std::unique_ptr<IHotplugListener>(new UsbHotplugListener()));
  
  return composite;
}

}  // namespace sdk
}  // namespace odin
