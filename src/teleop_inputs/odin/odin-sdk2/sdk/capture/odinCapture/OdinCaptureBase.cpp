#include "OdinCaptureBase.hpp"

#include "TcpTransport.hpp"
#include "UdpTransport.hpp"
#include "logger.h"

namespace odin {
namespace sdk {

NetworkCapture::NetworkCapture(bool use_tcp) : use_tcp_(use_tcp) {}

NetworkCapture::~NetworkCapture() {
  Stop();
}

bool NetworkCapture::Start(const ITransportAddress& address, RawDataCallback callback) {
  if (running_) {
    LOG_WARN("NetworkCapture::Start: Already running\n");
    return true;
  }

  if (!callback) {
    LOG_ERROR("NetworkCapture::Start: callback is null\n");
    return false;
  }

  callback_ = callback;

  // Create transport based on type
  if (use_tcp_) {
    transport_.reset(new TcpTransport());
  } else {
    transport_.reset(new UdpTransport());
  }

  if (!transport_->Open(address)) {
    LOG_ERROR("NetworkCapture::Start: Open failed\n");
    return false;
  }

  // Set receive callback - pass raw data directly to user callback
  transport_->SetReceiveCallback(
      [this](const uint8_t* data, size_t len, const ITransportAddress& addr) {
        if (callback_) {
          callback_(data, len, addr);
        }
      });

  if (!transport_->StartReceiving()) {
    LOG_ERROR("NetworkCapture::Start: StartReceiving failed\n");
    transport_->Close();
    return false;
  }

  running_ = true;
  LOG_INFO("NetworkCapture::Start: %s, %s\n", 
           address.ToString().c_str(), use_tcp_ ? "TCP" : "UDP");
  return true;
}

void NetworkCapture::Stop() {
  if (!running_) {
    return;
  }

  running_ = false;

  if (transport_) {
    transport_->StopReceiving();
    transport_->Close();
    transport_.reset();
  }

  callback_ = nullptr;
  LOG_INFO("NetworkCapture::Stop\n");
}

bool NetworkCapture::IsRunning() const {
  return running_;
}

}  // namespace sdk
}  // namespace odin
