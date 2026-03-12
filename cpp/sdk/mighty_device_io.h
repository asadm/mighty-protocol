#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mighty_protocol {
namespace sdk {

struct DeviceInfo {
  std::string transport;
  std::string source;
};

class MightyDeviceIO {
 public:
  using BytesCallback = std::function<void(const uint8_t* data, size_t len)>;

  virtual ~MightyDeviceIO() = default;

  // Blocking stream loop. Returns false on transport errors.
  virtual bool connect(BytesCallback on_bytes, std::string* error) = 0;
  virtual void disconnect() = 0;

  // Command roundtrip path. Optional for transports that cannot send commands.
  virtual bool send_command_payload(const std::vector<uint8_t>& cmd_payload,
                                    std::vector<uint8_t>* response_payload,
                                    std::string* error) {
    (void)cmd_payload;
    if (response_payload) response_payload->clear();
    if (error) *error = "device does not support command request/response";
    return false;
  }

  virtual DeviceInfo get_info() const = 0;
};

}  // namespace sdk
}  // namespace mighty_protocol
