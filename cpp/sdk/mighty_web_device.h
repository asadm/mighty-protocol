#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "mighty_device_io.h"
#include "../third_party/httplib.h"

namespace mighty_protocol {
namespace sdk {

struct MightyWebDeviceOptions {
  std::string base_url;
  std::string stream_path = "/stream";
  std::string command_path = "/command";
  std::map<std::string, std::string> headers;
  int connect_timeout_ms = 5000;
  int read_timeout_ms = 1000;
  int write_timeout_ms = 5000;
};

class MightyWebDevice : public MightyDeviceIO {
 public:
  explicit MightyWebDevice(const MightyWebDeviceOptions& options)
      : options_(options) {
    std::string error;
    if (!parse_base_url(options_.base_url, &origin_, &base_path_prefix_, &error)) {
      init_error_ = error;
      return;
    }
    stream_path_ = join_path(base_path_prefix_, options_.stream_path.empty() ? "/stream" : options_.stream_path);
    command_path_ = join_path(base_path_prefix_, options_.command_path.empty() ? "/command" : options_.command_path);
    is_valid_ = true;
  }

  bool connect(BytesCallback on_bytes, std::string* error) override {
    if (!on_bytes) {
      if (error) *error = "connect requires a valid bytes callback";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(state_mu_);
      if (connected_) {
        if (error) *error = "stream already connected";
        return false;
      }
      connected_ = true;
      stop_requested_ = false;
    }

    if (!is_valid_) {
      std::lock_guard<std::mutex> lock(state_mu_);
      connected_ = false;
      if (error) *error = init_error_;
      return false;
    }

    httplib::Client client(origin_);
    configure_client(client);

    {
      std::lock_guard<std::mutex> lock(state_mu_);
      active_stream_client_ = &client;
    }

    httplib::Headers headers;
    headers.emplace("Accept", "application/octet-stream");
    for (const auto& kv : options_.headers) {
      headers.emplace(kv.first, kv.second);
    }

    std::string stream_error;

    auto result = client.Get(
        stream_path_,
        headers,
        [&](const httplib::Response& response) {
          if (response.status != 200) {
            stream_error = "stream request failed (" + std::to_string(response.status) + ")";
            return false;
          }
          return true;
        },
        [&](const char* data, size_t len) {
          if (should_stop()) return false;
          if (len > 0) {
            on_bytes(reinterpret_cast<const uint8_t*>(data), len);
          }
          return true;
        });

    const bool stopped_by_user = should_stop();

    {
      std::lock_guard<std::mutex> lock(state_mu_);
      active_stream_client_ = nullptr;
      connected_ = false;
    }

    if (stopped_by_user) return true;

    if (!stream_error.empty()) {
      if (error) *error = stream_error;
      return false;
    }

    if (!result) {
      if (error) {
        *error = "stream transport error (" + httplib::to_string(result.error()) + ")";
      }
      return false;
    }

    if (result->status != 200) {
      if (error) {
        *error = "stream request failed (" + std::to_string(result->status) + ")";
      }
      return false;
    }

    return true;
  }

  void disconnect() override {
    std::lock_guard<std::mutex> lock(state_mu_);
    stop_requested_ = true;
    if (active_stream_client_ != nullptr) {
      active_stream_client_->stop();
    }
  }

  bool send_command_payload(const std::vector<uint8_t>& cmd_payload,
                            std::vector<uint8_t>* response_payload,
                            std::string* error) override {
    if (!is_valid_) {
      if (error) *error = init_error_;
      return false;
    }

    httplib::Client client(origin_);
    configure_client(client);

    httplib::Headers headers;
    headers.emplace("Accept", "application/octet-stream");
    for (const auto& kv : options_.headers) {
      headers.emplace(kv.first, kv.second);
    }

    const char* body_ptr = cmd_payload.empty() ? "" : reinterpret_cast<const char*>(cmd_payload.data());
    auto result = client.Post(command_path_, headers, body_ptr, cmd_payload.size(), "application/octet-stream");
    if (!result) {
      if (error) {
        *error = "command transport error (" + httplib::to_string(result.error()) + ")";
      }
      return false;
    }

    if (result->status != 200) {
      if (error) {
        *error = "command request failed (" + std::to_string(result->status) + ")";
      }
      return false;
    }

    if (response_payload) {
      const std::string& body = result->body;
      response_payload->assign(body.begin(), body.end());
    }
    return true;
  }

  DeviceInfo get_info() const override {
    DeviceInfo info;
    info.transport = "http";
    info.source = options_.base_url;
    return info;
  }

 private:
  static bool parse_base_url(const std::string& base_url,
                             std::string* origin,
                             std::string* path_prefix,
                             std::string* error) {
    if (base_url.empty()) {
      if (error) *error = "base_url is required";
      return false;
    }

    const bool has_http = base_url.rfind("http://", 0) == 0;
    const bool has_https = base_url.rfind("https://", 0) == 0;
    if (!has_http && !has_https) {
      if (error) *error = "base_url must start with http:// or https://";
      return false;
    }

    const size_t scheme_end = base_url.find("://") + 3;
    const size_t slash = base_url.find('/', scheme_end);
    if (slash == std::string::npos) {
      *origin = base_url;
      *path_prefix = "";
    } else {
      *origin = base_url.substr(0, slash);
      *path_prefix = base_url.substr(slash);
    }

    if (origin->empty()) {
      if (error) *error = "invalid base_url";
      return false;
    }

    if (!path_prefix->empty() && path_prefix->size() > 1 && path_prefix->back() == '/') {
      path_prefix->pop_back();
    }

    return true;
  }

  static std::string join_path(const std::string& prefix, const std::string& path) {
    const std::string normalized = normalize_path(path);
    if (prefix.empty() || prefix == "/") return normalized;
    if (normalized == "/") return prefix;
    if (prefix.back() == '/') return prefix.substr(0, prefix.size() - 1) + normalized;
    return prefix + normalized;
  }

  static std::string normalize_path(const std::string& path) {
    if (path.empty()) return "/";
    if (path.front() == '/') return path;
    return "/" + path;
  }

  static std::chrono::milliseconds clamp_timeout_ms(int value_ms) {
    if (value_ms <= 0) return std::chrono::milliseconds(1);
    return std::chrono::milliseconds(value_ms);
  }

  void configure_client(httplib::Client& client) const {
    client.set_connection_timeout(clamp_timeout_ms(options_.connect_timeout_ms));
    client.set_read_timeout(clamp_timeout_ms(options_.read_timeout_ms));
    client.set_write_timeout(clamp_timeout_ms(options_.write_timeout_ms));
    client.set_keep_alive(true);
    client.set_follow_location(true);
  }

  bool should_stop() const {
    std::lock_guard<std::mutex> lock(state_mu_);
    return stop_requested_;
  }

  MightyWebDeviceOptions options_;
  std::string origin_;
  std::string base_path_prefix_;
  std::string stream_path_;
  std::string command_path_;

  bool is_valid_ = false;
  std::string init_error_;

  mutable std::mutex state_mu_;
  bool connected_ = false;
  bool stop_requested_ = false;
  httplib::Client* active_stream_client_ = nullptr;
};

}  // namespace sdk
}  // namespace mighty_protocol
